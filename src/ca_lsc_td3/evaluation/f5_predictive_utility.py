"""Leakage-safe F5 comparison of airspeed and capability feature sets.

The implementation intentionally uses a small NumPy logistic regressor.  It
keeps Model A and Model B in the same linear model family without adding a new
runtime dependency to the frozen ``vtol_nav`` environment.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np


MODEL_FEATURES = {
    'model_a': ('va_mps', 'lambda_exec'),
    'model_b': ('va_mps', 'eta_l', 'eta_c', 'lambda_exec'),
}


def _json_safe(value: object) -> object:
    if isinstance(value, (np.floating, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [_json_safe(item) for item in value]
    return value


def _sigmoid(logits: np.ndarray) -> np.ndarray:
    logits = np.clip(logits, -40.0, 40.0)
    return 1.0 / (1.0 + np.exp(-logits))


def fit_logistic(
    features: np.ndarray,
    labels: np.ndarray,
    *,
    l2: float = 1.0,
    maximum_iterations: int = 100,
) -> np.ndarray:
    """Fit an unweighted L2 logistic model with damped Newton updates."""
    if features.ndim != 2 or labels.ndim != 1:
        raise ValueError('features must be 2D and labels must be 1D')
    if len(features) != len(labels) or len(labels) == 0:
        raise ValueError('features and labels must be nonempty and aligned')
    if len(np.unique(labels)) != 2:
        raise ValueError('training labels must contain both classes')
    design = np.column_stack((np.ones(len(features)), features))
    coefficients = np.zeros(design.shape[1], dtype=float)
    regularizer = np.eye(design.shape[1], dtype=float) * l2
    regularizer[0, 0] = 0.0
    for _ in range(maximum_iterations):
        probability = _sigmoid(design @ coefficients)
        gradient = design.T @ (probability - labels) + regularizer @ coefficients
        weights = np.maximum(probability * (1.0 - probability), 1.0e-8)
        hessian = design.T @ (design * weights[:, None]) + regularizer
        try:
            step = np.linalg.solve(hessian, gradient)
        except np.linalg.LinAlgError:
            step = np.linalg.pinv(hessian) @ gradient
        coefficients -= step
        if float(np.linalg.norm(step)) < 1.0e-8:
            break
    return coefficients


def _standardize(
    train: np.ndarray, test: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    mean = np.mean(train, axis=0)
    scale = np.std(train, axis=0)
    scale = np.where(scale > 1.0e-12, scale, 1.0)
    return (train - mean) / scale, (test - mean) / scale, mean, scale


def _predict(features: np.ndarray, coefficients: np.ndarray) -> np.ndarray:
    design = np.column_stack((np.ones(len(features)), features))
    return _sigmoid(design @ coefficients)


def roc_auc(labels: np.ndarray, probabilities: np.ndarray) -> float:
    positive = int(np.sum(labels == 1))
    negative = int(np.sum(labels == 0))
    if positive == 0 or negative == 0:
        return math.nan
    order = np.argsort(probabilities, kind='mergesort')
    ranks = np.empty(len(probabilities), dtype=float)
    index = 0
    while index < len(order):
        end = index + 1
        while (
            end < len(order)
            and probabilities[order[end]] == probabilities[order[index]]
        ):
            end += 1
        ranks[order[index:end]] = 0.5 * (index + 1 + end)
        index = end
    positive_rank_sum = float(np.sum(ranks[labels == 1]))
    return (
        positive_rank_sum - positive * (positive + 1) / 2.0
    ) / (positive * negative)


def pr_auc(labels: np.ndarray, probabilities: np.ndarray) -> float:
    positive = int(np.sum(labels == 1))
    if positive == 0:
        return math.nan
    order = np.argsort(-probabilities, kind='mergesort')
    sorted_labels = labels[order]
    true_positive = np.cumsum(sorted_labels == 1)
    false_positive = np.cumsum(sorted_labels == 0)
    precision = true_positive / np.maximum(true_positive + false_positive, 1)
    return float(np.sum(precision[sorted_labels == 1]) / positive)


def f1_score(labels: np.ndarray, predicted: np.ndarray) -> float:
    tp = int(np.sum((labels == 1) & (predicted == 1)))
    fp = int(np.sum((labels == 0) & (predicted == 1)))
    fn = int(np.sum((labels == 1) & (predicted == 0)))
    denominator = 2 * tp + fp + fn
    return 2.0 * tp / denominator if denominator else 0.0


def _recall(labels: np.ndarray, predicted: np.ndarray, positive: int) -> float:
    tp = int(np.sum((labels == positive) & (predicted == positive)))
    fn = int(np.sum((labels == positive) & (predicted != positive)))
    denominator = tp + fn
    return tp / denominator if denominator else math.nan


def _mcc(labels: np.ndarray, predicted: np.ndarray) -> float:
    tp = int(np.sum((labels == 1) & (predicted == 1)))
    tn = int(np.sum((labels == 0) & (predicted == 0)))
    fp = int(np.sum((labels == 0) & (predicted == 1)))
    fn = int(np.sum((labels == 1) & (predicted == 0)))
    denominator = math.sqrt((tp + fp) * (tp + fn) * (tn + fp) * (tn + fn))
    return (tp * tn - fp * fn) / denominator if denominator else 0.0


def select_unsafe_alarm_threshold(
    labels: np.ndarray,
    safe_probabilities: np.ndarray,
    *,
    max_safe_false_alarm_rate: float = 0.05,
) -> float:
    """Select a safe-probability threshold for warning on unsafe states.

    Labels use the F5 convention where 1 means safe.  An unsafe alarm is raised
    when ``P(safe) < threshold``.  The threshold is selected on the training fold
    only by maximizing unsafe recall subject to the requested false-alarm budget
    on actually safe samples.
    """
    if not 0.0 <= max_safe_false_alarm_rate <= 1.0:
        raise ValueError('max_safe_false_alarm_rate must be in [0, 1]')
    candidates = np.unique(np.concatenate((
        np.linspace(0.0, 1.0, 201),
        np.quantile(safe_probabilities, np.linspace(0.0, 1.0, 101)),
    )))
    best = (float('-inf'), float('-inf'), float('-inf'), 0.0)
    safe_mask = labels == 1
    unsafe_mask = labels == 0
    for threshold in candidates:
        unsafe_alarm = safe_probabilities < threshold
        false_alarm = (
            float(np.mean(unsafe_alarm[safe_mask]))
            if np.any(safe_mask) else math.nan
        )
        if not math.isfinite(false_alarm) or false_alarm > max_safe_false_alarm_rate:
            continue
        unsafe_recall = (
            float(np.mean(unsafe_alarm[unsafe_mask]))
            if np.any(unsafe_mask) else math.nan
        )
        if not math.isfinite(unsafe_recall):
            continue
        candidate = (
            unsafe_recall,
            -false_alarm,
            float(threshold),
            float(threshold),
        )
        if candidate > best:
            best = candidate
    return best[3]


def select_f1_threshold(labels: np.ndarray, probabilities: np.ndarray) -> float:
    candidates = np.unique(np.concatenate((
        np.linspace(0.0, 1.0, 201),
        np.quantile(probabilities, np.linspace(0.0, 1.0, 101)),
        np.array([0.5]),
    )))
    best = (float('-inf'), float('-inf'), 0.5)
    for threshold in candidates:
        score = f1_score(labels, (probabilities >= threshold).astype(int))
        # Prefer a threshold closer to 0.5, then the larger threshold.
        candidate = (score, -abs(float(threshold) - 0.5), float(threshold))
        if candidate > best:
            best = candidate
    return best[2]


def metric_bundle(
    labels: np.ndarray,
    probabilities: np.ndarray,
    thresholds: np.ndarray | float,
    unsafe_alarm_thresholds: np.ndarray | float | None = None,
) -> dict[str, float]:
    predicted = (probabilities >= thresholds).astype(int)
    unsafe_predicted = 1 - predicted
    unsafe_labels = 1 - labels
    unsafe_probability = 1.0 - probabilities
    safe_recall = _recall(labels, predicted, 1)
    unsafe_recall = _recall(unsafe_labels, unsafe_predicted, 1)
    safe_f1 = f1_score(labels, predicted)
    unsafe_f1 = f1_score(unsafe_labels, unsafe_predicted)
    false_alarm_mask = (labels == 1) & (unsafe_predicted == 1)
    safe_count = int(np.sum(labels == 1))
    unsafe_false_alarm_rate = (
        int(np.sum(false_alarm_mask)) / safe_count if safe_count else math.nan
    )
    if unsafe_alarm_thresholds is None:
        unsafe_alarm_thresholds = select_unsafe_alarm_threshold(
            labels, probabilities,
        )
    unsafe_alarm = (probabilities < unsafe_alarm_thresholds).astype(int)
    unsafe_alarm_recall = (
        float(np.mean(unsafe_alarm[labels == 0]))
        if np.any(labels == 0) else math.nan
    )
    unsafe_alarm_false_alarm = (
        float(np.mean(unsafe_alarm[labels == 1]))
        if np.any(labels == 1) else math.nan
    )
    return {
        'roc_auc': roc_auc(labels, probabilities),
        'pr_auc': pr_auc(labels, probabilities),
        'f1': safe_f1,
        'brier': float(np.mean((probabilities - labels) ** 2)),
        'unsafe_roc_auc': roc_auc(unsafe_labels, unsafe_probability),
        'unsafe_pr_auc': pr_auc(unsafe_labels, unsafe_probability),
        'safe_recall': safe_recall,
        'unsafe_recall': unsafe_recall,
        'unsafe_false_alarm_rate': unsafe_false_alarm_rate,
        'balanced_accuracy': float(np.nanmean([safe_recall, unsafe_recall])),
        'macro_f1': float(np.nanmean([safe_f1, unsafe_f1])),
        'mcc': _mcc(labels, predicted),
        'unsafe_recall_at_5pct_safe_false_alarm': unsafe_alarm_recall,
        'safe_false_alarm_rate_at_unsafe_threshold': unsafe_alarm_false_alarm,
    }


def make_group_folds(
    labels: np.ndarray,
    groups: np.ndarray,
    *,
    n_splits: int = 5,
) -> list[np.ndarray]:
    """Greedily balance samples and positive labels without splitting groups."""
    unique = sorted(set(str(item) for item in groups))
    if len(unique) < n_splits:
        raise ValueError(f'need at least {n_splits} condition groups')
    statistics = []
    minority_label = 1 if int(np.sum(labels == 1)) < int(np.sum(labels == 0)) else 0
    for group in unique:
        indices = np.flatnonzero(groups == group)
        positives = int(np.sum(labels[indices]))
        minority = int(np.sum(labels[indices] == minority_label))
        statistics.append((
            group, indices, len(indices), positives, minority,
            minority / len(indices),
        ))
    # Seed folds with minority-bearing groups whenever possible.  This avoids
    # a formally valid grouped split whose held-out fold contains only the
    # majority class.
    statistics.sort(key=lambda item: (
        item[4] == 0, -item[5], -item[4], -item[2], item[0]
    ))
    fold_groups: list[list[str]] = [[] for _ in range(n_splits)]
    fold_count = np.zeros(n_splits, dtype=float)
    fold_positive = np.zeros(n_splits, dtype=float)
    target_count = len(labels) / n_splits
    target_positive = float(np.sum(labels)) / n_splits
    for index, (group, _, count, positive, _, _) in enumerate(statistics):
        if index < n_splits:
            choice = index
        else:
            costs = []
            for fold in range(n_splits):
                count_cost = ((fold_count[fold] + count) - target_count) ** 2
                positive_cost = (
                    (fold_positive[fold] + positive) - target_positive
                ) ** 2
                costs.append((
                    count_cost / max(target_count**2, 1.0)
                    + positive_cost / max(target_positive**2, 1.0),
                    len(fold_groups[fold]), fold,
                ))
            choice = min(costs)[2]
        fold_groups[choice].append(group)
        fold_count[choice] += count
        fold_positive[choice] += positive
    return [
        np.flatnonzero(np.isin(groups, group_names))
        for group_names in fold_groups
    ]


def _calibration_curve(
    labels: np.ndarray, probabilities: np.ndarray, bins: int = 10,
) -> list[dict[str, float | int]]:
    output = []
    edges = np.linspace(0.0, 1.0, bins + 1)
    for index in range(bins):
        if index == bins - 1:
            selected = (probabilities >= edges[index]) & (probabilities <= edges[index + 1])
        else:
            selected = (probabilities >= edges[index]) & (probabilities < edges[index + 1])
        if not np.any(selected):
            continue
        output.append({
            'bin_lower': float(edges[index]),
            'bin_upper': float(edges[index + 1]),
            'sample_count': int(np.sum(selected)),
            'mean_predicted_probability': float(np.mean(probabilities[selected])),
            'observed_safe_fraction': float(np.mean(labels[selected])),
        })
    return output


def _read_dataset(path: Path) -> tuple[list[dict[str, str]], np.ndarray, np.ndarray]:
    with path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    required = {
        'condition_group', 'label_safe_2s',
        *MODEL_FEATURES['model_a'], *MODEL_FEATURES['model_b'],
    }
    if not rows or not required.issubset(rows[0]):
        missing = sorted(required - (set(rows[0]) if rows else set()))
        raise ValueError(f'F5 dataset missing columns: {missing}')
    retained = []
    labels = []
    groups = []
    for row in rows:
        try:
            label = int(row['label_safe_2s'])
            values = [float(row[key]) for key in MODEL_FEATURES['model_b']]
        except (TypeError, ValueError, KeyError):
            continue
        if label not in (0, 1) or not all(math.isfinite(value) for value in values):
            continue
        retained.append(row)
        labels.append(label)
        groups.append(row['condition_group'])
    return retained, np.asarray(labels, dtype=int), np.asarray(groups, dtype=object)


def _bootstrap_deltas(
    labels: np.ndarray,
    groups: np.ndarray,
    probabilities: dict[str, np.ndarray],
    predicted: dict[str, np.ndarray],
    unsafe_alarm_predicted: dict[str, np.ndarray],
    *,
    repetitions: int,
    seed: int,
) -> dict[str, object]:
    rng = np.random.default_rng(seed)
    unique = np.asarray(sorted(set(str(item) for item in groups)), dtype=object)
    distributions = {
        key: [] for key in (
            'roc_auc', 'pr_auc', 'f1', 'brier', 'unsafe_pr_auc',
            'balanced_accuracy', 'macro_f1', 'mcc',
            'unsafe_recall_at_5pct_safe_false_alarm',
        )
    }
    for _ in range(repetitions):
        sampled = rng.choice(unique, size=len(unique), replace=True)
        blocks = [np.flatnonzero(groups == group) for group in sampled]
        indices = np.concatenate(blocks) if blocks else np.asarray([], dtype=int)
        if len(indices) == 0 or len(np.unique(labels[indices])) < 2:
            continue
        metrics = {}
        for model in MODEL_FEATURES:
            selected_labels = labels[indices]
            selected_probability = probabilities[model][indices]
            selected_predicted = predicted[model][indices]
            selected_unsafe_predicted = 1 - selected_predicted
            unsafe_labels = 1 - selected_labels
            safe_recall = _recall(selected_labels, selected_predicted, 1)
            unsafe_recall = _recall(
                unsafe_labels, selected_unsafe_predicted, 1,
            )
            safe_f1 = f1_score(selected_labels, selected_predicted)
            unsafe_f1 = f1_score(unsafe_labels, selected_unsafe_predicted)
            alarm = unsafe_alarm_predicted[model][indices]
            alarm_recall = (
                float(np.mean(alarm[selected_labels == 0]))
                if np.any(selected_labels == 0) else math.nan
            )
            metrics[model] = {
                'roc_auc': roc_auc(selected_labels, selected_probability),
                'pr_auc': pr_auc(selected_labels, selected_probability),
                'f1': safe_f1,
                'brier': float(np.mean(
                    (selected_probability - selected_labels) ** 2
                )),
                'unsafe_pr_auc': pr_auc(
                    unsafe_labels, 1.0 - selected_probability,
                ),
                'balanced_accuracy': float(np.nanmean([
                    safe_recall, unsafe_recall,
                ])),
                'macro_f1': float(np.nanmean([safe_f1, unsafe_f1])),
                'mcc': _mcc(selected_labels, selected_predicted),
                'unsafe_recall_at_5pct_safe_false_alarm': alarm_recall,
            }
        for key in distributions:
            distributions[key].append(
                metrics['model_b'][key] - metrics['model_a'][key]
            )
    output: dict[str, object] = {'valid_repetitions': 0, 'metrics': {}}
    for key, values in distributions.items():
        array = np.asarray(values, dtype=float)
        output['valid_repetitions'] = max(output['valid_repetitions'], len(array))
        output['metrics'][key] = {
            'mean_delta_b_minus_a': float(np.mean(array)) if len(array) else math.nan,
            'ci95_lower': float(np.quantile(array, 0.025)) if len(array) else math.nan,
            'ci95_upper': float(np.quantile(array, 0.975)) if len(array) else math.nan,
        }
    return output


def _condition_macro_summary(
    labels: np.ndarray,
    groups: np.ndarray,
    probabilities: dict[str, np.ndarray],
    thresholds: dict[str, np.ndarray],
    unsafe_alarm_thresholds: dict[str, np.ndarray],
) -> dict[str, object]:
    unique = sorted(set(str(item) for item in groups))
    per_condition: list[dict[str, object]] = []
    metric_names = (
        'roc_auc', 'pr_auc', 'f1', 'brier', 'unsafe_pr_auc',
        'safe_recall', 'unsafe_recall', 'unsafe_false_alarm_rate',
        'balanced_accuracy', 'macro_f1', 'mcc',
        'unsafe_recall_at_5pct_safe_false_alarm',
        'safe_false_alarm_rate_at_unsafe_threshold',
    )
    for group in unique:
        indices = np.flatnonzero(groups == group)
        item: dict[str, object] = {
            'condition_group': group,
            'sample_count': int(len(indices)),
            'safe_count': int(np.sum(labels[indices] == 1)),
            'unsafe_count': int(np.sum(labels[indices] == 0)),
            'models': {},
        }
        for model in MODEL_FEATURES:
            item['models'][model] = metric_bundle(
                labels[indices],
                probabilities[model][indices],
                thresholds[model][indices],
                unsafe_alarm_thresholds[model][indices],
            )
        item['delta_b_minus_a'] = {
            key: (
                item['models']['model_b'][key]
                - item['models']['model_a'][key]
            )
            for key in metric_names
        }
        per_condition.append(item)

    macro: dict[str, object] = {'condition_count': len(per_condition)}
    for model in MODEL_FEATURES:
        macro[model] = {}
        for key in metric_names:
            values = np.asarray([
                condition['models'][model][key]
                for condition in per_condition
            ], dtype=float)
            values = values[np.isfinite(values)]
            macro[model][key] = (
                float(np.mean(values)) if len(values) else math.nan
            )
    macro['delta_b_minus_a'] = {
        key: macro['model_b'][key] - macro['model_a'][key]
        for key in metric_names
    }
    macro['per_condition'] = per_condition
    return macro


def evaluate_predictive_utility(
    dataset_csv: Path,
    *,
    n_splits: int = 5,
    l2: float = 1.0,
    bootstrap_repetitions: int = 1000,
    seed: int = 20260906,
) -> dict[str, object]:
    rows, labels, groups = _read_dataset(dataset_csv)
    if len(rows) == 0 or len(np.unique(labels)) < 2:
        raise ValueError('F5 dataset must contain both safe and unsafe samples')
    folds = make_group_folds(labels, groups, n_splits=n_splits)
    probabilities = {
        model: np.full(len(labels), np.nan, dtype=float) for model in MODEL_FEATURES
    }
    thresholds = {
        model: np.full(len(labels), np.nan, dtype=float) for model in MODEL_FEATURES
    }
    unsafe_alarm_thresholds = {
        model: np.full(len(labels), np.nan, dtype=float) for model in MODEL_FEATURES
    }
    fold_reports = []
    for fold_index, test_indices in enumerate(folds):
        train_mask = np.ones(len(labels), dtype=bool)
        train_mask[test_indices] = False
        train_indices = np.flatnonzero(train_mask)
        if len(np.unique(labels[train_indices])) < 2:
            raise ValueError(f'training fold {fold_index} has only one class')
        fold_report: dict[str, object] = {
            'fold': fold_index,
            'train_sample_count': len(train_indices),
            'test_sample_count': len(test_indices),
            'train_condition_count': len(set(groups[train_indices])),
            'test_condition_count': len(set(groups[test_indices])),
            'test_condition_groups': sorted(set(str(item) for item in groups[test_indices])),
            'test_safe_count': int(np.sum(labels[test_indices] == 1)),
            'test_unsafe_count': int(np.sum(labels[test_indices] == 0)),
            'models': {},
        }
        for model, feature_names in MODEL_FEATURES.items():
            all_features = np.asarray([
                [float(row[key]) for key in feature_names] for row in rows
            ], dtype=float)
            train_x, test_x, _, _ = _standardize(
                all_features[train_indices], all_features[test_indices],
            )
            coefficients = fit_logistic(train_x, labels[train_indices], l2=l2)
            train_probability = _predict(train_x, coefficients)
            threshold = select_f1_threshold(labels[train_indices], train_probability)
            unsafe_threshold = select_unsafe_alarm_threshold(
                labels[train_indices], train_probability,
            )
            test_probability = _predict(test_x, coefficients)
            probabilities[model][test_indices] = test_probability
            thresholds[model][test_indices] = threshold
            unsafe_alarm_thresholds[model][test_indices] = unsafe_threshold
            fold_report['models'][model] = {
                'threshold_selected_on_train': threshold,
                'unsafe_alarm_safe_probability_threshold_selected_on_train': (
                    unsafe_threshold
                ),
                **metric_bundle(
                    labels[test_indices], test_probability, threshold,
                    unsafe_threshold,
                ),
            }
        fold_report['delta_b_minus_a'] = {
            key: (
                fold_report['models']['model_b'][key]
                - fold_report['models']['model_a'][key]
            )
            for key in (
                'roc_auc', 'pr_auc', 'f1', 'brier', 'unsafe_pr_auc',
                'balanced_accuracy', 'macro_f1', 'mcc',
                'unsafe_recall_at_5pct_safe_false_alarm',
            )
        }
        fold_reports.append(fold_report)

    models: dict[str, object] = {}
    predicted = {}
    for model, feature_names in MODEL_FEATURES.items():
        predicted[model] = (
            probabilities[model] >= thresholds[model]
        ).astype(int)
        models[model] = {
            'features': list(feature_names),
            **metric_bundle(
                labels, probabilities[model], thresholds[model],
                unsafe_alarm_thresholds[model],
            ),
            'calibration_curve': _calibration_curve(labels, probabilities[model]),
        }
    delta = {
        key: models['model_b'][key] - models['model_a'][key]
        for key in (
            'roc_auc', 'pr_auc', 'f1', 'brier', 'unsafe_pr_auc',
            'balanced_accuracy', 'macro_f1', 'mcc',
            'unsafe_recall_at_5pct_safe_false_alarm',
        )
    }
    unsafe_alarm_predicted = {
        model: (probabilities[model] < unsafe_alarm_thresholds[model]).astype(int)
        for model in MODEL_FEATURES
    }
    condition_macro = _condition_macro_summary(
        labels, groups, probabilities, thresholds, unsafe_alarm_thresholds,
    )
    bootstrap = _bootstrap_deltas(
        labels, groups, probabilities, predicted, unsafe_alarm_predicted,
        repetitions=bootstrap_repetitions, seed=seed,
    )
    every_test_fold_has_both_classes = all(
        fold['test_safe_count'] > 0 and fold['test_unsafe_count'] > 0
        for fold in fold_reports
    )
    return {
        'analysis_type': 'f5_predictive_utility',
        'dataset_csv': str(dataset_csv.resolve()),
        'sample_count': len(labels),
        'safe_sample_count': int(np.sum(labels == 1)),
        'unsafe_sample_count': int(np.sum(labels == 0)),
        'condition_group_count': len(set(groups)),
        'split_policy': 'condition_group held out; no trajectory crosses folds',
        'n_splits': n_splits,
        'model_family': 'L2 logistic regression with train-fold standardization',
        'f1_threshold_policy': 'maximize F1 on training fold only',
        'l2': l2,
        'model_a': models['model_a'],
        'model_b': models['model_b'],
        'delta_b_minus_a': delta,
        'folds': fold_reports,
        'condition_macro': condition_macro,
        'group_bootstrap': bootstrap,
        'cross_validation_valid': every_test_fold_has_both_classes,
        'directional_evidence': {
            'roc_auc_b_gt_a': delta['roc_auc'] > 0.0,
            'pr_auc_b_gt_a': delta['pr_auc'] > 0.0,
            'brier_b_lt_a': delta['brier'] < 0.0,
            'all_primary_directions_supported': bool(
                every_test_fold_has_both_classes
                and delta['roc_auc'] > 0.0
                and delta['pr_auc'] > 0.0
                and delta['brier'] < 0.0
            ),
        },
    }


def write_predictive_outputs(payload: dict[str, object], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / 'f5_predictive_utility.json').write_text(
        json.dumps(_json_safe(payload), indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    with (output_dir / 'f5_predictive_utility_folds.csv').open(
        'w', newline='', encoding='utf-8',
    ) as stream:
        fields = (
            'fold', 'model', 'train_sample_count', 'test_sample_count',
            'train_condition_count', 'test_condition_count',
            'test_condition_groups', 'test_safe_count', 'test_unsafe_count',
            'threshold', 'unsafe_alarm_threshold',
            'roc_auc', 'pr_auc', 'f1', 'brier', 'unsafe_roc_auc',
            'unsafe_pr_auc', 'safe_recall', 'unsafe_recall',
            'unsafe_false_alarm_rate', 'balanced_accuracy', 'macro_f1',
            'mcc', 'unsafe_recall_at_5pct_safe_false_alarm',
            'safe_false_alarm_rate_at_unsafe_threshold',
        )
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for fold in payload['folds']:
            for model in MODEL_FEATURES:
                metrics = fold['models'][model]
                writer.writerow({
                    'fold': fold['fold'], 'model': model,
                    'train_sample_count': fold['train_sample_count'],
                    'test_sample_count': fold['test_sample_count'],
                    'train_condition_count': fold['train_condition_count'],
                    'test_condition_count': fold['test_condition_count'],
                    'test_condition_groups': ' ; '.join(
                        fold['test_condition_groups']
                    ),
                    'test_safe_count': fold['test_safe_count'],
                    'test_unsafe_count': fold['test_unsafe_count'],
                    'threshold': metrics['threshold_selected_on_train'],
                    'unsafe_alarm_threshold': metrics[
                        'unsafe_alarm_safe_probability_threshold_selected_on_train'
                    ],
                    **{
                        key: metrics[key] for key in (
                            'roc_auc', 'pr_auc', 'f1', 'brier',
                            'unsafe_roc_auc', 'unsafe_pr_auc',
                            'safe_recall', 'unsafe_recall',
                            'unsafe_false_alarm_rate', 'balanced_accuracy',
                            'macro_f1', 'mcc',
                            'unsafe_recall_at_5pct_safe_false_alarm',
                            'safe_false_alarm_rate_at_unsafe_threshold',
                        )
                    },
                })


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset_csv', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--folds', type=int, default=5)
    parser.add_argument('--l2', type=float, default=1.0)
    parser.add_argument('--bootstrap', type=int, default=1000)
    parser.add_argument('--seed', type=int, default=20260906)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    payload = evaluate_predictive_utility(
        args.dataset_csv,
        n_splits=args.folds,
        l2=args.l2,
        bootstrap_repetitions=args.bootstrap,
        seed=args.seed,
    )
    write_predictive_outputs(payload, args.output_dir)
    print(json.dumps(_json_safe({
        'sample_count': payload['sample_count'],
        'condition_group_count': payload['condition_group_count'],
        'model_a': payload['model_a'],
        'model_b': payload['model_b'],
        'delta_b_minus_a': payload['delta_b_minus_a'],
        'directional_evidence': payload['directional_evidence'],
    }), indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
