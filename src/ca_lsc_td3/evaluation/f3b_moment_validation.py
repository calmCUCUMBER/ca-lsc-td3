"""F3-B1 elevator aerodynamic moment validation.

The schema-20 instrumented model records the aerodynamic wrench that Gazebo
applies for the elevator LiftDrag element.  The online eta_C calculation uses
the dimensional derivative

    M_e ~= qbar * K_delta * delta_e

where ``K_delta`` is ``elevator_dmoment_ddelta_per_q_m3_config``.  Absolute
GT moment can include a steady alpha / center-of-pressure bias.  Formal
repeated-run cells therefore use run-specific intercepts and a cell-level
``qbar * alpha`` nuisance regressor:

    M_e^GT = b_run + K_delta * (qbar * delta_e)
             + K_alpha * (qbar * alpha).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import fmean, stdev
from typing import Iterable


def _finite(value: object) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def _mean(values: Iterable[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return fmean(finite) if finite else math.nan


def _std(values: Iterable[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return stdev(finite) if len(finite) >= 2 else 0.0 if finite else math.nan


def _measurement_row(row: dict[str, str]) -> bool:
    return (
        _finite(row.get('va_hold_measurement_active')) > 0.5
        and _finite(row.get('va_hold_measurement_complete')) <= 0.5
    )


def _elevator_effective_angle(row: dict[str, str]) -> float:
    for key in (
        'elevator_effective_angle_gt_rad',
        'elevator_control_angle_for_eta_c_rad',
        'elevator_actual_angle_rad',
    ):
        value = _finite(row.get(key))
        if math.isfinite(value):
            return value
    return math.nan


def _rmse(predicted: list[float], actual: list[float]) -> float:
    if not predicted or len(predicted) != len(actual):
        return math.nan
    return math.sqrt(fmean(
        (prediction - truth) ** 2
        for prediction, truth in zip(predicted, actual)
    ))


def _r_squared(predicted: list[float], actual: list[float]) -> float:
    if len(predicted) < 2 or len(predicted) != len(actual):
        return math.nan
    center = fmean(actual)
    total = sum((truth - center) ** 2 for truth in actual)
    if total <= 1.0e-12:
        return math.nan
    residual = sum(
        (prediction - truth) ** 2
        for prediction, truth in zip(predicted, actual)
    )
    return 1.0 - residual / total


def _linear_fit(x_values: list[float], y_values: list[float]) -> tuple[float, float]:
    if len(x_values) < 2 or len(x_values) != len(y_values):
        return math.nan, math.nan
    x_mean = fmean(x_values)
    y_mean = fmean(y_values)
    variance = sum((value - x_mean) ** 2 for value in x_values)
    if variance <= 1.0e-12:
        return math.nan, math.nan
    covariance = sum(
        (x - x_mean) * (y - y_mean)
        for x, y in zip(x_values, y_values)
    )
    slope = covariance / variance
    intercept = y_mean - slope * x_mean
    return slope, intercept


def _solve_3x3(
    matrix: list[list[float]],
    vector: list[float],
) -> tuple[float, float, float]:
    a = [row[:] + [value] for row, value in zip(matrix, vector)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(a[row][column]))
        if abs(a[pivot][column]) <= 1.0e-12:
            return math.nan, math.nan, math.nan
        if pivot != column:
            a[column], a[pivot] = a[pivot], a[column]
        scale = a[column][column]
        for item in range(column, 4):
            a[column][item] /= scale
        for row in range(3):
            if row == column:
                continue
            factor = a[row][column]
            for item in range(column, 4):
                a[row][item] -= factor * a[column][item]
    return a[0][3], a[1][3], a[2][3]


def _two_variable_fit(
    x_delta: list[float],
    x_alpha: list[float],
    y_values: list[float],
) -> tuple[float, float, float]:
    """Fit y = intercept + k_delta * x_delta + k_alpha * x_alpha."""
    if (
        len(x_delta) < 3
        or len(x_delta) != len(x_alpha)
        or len(x_delta) != len(y_values)
    ):
        return math.nan, math.nan, math.nan
    n = float(len(y_values))
    sx = sum(x_delta)
    sa = sum(x_alpha)
    sy = sum(y_values)
    sxx = sum(value * value for value in x_delta)
    saa = sum(value * value for value in x_alpha)
    sxa = sum(x * a for x, a in zip(x_delta, x_alpha))
    sxy = sum(x * y for x, y in zip(x_delta, y_values))
    say = sum(a * y for a, y in zip(x_alpha, y_values))
    intercept, k_delta, k_alpha = _solve_3x3(
        [
            [n, sx, sa],
            [sx, sxx, sxa],
            [sa, sxa, saa],
        ],
        [sy, sxy, say],
    )
    return intercept, k_delta, k_alpha


def _two_variable_no_intercept_fit(
    x_delta: list[float],
    x_alpha: list[float],
    y_values: list[float],
) -> tuple[float, float]:
    """Fit y = k_delta * x_delta + k_alpha * x_alpha."""
    if (
        len(x_delta) < 2
        or len(x_delta) != len(x_alpha)
        or len(x_delta) != len(y_values)
    ):
        return math.nan, math.nan
    sxx = sum(value * value for value in x_delta)
    saa = sum(value * value for value in x_alpha)
    sxa = sum(x * a for x, a in zip(x_delta, x_alpha))
    sxy = sum(x * y for x, y in zip(x_delta, y_values))
    say = sum(a * y for a, y in zip(x_alpha, y_values))
    determinant = sxx * saa - sxa * sxa
    if abs(determinant) <= 1.0e-12:
        return math.nan, math.nan
    k_delta = (sxy * saa - say * sxa) / determinant
    k_alpha = (sxx * say - sxa * sxy) / determinant
    return k_delta, k_alpha


def _json_safe(value: object) -> object:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _read_rows(root: Path) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for path in sorted(root.rglob('telemetry.csv')):
        with path.open(newline='', encoding='utf-8') as stream:
            for row in csv.DictReader(stream):
                row['_source_csv'] = str(path.resolve())
                rows.append(row)
    if not rows:
        raise ValueError(f'no telemetry.csv files under {root}')
    return rows


def _read_summary(path: Path) -> dict[str, object]:
    summary = path.parent / 'summary.json'
    if not summary.exists():
        return {}
    try:
        item = json.loads(summary.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        return {'_summary_read_error': True}
    item['_source_summary'] = str(summary.resolve())
    return item


def _bool(value: object, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        stripped = value.strip().lower()
        if stripped in {'1', 'true', 'yes', 'pass', 'valid'}:
            return True
        if stripped in {'0', 'false', 'no', 'fail', 'invalid'}:
            return False
    return default


def _summary_float(summary: dict[str, object], key: str) -> float:
    return _finite(summary.get(key))


def _summary_bool(summary: dict[str, object], key: str, default: bool) -> bool:
    return _bool(summary.get(key), default)


def _first_finite(rows: list[dict[str, str]], key: str) -> float:
    for row in rows:
        value = _finite(row.get(key))
        if math.isfinite(value):
            return value
    return math.nan


def _select_rows(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    selected: list[dict[str, str]] = []
    for row in rows:
        if not _measurement_row(row):
            continue
        if _finite(row.get('elevator_aero_wrench_gt_valid')) <= 0.5:
            continue
        required = (
            'qbar_selected_pa',
            'elevator_pitch_moment_gt_nm',
            'current_elevator_pitch_moment_nm',
            'elevator_dmoment_ddelta_per_q_m3_config',
            'alpha_est_rad',
        )
        if all(math.isfinite(_finite(row.get(key))) for key in required) and math.isfinite(
            _elevator_effective_angle(row)
        ):
            selected.append(row)
    return selected


def _run_quality(
    path: Path,
    *,
    minimum_gt_coverage: float,
    maximum_altitude_rmse_m: float,
    maximum_abs_altitude_error_m: float,
    maximum_mean_airspeed_error_mps: float = 0.30,
    maximum_std_airspeed_mps: float = 0.30,
    maximum_abs_airspeed_error_mps: float = 0.60,
) -> tuple[bool, dict[str, object], list[dict[str, str]], list[dict[str, str]]]:
    all_rows: list[dict[str, str]] = []
    with path.open(newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            row['_source_csv'] = str(path.resolve())
            all_rows.append(row)
    measurement_rows = [row for row in all_rows if _measurement_row(row)]
    selected_rows = _select_rows(all_rows)
    summary = _read_summary(path)
    gt_coverage = len(selected_rows) / len(measurement_rows) if measurement_rows else 0.0
    quality = str(summary.get('f1_attempt_quality', 'valid')).strip()
    quality_reason = str(summary.get('f1_attempt_quality_reason', 'none')).strip()
    # F3-B1 is local aerodynamic characterization.  A later failure to finish
    # the full MC->FW state sequence is diagnostic, not automatically a reason
    # to discard an already clean fixed measurement window.
    terminal_only_failure = (
        quality == 'protocol_invalid'
        and quality_reason == 'transition_did_not_reach_fw'
    )
    altitude_rmse = _summary_float(summary, 'va_hold_altitude_rmse_m')
    max_altitude_error = _summary_float(summary, 'va_hold_max_abs_altitude_error_m')
    mean_airspeed_error = abs(
        _summary_float(summary, 'va_hold_mean_airspeed_error_mps'))
    if not math.isfinite(mean_airspeed_error):
        mean_airspeed = _summary_float(summary, 'va_hold_mean_airspeed_mps')
        target_airspeed = _summary_float(summary, 'va_target_config')
        if math.isfinite(mean_airspeed) and math.isfinite(target_airspeed):
            mean_airspeed_error = abs(mean_airspeed - target_airspeed)
    std_airspeed = _summary_float(summary, 'va_hold_std_airspeed_mps')
    max_airspeed_error = _summary_float(
        summary, 'va_hold_max_abs_airspeed_error_mps')
    primary_failure = str(
        summary.get('primary_failure_cause', 'none')).strip().lower()
    va_hold_abort = str(
        summary.get('va_hold_abort_reason', 'none')).strip().lower()
    checks = {
        'measurement_complete': _bool(
            summary.get('va_hold_measurement_complete'), False),
        'measurement_protocol_valid': _bool(
            summary.get('va_hold_measurement_protocol_valid'), True),
        'target_condition_established': _bool(
            summary.get('f1_target_condition_established'), True),
        'lambda_hold_pass': _bool(
            summary.get('va_hold_lambda_band_occupancy_pass'), True),
        'gt_coverage': gt_coverage >= minimum_gt_coverage,
        'no_failsafe': not _bool(summary.get('any_failsafe'), False),
        'no_primary_failure': primary_failure in {'', 'none'},
        'no_va_hold_abort': va_hold_abort in {'', 'none'},
        'altitude_rmse_not_runaway': (
            math.isfinite(altitude_rmse)
            and altitude_rmse <= maximum_altitude_rmse_m
        ),
        'altitude_error_not_runaway': (
            math.isfinite(max_altitude_error)
            and max_altitude_error <= maximum_abs_altitude_error_m
        ),
        'mean_airspeed_error': (
            math.isfinite(mean_airspeed_error)
            and mean_airspeed_error <= maximum_mean_airspeed_error_mps
        ),
        'std_airspeed': (
            math.isfinite(std_airspeed)
            and std_airspeed <= maximum_std_airspeed_mps
        ),
        'max_abs_airspeed_error': (
            math.isfinite(max_airspeed_error)
            and max_airspeed_error <= maximum_abs_airspeed_error_mps
        ),
        'not_protocol_invalid_before_measurement': (
            quality in {'', 'valid'} or terminal_only_failure
        ),
    }
    result = {
        'source_csv': str(path.resolve()),
        'source_summary': summary.get('_source_summary', ''),
        'pass': all(checks.values()),
        'checks': checks,
        'quality': quality or 'valid',
        'quality_reason': quality_reason or 'none',
        'va_target_mps': _summary_float(summary, 'va_target_config'),
        'lambda_target': _summary_float(summary, 'lambda_target_config'),
        'measurement_sample_count': len(measurement_rows),
        'gt_sample_count': len(selected_rows),
        'gt_coverage_fraction': gt_coverage,
        'va_hold_altitude_rmse_m': altitude_rmse,
        'va_hold_max_abs_altitude_error_m': max_altitude_error,
        'va_hold_mean_airspeed_error_abs_mps': mean_airspeed_error,
        'va_hold_std_airspeed_mps': std_airspeed,
        'va_hold_max_abs_airspeed_error_mps': max_airspeed_error,
    }
    return bool(result['pass']), result, selected_rows, all_rows


def _metrics(rows: list[dict[str, str]]) -> dict[str, object]:
    x_values = [
        _finite(row['qbar_selected_pa'])
        * _elevator_effective_angle(row)
        for row in rows
    ]
    x_alpha = [
        _finite(row['qbar_selected_pa']) * _finite(row['alpha_est_rad'])
        for row in rows
    ]
    truth = [_finite(row['elevator_pitch_moment_gt_nm']) for row in rows]
    raw_estimate = [
        _finite(row['current_elevator_pitch_moment_nm']) for row in rows
    ]
    configured_slopes = [
        _finite(row['elevator_dmoment_ddelta_per_q_m3_config'])
        for row in rows
    ]
    configured_slope = _mean(configured_slopes)
    fit_slope, fit_intercept = _linear_fit(x_values, truth)
    univariate_prediction = [
        fit_slope * value + fit_intercept
        if math.isfinite(fit_slope) and math.isfinite(fit_intercept)
        else math.nan
        for value in x_values
    ]
    (
        nuisance_intercept,
        nuisance_delta_slope,
        nuisance_alpha_slope,
    ) = _two_variable_fit(x_values, x_alpha, truth)
    fit_prediction = [
        nuisance_intercept
        + nuisance_delta_slope * delta_value
        + nuisance_alpha_slope * alpha_value
        if all(math.isfinite(value) for value in (
            nuisance_intercept, nuisance_delta_slope, nuisance_alpha_slope,
        )) else math.nan
        for delta_value, alpha_value in zip(x_values, x_alpha)
    ]
    fixed_slope_intercept = (
        _mean(
            actual - configured_slope * value
            for value, actual in zip(x_values, truth)
        )
        if math.isfinite(configured_slope) else math.nan
    )
    fixed_slope_prediction = [
        configured_slope * value + fixed_slope_intercept
        if math.isfinite(configured_slope)
        and math.isfinite(fixed_slope_intercept) else math.nan
        for value in x_values
    ]
    slope_relative_error = (
        abs(nuisance_delta_slope - configured_slope) / abs(configured_slope)
        if math.isfinite(nuisance_delta_slope)
        and math.isfinite(configured_slope)
        and abs(configured_slope) > 1.0e-12 else math.nan
    )
    return {
        'sample_count': len(rows),
        'source_csv_count': len({
            row.get('_source_csv', '') for row in rows
        }),
        'qbar_delta_min_pa_rad': min(x_values) if x_values else math.nan,
        'qbar_delta_max_pa_rad': max(x_values) if x_values else math.nan,
        'qbar_delta_span_pa_rad': (
            max(x_values) - min(x_values) if x_values else math.nan
        ),
        'configured_derivative_per_q_m3': configured_slope,
        'fit_model': 'qbar_delta_plus_qbar_alpha_nuisance',
        'fit_derivative_per_q_m3': nuisance_delta_slope,
        'fit_alpha_derivative_per_q_m3': nuisance_alpha_slope,
        'fit_intercept_nm': nuisance_intercept,
        'fit_slope_relative_error': slope_relative_error,
        'fit_rmse_nm': _rmse(fit_prediction, truth),
        'fit_r_squared': _r_squared(fit_prediction, truth),
        'univariate_fit_derivative_per_q_m3': fit_slope,
        'univariate_fit_intercept_nm': fit_intercept,
        'univariate_fit_slope_relative_error': (
            abs(fit_slope - configured_slope) / abs(configured_slope)
            if math.isfinite(fit_slope)
            and math.isfinite(configured_slope)
            and abs(configured_slope) > 1.0e-12 else math.nan
        ),
        'univariate_fit_rmse_nm': _rmse(univariate_prediction, truth),
        'univariate_fit_r_squared': _r_squared(univariate_prediction, truth),
        'fixed_slope_intercept_nm': fixed_slope_intercept,
        'fixed_slope_rmse_nm': _rmse(fixed_slope_prediction, truth),
        'fixed_slope_r_squared': _r_squared(fixed_slope_prediction, truth),
        'raw_estimate_rmse_nm': _rmse(raw_estimate, truth),
        'raw_estimate_r_squared': _r_squared(raw_estimate, truth),
        'gt_moment_mean_nm': _mean(truth),
        'gt_moment_abs_mean_nm': _mean(abs(value) for value in truth),
        'online_current_moment_mean_nm': _mean(raw_estimate),
        'qbar_mean_pa': _mean(_finite(row['qbar_selected_pa']) for row in rows),
        'elevator_angle_mean_rad': _mean(
            _elevator_effective_angle(row) for row in rows
        ),
    }


def _centered_cell_metrics(rows: list[dict[str, str]]) -> dict[str, object]:
    """Fit a repeated-run cell with one intercept per source CSV.

    This is equivalent to within-run centering and then fitting
    y = K_delta * qbar_delta + K_alpha * qbar_alpha.  It keeps independent
    restarts from being forced to share the same aerodynamic trim moment.
    """
    records: list[tuple[str, float, float, float]] = []
    for row in rows:
        qbar = _finite(row['qbar_selected_pa'])
        records.append((
            row.get('_source_csv', ''),
            qbar * _elevator_effective_angle(row),
            qbar * _finite(row['alpha_est_rad']),
            _finite(row['elevator_pitch_moment_gt_nm']),
        ))
    by_source: dict[str, list[tuple[float, float, float]]] = {}
    for source, x_delta, x_alpha, truth in records:
        if all(math.isfinite(value) for value in (x_delta, x_alpha, truth)):
            by_source.setdefault(source, []).append((x_delta, x_alpha, truth))

    if len(by_source) <= 1:
        metrics = _metrics(rows)
        return {
            'fit_model': 'single_run_qbar_delta_plus_qbar_alpha_nuisance',
            'fit_derivative_per_q_m3': metrics['fit_derivative_per_q_m3'],
            'fit_alpha_derivative_per_q_m3': (
                metrics['fit_alpha_derivative_per_q_m3']
            ),
            'fit_intercept_nm': metrics['fit_intercept_nm'],
            'fit_run_intercepts_nm': {
                source: metrics['fit_intercept_nm'] for source in by_source
            },
            'fit_rmse_nm': metrics['fit_rmse_nm'],
            'fit_r_squared': metrics['fit_r_squared'],
            'centered_qbar_delta_min_pa_rad': (
                metrics['qbar_delta_min_pa_rad']
            ),
            'centered_qbar_delta_max_pa_rad': (
                metrics['qbar_delta_max_pa_rad']
            ),
            'centered_qbar_delta_span_pa_rad': (
                metrics['qbar_delta_span_pa_rad']
            ),
            'run_intercept_count': len(by_source),
        }

    centered_delta: list[float] = []
    centered_alpha: list[float] = []
    centered_truth: list[float] = []
    means: dict[str, tuple[float, float, float]] = {}
    for source, items in sorted(by_source.items()):
        if not items:
            continue
        mean_delta = fmean(item[0] for item in items)
        mean_alpha = fmean(item[1] for item in items)
        mean_truth = fmean(item[2] for item in items)
        means[source] = (mean_delta, mean_alpha, mean_truth)
        for x_delta, x_alpha, truth in items:
            centered_delta.append(x_delta - mean_delta)
            centered_alpha.append(x_alpha - mean_alpha)
            centered_truth.append(truth - mean_truth)

    k_delta, k_alpha = _two_variable_no_intercept_fit(
        centered_delta,
        centered_alpha,
        centered_truth,
    )
    centered_prediction = [
        k_delta * x_delta + k_alpha * x_alpha
        if math.isfinite(k_delta) and math.isfinite(k_alpha) else math.nan
        for x_delta, x_alpha in zip(centered_delta, centered_alpha)
    ]
    run_intercepts = {
        source: mean_truth - k_delta * mean_delta - k_alpha * mean_alpha
        if math.isfinite(k_delta) and math.isfinite(k_alpha) else math.nan
        for source, (mean_delta, mean_alpha, mean_truth) in means.items()
    }
    return {
        'fit_model': 'run_intercepts_plus_qbar_delta_plus_qbar_alpha_nuisance',
        'fit_derivative_per_q_m3': k_delta,
        'fit_alpha_derivative_per_q_m3': k_alpha,
        'fit_intercept_nm': math.nan,
        'fit_run_intercepts_nm': run_intercepts,
        'fit_rmse_nm': _rmse(centered_prediction, centered_truth),
        'fit_r_squared': _r_squared(centered_prediction, centered_truth),
        'centered_qbar_delta_min_pa_rad': (
            min(centered_delta) if centered_delta else math.nan
        ),
        'centered_qbar_delta_max_pa_rad': (
            max(centered_delta) if centered_delta else math.nan
        ),
        'centered_qbar_delta_span_pa_rad': (
            max(centered_delta) - min(centered_delta)
            if centered_delta else math.nan
        ),
        'run_intercept_count': len(run_intercepts),
    }


def _run_level_result(
    source: str,
    rows: list[dict[str, str]],
    *,
    minimum_samples_per_cell: int,
    minimum_qbar_delta_span_pa_rad: float,
    minimum_fit_r_squared: float,
    maximum_fit_rmse_nm: float,
    maximum_slope_relative_error: float,
) -> dict[str, object]:
    result = _metrics(rows)
    configured = _finite(result.get('configured_derivative_per_q_m3'))
    relative_error = (
        abs(_finite(result.get('fit_derivative_per_q_m3')) - configured)
        / abs(configured)
        if math.isfinite(_finite(result.get('fit_derivative_per_q_m3')))
        and math.isfinite(configured)
        and abs(configured) > 1.0e-12 else math.nan
    )
    checks = {
        'minimum_samples': (
            int(result['sample_count']) >= minimum_samples_per_cell
        ),
        'qbar_delta_excitation': (
            _finite(result['qbar_delta_span_pa_rad'])
            >= minimum_qbar_delta_span_pa_rad
        ),
        'fit_r_squared': (
            _finite(result['fit_r_squared']) >= minimum_fit_r_squared
        ),
        'fit_rmse': _finite(result['fit_rmse_nm']) <= maximum_fit_rmse_nm,
        'fit_slope_matches_config': (
            relative_error <= maximum_slope_relative_error
            if math.isfinite(relative_error) else False
        ),
    }
    return {
        'source_csv': source,
        'sample_count': result['sample_count'],
        'configured_derivative_per_q_m3': configured,
        'fit_derivative_per_q_m3': result['fit_derivative_per_q_m3'],
        'fit_alpha_derivative_per_q_m3': result['fit_alpha_derivative_per_q_m3'],
        'fit_intercept_nm': result['fit_intercept_nm'],
        'fit_slope_relative_error': relative_error,
        'fit_r_squared': result['fit_r_squared'],
        'fit_rmse_nm': result['fit_rmse_nm'],
        'qbar_delta_span_pa_rad': result['qbar_delta_span_pa_rad'],
        'checks': checks,
        'run_derivative_pass': all(checks.values()),
    }


def _cell_result(
    key: tuple[float, float],
    rows: list[dict[str, str]],
    *,
    minimum_samples_per_cell: int,
    minimum_qbar_delta_span_pa_rad: float,
    minimum_fit_r_squared: float,
    maximum_fit_rmse_nm: float,
    maximum_slope_relative_error: float,
    minimum_fixed_slope_r_squared: float,
) -> dict[str, object]:
    diagnostic = _metrics(rows)
    result = dict(diagnostic)
    formal = _centered_cell_metrics(rows)
    result.update(formal)
    configured = _finite(diagnostic.get('configured_derivative_per_q_m3'))
    result['configured_derivative_per_q_m3'] = configured
    result['pooled_single_intercept_fit_derivative_per_q_m3'] = (
        diagnostic['fit_derivative_per_q_m3']
    )
    result['pooled_single_intercept_fit_r_squared'] = diagnostic['fit_r_squared']
    result['pooled_single_intercept_fit_rmse_nm'] = diagnostic['fit_rmse_nm']
    result['pooled_single_intercept_fit_is_diagnostic_only'] = True
    result['fit_slope_relative_error'] = (
        abs(_finite(result['fit_derivative_per_q_m3']) - configured)
        / abs(configured)
        if math.isfinite(_finite(result['fit_derivative_per_q_m3']))
        and math.isfinite(configured)
        and abs(configured) > 1.0e-12 else math.nan
    )
    rows_by_source: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        rows_by_source.setdefault(row.get('_source_csv', ''), []).append(row)
    run_results = [
        _run_level_result(
            source,
            source_rows,
            minimum_samples_per_cell=minimum_samples_per_cell,
            minimum_qbar_delta_span_pa_rad=minimum_qbar_delta_span_pa_rad,
            minimum_fit_r_squared=minimum_fit_r_squared,
            maximum_fit_rmse_nm=maximum_fit_rmse_nm,
            maximum_slope_relative_error=maximum_slope_relative_error,
        )
        for source, source_rows in sorted(rows_by_source.items())
    ]
    run_slopes = [
        _finite(item['fit_derivative_per_q_m3'])
        for item in run_results
        if bool(item['run_derivative_pass'])
    ]
    checks = {
        'minimum_samples': (
            int(result['sample_count']) >= minimum_samples_per_cell
        ),
        'qbar_delta_excitation': (
            _finite(result['centered_qbar_delta_span_pa_rad'])
            >= minimum_qbar_delta_span_pa_rad
        ),
        'fit_r_squared': (
            _finite(result['fit_r_squared']) >= minimum_fit_r_squared
        ),
        'fit_rmse': _finite(result['fit_rmse_nm']) <= maximum_fit_rmse_nm,
        'fit_slope_matches_config': (
            _finite(result['fit_slope_relative_error'])
            <= maximum_slope_relative_error
        ),
        'all_run_derivative_fits_pass': bool(run_results) and all(
            bool(item['run_derivative_pass']) for item in run_results
        ),
    }
    result['fixed_slope_r_squared_diagnostic_pass'] = (
        _finite(result['fixed_slope_r_squared']) >= minimum_fixed_slope_r_squared
    )
    result.update({
        'va_target_mps': key[0],
        'lambda_target': key[1],
        'run_level_results': run_results,
        'run_derivative_pass_count': sum(
            1 for item in run_results if bool(item['run_derivative_pass'])
        ),
        'run_derivative_total_count': len(run_results),
        'run_level_derivative_mean_per_q_m3': _mean(run_slopes),
        'run_level_derivative_std_per_q_m3': _std(run_slopes),
        'checks': checks,
        'cell_validation_pass': all(checks.values()),
    })
    return result


def validate_elevator_moment(
    root: Path,
    *,
    minimum_samples: int = 50,
    minimum_samples_per_cell: int = 50,
    minimum_valid_cells: int = 1,
    required_airspeeds_mps: tuple[float, ...] = (),
    minimum_gt_coverage: float = 0.90,
    maximum_altitude_rmse_m: float = 3.0,
    maximum_abs_altitude_error_m: float = 5.0,
    minimum_qbar_delta_span_pa_rad: float = 0.5,
    minimum_fit_r_squared: float = 0.95,
    maximum_fit_rmse_nm: float = 0.05,
    maximum_slope_relative_error: float = 0.25,
    minimum_fixed_slope_r_squared: float = 0.90,
    include_quality_invalid: bool = False,
) -> dict[str, object]:
    _ = _read_rows(root)
    paths = sorted(root.rglob('telemetry.csv'))
    included_rows: list[dict[str, str]] = []
    excluded_runs: list[dict[str, object]] = []
    run_quality_results: list[dict[str, object]] = []
    cell_rows: dict[tuple[float, float], list[dict[str, str]]] = {}
    for path in paths:
        quality_pass, quality, selected_rows, all_rows = _run_quality(
            path,
            minimum_gt_coverage=minimum_gt_coverage,
            maximum_altitude_rmse_m=maximum_altitude_rmse_m,
            maximum_abs_altitude_error_m=maximum_abs_altitude_error_m,
        )
        run_quality_results.append(quality)
        if not quality_pass and not include_quality_invalid:
            excluded_runs.append(quality)
            continue
        included_rows.extend(selected_rows)
        va_target = _finite(quality.get('va_target_mps'))
        lambda_target = _finite(quality.get('lambda_target'))
        if not math.isfinite(va_target):
            va_target = _first_finite(all_rows, 'va_target_config')
        if not math.isfinite(lambda_target):
            lambda_target = _first_finite(all_rows, 'lambda_target_config')
        if math.isfinite(va_target) and math.isfinite(lambda_target):
            key = (round(va_target, 6), round(lambda_target, 6))
            cell_rows.setdefault(key, []).extend(selected_rows)
    metrics = _metrics(included_rows)
    cell_results = [
        _cell_result(
            key,
            rows,
            minimum_samples_per_cell=minimum_samples_per_cell,
            minimum_qbar_delta_span_pa_rad=minimum_qbar_delta_span_pa_rad,
            minimum_fit_r_squared=minimum_fit_r_squared,
            maximum_fit_rmse_nm=maximum_fit_rmse_nm,
            maximum_slope_relative_error=maximum_slope_relative_error,
            minimum_fixed_slope_r_squared=minimum_fixed_slope_r_squared,
        )
        for key, rows in sorted(cell_rows.items())
    ]
    run_derivative_results = [
        {
            **run,
            'va_target_mps': cell['va_target_mps'],
            'lambda_target': cell['lambda_target'],
        }
        for cell in cell_results
        for run in cell.get('run_level_results', [])
    ]
    local_slopes = [
        _finite(cell.get('fit_derivative_per_q_m3'))
        for cell in cell_results
        if bool(cell.get('cell_validation_pass'))
    ]
    configured = _finite(metrics.get('configured_derivative_per_q_m3'))
    valid_airspeeds = {
        round(_finite(cell['va_target_mps']), 6)
        for cell in cell_results
        if bool(cell.get('cell_validation_pass'))
    }
    derivative_pass_cell_count = sum(
        1 for cell in cell_results if bool(cell.get('cell_validation_pass'))
    )
    run_derivative_pass_count = sum(
        1 for run in run_derivative_results
        if bool(run.get('run_derivative_pass'))
    )
    required = {round(value, 6) for value in required_airspeeds_mps}
    missing_required = sorted(required - valid_airspeeds)
    checks = {
        'minimum_samples': (
            int(metrics['sample_count']) >= minimum_samples
        ),
        'all_run_derivative_fits_pass': bool(run_derivative_results) and all(
            bool(run['run_derivative_pass'])
            for run in run_derivative_results
        ),
        'minimum_derivative_pass_cells': (
            derivative_pass_cell_count >= minimum_valid_cells
        ),
        'required_airspeeds_present': not missing_required,
        'all_quality_valid_cells_derivative_pass': bool(cell_results) and all(
            bool(cell['cell_validation_pass']) for cell in cell_results
        ),
    }
    return {
        'experiment': 'F3B1_elevator_moment_validation',
        'root': str(root.resolve()),
        'thresholds': {
            'minimum_samples': minimum_samples,
            'minimum_samples_per_cell': minimum_samples_per_cell,
            'minimum_valid_cells': minimum_valid_cells,
            'required_airspeeds_mps': list(required_airspeeds_mps),
            'minimum_gt_coverage': minimum_gt_coverage,
            'maximum_altitude_rmse_m': maximum_altitude_rmse_m,
            'maximum_abs_altitude_error_m': maximum_abs_altitude_error_m,
            'minimum_qbar_delta_span_pa_rad': minimum_qbar_delta_span_pa_rad,
            'minimum_fit_r_squared': minimum_fit_r_squared,
            'maximum_fit_rmse_nm': maximum_fit_rmse_nm,
            'maximum_slope_relative_error': maximum_slope_relative_error,
            'minimum_fixed_slope_r_squared': minimum_fixed_slope_r_squared,
            'maximum_mean_airspeed_error_mps': 0.30,
            'maximum_std_airspeed_mps': 0.30,
            'maximum_abs_airspeed_error_mps': 0.60,
        },
        **metrics,
        'analysis_revision': (
            'F3-B-specific operating-point quality gate; run-specific '
            'intercepts for repeated local identification'
        ),
        'valid_run_count': sum(
            1 for item in run_quality_results if bool(item.get('pass'))
        ),
        'excluded_run_count': len(excluded_runs),
        'excluded_runs': excluded_runs,
        'run_derivative_results': run_derivative_results,
        'run_quality_valid_count': sum(
            1 for item in run_quality_results if bool(item.get('pass'))
        ),
        'run_derivative_pass_count': run_derivative_pass_count,
        'run_derivative_total_count': len(run_derivative_results),
        'cell_results': cell_results,
        'quality_valid_cell_count': len(cell_results),
        'derivative_pass_cell_count': derivative_pass_cell_count,
        'required_airspeed_cells': len(required),
        'valid_cell_count': len(cell_results),
        'missing_required_airspeeds_mps': missing_required,
        'local_derivative_mean_per_q_m3': _mean(local_slopes),
        'local_derivative_min_per_q_m3': min(local_slopes) if local_slopes else math.nan,
        'local_derivative_max_per_q_m3': max(local_slopes) if local_slopes else math.nan,
        'local_derivative_max_relative_error': (
            max(
                abs(slope - configured) / abs(configured)
                for slope in local_slopes
            )
            if local_slopes and math.isfinite(configured)
            and abs(configured) > 1.0e-12 else math.nan
        ),
        'pooled_global_intercept_fit_is_diagnostic_only': True,
        'checks': checks,
        'incremental_derivative_validation_pass': all(checks.values()),
        'f3b1_elevator_moment_validation_pass': all(checks.values()),
        'interpretation': (
            'absolute moment contains operating-point-dependent aerodynamic '
            'bias; formal repeated-run cells use run-specific intercepts '
            'with a qbar*alpha nuisance regressor for eta_C incremental '
            'authority validation; pooled single-intercept fits are '
            'diagnostic only'
        ),
    }


def write_validation(payload: dict[str, object], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / 'f3b1_elevator_moment_validation.json').write_text(
        json.dumps(_json_safe(payload), indent=2, allow_nan=False) + '\n',
        encoding='utf-8',
    )
    cell_fields = (
        'va_target_mps', 'lambda_target', 'cell_validation_pass',
        'sample_count', 'source_csv_count',
        'configured_derivative_per_q_m3', 'fit_derivative_per_q_m3',
        'run_level_derivative_mean_per_q_m3',
        'run_level_derivative_std_per_q_m3',
        'fit_slope_relative_error', 'fit_r_squared', 'fit_rmse_nm',
        'fit_model', 'run_intercept_count',
        'centered_qbar_delta_span_pa_rad',
        'pooled_single_intercept_fit_r_squared',
        'fixed_slope_r_squared', 'fixed_slope_rmse_nm',
        'fit_intercept_nm', 'qbar_delta_span_pa_rad',
    )
    with (output_dir / 'f3b1_elevator_moment_cells.csv').open(
        'w', newline='', encoding='utf-8',
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=cell_fields)
        writer.writeheader()
        for cell in payload.get('cell_results', []):
            writer.writerow({key: _json_safe(cell.get(key)) for key in cell_fields})
    run_fields = (
        'va_target_mps', 'lambda_target', 'source_csv',
        'run_derivative_pass', 'sample_count',
        'configured_derivative_per_q_m3', 'fit_derivative_per_q_m3',
        'fit_alpha_derivative_per_q_m3', 'fit_slope_relative_error',
        'fit_r_squared', 'fit_rmse_nm', 'qbar_delta_span_pa_rad',
    )
    with (output_dir / 'f3b1_elevator_moment_runs.csv').open(
        'w', newline='', encoding='utf-8',
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=run_fields)
        writer.writeheader()
        for run in payload.get('run_derivative_results', []):
            writer.writerow({key: _json_safe(run.get(key)) for key in run_fields})
    exclusion_fields = (
        'source_csv', 'pass', 'quality', 'quality_reason',
        'va_target_mps', 'lambda_target', 'measurement_sample_count',
        'gt_sample_count', 'gt_coverage_fraction',
        'va_hold_altitude_rmse_m', 'va_hold_max_abs_altitude_error_m',
    )
    with (output_dir / 'f3b1_elevator_moment_excluded_runs.csv').open(
        'w', newline='', encoding='utf-8',
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=exclusion_fields)
        writer.writeheader()
        for item in payload.get('excluded_runs', []):
            writer.writerow({
                key: _json_safe(item.get(key)) for key in exclusion_fields
            })


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--required-va', type=float, nargs='*', default=())
    parser.add_argument('--minimum-valid-cells', type=int, default=1)
    parser.add_argument(
        '--include-quality-invalid',
        action='store_true',
        help='Diagnostic mode only: include runs that fail the F3-B1 local '
             'measurement-quality gate.',
    )
    args = parser.parse_args()
    payload = validate_elevator_moment(
        args.root,
        minimum_valid_cells=args.minimum_valid_cells,
        required_airspeeds_mps=tuple(args.required_va),
        include_quality_invalid=args.include_quality_invalid,
    )
    write_validation(payload, args.output_dir)
    print(json.dumps(_json_safe(payload), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
