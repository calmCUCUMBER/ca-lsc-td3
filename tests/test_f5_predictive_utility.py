import csv
import tempfile
import unittest
from pathlib import Path

import numpy as np

from ca_lsc_td3.evaluation.f5_predictive_utility import (
    evaluate_predictive_utility,
    make_group_folds,
    pr_auc,
    roc_auc,
    select_unsafe_alarm_threshold,
)


class F5PredictiveUtilityTests(unittest.TestCase):
    def test_group_folds_never_split_a_condition(self):
        labels = np.asarray([0, 0, 1, 1, 0, 1, 0, 1], dtype=int)
        groups = np.asarray(['a', 'a', 'b', 'b', 'c', 'c', 'd', 'e'])
        folds = make_group_folds(labels, groups, n_splits=3)
        ownership = {}
        for fold, indices in enumerate(folds):
            for group in set(groups[indices]):
                self.assertNotIn(group, ownership)
                ownership[group] = fold
        self.assertEqual(set(ownership), set(groups))

    def test_minority_groups_are_distributed_across_folds(self):
        labels = []
        groups = []
        for group_index in range(10):
            group = f'g{group_index}'
            group_labels = [1] * 20
            if group_index < 5:
                group_labels[-1] = 0
            labels.extend(group_labels)
            groups.extend([group] * len(group_labels))
        labels_array = np.asarray(labels, dtype=int)
        groups_array = np.asarray(groups)
        folds = make_group_folds(labels_array, groups_array, n_splits=5)
        self.assertTrue(all(np.any(labels_array[fold] == 0) for fold in folds))

    def test_metrics_have_known_values(self):
        labels = np.asarray([0, 0, 1, 1], dtype=int)
        perfect = np.asarray([0.1, 0.2, 0.8, 0.9])
        self.assertAlmostEqual(roc_auc(labels, perfect), 1.0)
        self.assertAlmostEqual(pr_auc(labels, perfect), 1.0)

    def test_capability_model_improves_on_grouped_synthetic_data(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'dataset.csv'
            fields = (
                'condition_group', 'label_safe_2s', 'va_mps',
                'eta_l', 'eta_c', 'lambda_exec',
            )
            rng = np.random.default_rng(7)
            with path.open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                for group_index in range(20):
                    capability = 0.15 if group_index % 2 == 0 else 0.85
                    for _ in range(20):
                        eta_l = float(np.clip(capability + rng.normal(0, 0.03), 0, 1))
                        eta_c = float(np.clip(capability + rng.normal(0, 0.03), 0, 1))
                        writer.writerow({
                            'condition_group': f'condition_{group_index}',
                            'label_safe_2s': int(capability > 0.5),
                            'va_mps': 10.0 + rng.normal(0, 0.05),
                            'eta_l': eta_l,
                            'eta_c': eta_c,
                            'lambda_exec': 0.5 + rng.normal(0, 0.01),
                        })
            payload = evaluate_predictive_utility(
                path, n_splits=5, bootstrap_repetitions=20, seed=3,
            )
            self.assertGreater(
                payload['model_b']['roc_auc'], payload['model_a']['roc_auc'])
            self.assertLess(payload['model_b']['brier'], payload['model_a']['brier'])
            self.assertEqual(payload['condition_group_count'], 20)
            self.assertIn('condition_macro', payload)
            self.assertIn('unsafe_pr_auc', payload['model_b'])
            self.assertIn(
                'unsafe_recall_at_5pct_safe_false_alarm',
                payload['model_b'],
            )
            self.assertTrue(all(
                fold['test_condition_groups']
                for fold in payload['folds']
            ))

    def test_unsafe_alarm_threshold_respects_safe_false_alarm_budget(self):
        labels = np.asarray([1] * 20 + [0] * 5, dtype=int)
        probabilities = np.asarray(
            [0.95] * 19 + [0.10] + [0.20, 0.25, 0.30, 0.85, 0.90],
            dtype=float,
        )
        threshold = select_unsafe_alarm_threshold(
            labels, probabilities, max_safe_false_alarm_rate=0.05,
        )
        unsafe_alarm = probabilities < threshold
        self.assertLessEqual(float(np.mean(unsafe_alarm[labels == 1])), 0.05)
        self.assertGreaterEqual(float(np.mean(unsafe_alarm[labels == 0])), 0.6)

    def test_enhanced_fold_csv_fields_are_written(self):
        with tempfile.TemporaryDirectory() as temporary:
            dataset = Path(temporary) / 'dataset.csv'
            output = Path(temporary) / 'out'
            fields = (
                'condition_group', 'label_safe_2s', 'va_mps',
                'eta_l', 'eta_c', 'lambda_exec',
            )
            with dataset.open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                for group_index in range(8):
                    safe = int(group_index % 2 == 0)
                    for sample in range(8):
                        writer.writerow({
                            'condition_group': f'g{group_index}',
                            'label_safe_2s': safe if sample < 6 else 1 - safe,
                            'va_mps': 8.0 + group_index,
                            'eta_l': 0.2 + 0.1 * group_index,
                            'eta_c': 0.8 - 0.05 * group_index,
                            'lambda_exec': 0.4 + 0.01 * sample,
                        })
            from ca_lsc_td3.evaluation.f5_predictive_utility import (
                write_predictive_outputs,
            )
            payload = evaluate_predictive_utility(
                dataset, n_splits=4, bootstrap_repetitions=5, seed=11,
            )
            write_predictive_outputs(payload, output)
            with (output / 'f5_predictive_utility_folds.csv').open(
                newline='', encoding='utf-8',
            ) as stream:
                row = next(csv.DictReader(stream))
            self.assertIn('test_condition_groups', row)
            self.assertIn('unsafe_pr_auc', row)
            self.assertIn('balanced_accuracy', row)
            self.assertIn('macro_f1', row)
            self.assertIn('mcc', row)


if __name__ == '__main__':
    unittest.main()
