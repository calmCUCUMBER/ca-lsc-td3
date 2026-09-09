import tempfile
import unittest
from pathlib import Path

import numpy as np

from ca_lsc_td3.evaluation.f1_heatmaps import (
    _matrix,
    _outcome_class,
    render_all,
)


class F1HeatmapTests(unittest.TestCase):
    def test_formal_matrix_masks_quality_invalid_and_never_labels_t(self):
        points = [
            {
                'va_target_mps': 10.0,
                'lambda_target': 0.1,
                'physical_class': 'boundary_feasible',
                'data_quality': 'valid',
                'metric': 1.0,
            },
            {
                'va_target_mps': 10.0,
                'lambda_target': 0.3,
                'physical_class': 'unknown',
                'data_quality': 'protocol_invalid',
                'metric': 99.0,
            },
            {
                'va_target_mps': 12.0,
                'lambda_target': 0.1,
                'physical_class': 'physical_unsafe',
                'data_quality': 'valid',
                'metric': 2.0,
            },
        ]
        vas, lambdas, values, labels = _matrix(
            points, 'metric', valid_only=True
        )
        row10, row12 = vas.index(10.0), vas.index(12.0)
        col01, col03 = lambdas.index(0.1), lambdas.index(0.3)
        self.assertTrue(np.isnan(values[row10, col03]))
        self.assertEqual(labels[(row10, col01)], 'B')
        self.assertEqual(labels[(row12, col01)], 'X')
        self.assertNotIn('T', labels.values())

    def test_render_all_smoke_writes_separate_quality_and_boundary_figures(self):
        points = []
        for va in (10.0, 12.0):
            for lam in (0.0, 0.9):
                points.append({
                    'va_target_mps': va,
                    'lambda_target': lam,
                    'physical_class': 'nominal_feasible',
                    'data_quality': 'valid',
                    'va_hold_mean_total_power_w_mean': 100.0,
                    'va_hold_max_abs_altitude_error_m_p95': 0.5,
                    'va_hold_max_abs_altitude_error_m_max': 0.7,
                    'pusher_propulsive_efficiency_proxy_median': 1.2,
                })
        payload = {
            'points': points,
            'derived_by_airspeed': [
                {
                    'va_target_mps': va,
                    'lambda_max_feasible': 0.9,
                    'lambda_max_feasible_is_lower_bound': True,
                    'lambda_first_unsafe_above': None,
                } for va in (10.0, 12.0)
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            render_all(payload, output)
            expected = (
                'f1_va_lambda_total_power.png',
                'f1_va_lambda_max_altitude_error.png',
                'f1_va_lambda_max_altitude_error_diagnostic.png',
                'f1_pusher_power_sanity.png',
                'f1_data_quality.png',
                'f1_outcome_classes.png',
                'f1_feasible_boundary.png',
            )
            for name in expected:
                self.assertGreater((output / name).stat().st_size, 0)

    def test_valid_retry_is_physical_but_protocol_unstable_is_not(self):
        retry = {
            'physical_class': 'physical_unsafe',
            'data_quality': 'valid_with_retries',
        }
        unstable = {
            'physical_class': 'unknown',
            'data_quality': 'protocol_unstable',
            'outcome_class': 'protocol_invalid',
        }
        self.assertEqual(_outcome_class(retry), 'physical_unsafe')
        self.assertEqual(_outcome_class(unstable), 'missing')


if __name__ == '__main__':
    unittest.main()
