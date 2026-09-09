import csv
import json
import tempfile
import unittest
from pathlib import Path

from ca_lsc_td3.evaluation.f3_capabilities import (
    _effective_phase,
    _scientific_force_phase,
    eta_l_outcome_space,
    validate_eta_l,
)


class F3CapabilityEvaluationTests(unittest.TestCase):
    def test_eta_l_validation_uses_only_fresh_ground_truth(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = root / 'run_1' / 'telemetry.csv'
            path.parent.mkdir()
            fields = (
                'eta_l_valid', 'selected_airspeed_mps',
                'wing_lift_gt_valid', 'wing_vertical_support_est_n',
                'wing_lift_gt_vertical_enu_n', 'eta_l', 'eta_l_gt',
                'va_hold_phase',
            )
            with path.open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                for index in range(10):
                    truth = 10.0 + index
                    writer.writerow({
                        'eta_l_valid': 1,
                        'selected_airspeed_mps': 8.0 + index / 10.0,
                        'wing_lift_gt_valid': int(index != 0),
                        'wing_vertical_support_est_n': truth,
                        'wing_lift_gt_vertical_enu_n': truth,
                        'eta_l': truth / 50.0,
                        'eta_l_gt': truth / 50.0,
                        'va_hold_phase': 'measurement',
                    })
            payload, rows = validate_eta_l(
                root,
                minimum_samples=9,
                minimum_gt_coverage=0.9,
                minimum_condition_samples=9,
            )
            self.assertEqual(len(rows), 9)
            self.assertEqual(payload['eta_l_valid_sample_count'], 10)
            self.assertAlmostEqual(
                payload['ground_truth_coverage_fraction'], 0.9
            )
            self.assertAlmostEqual(
                payload['combined_metrics']['force_rmse_n'], 0.0
            )
            self.assertTrue(payload['eta_l_physical_validation_pass'])
            self.assertEqual(
                payload['combined_metrics']['phase_sample_counts'],
                {'measurement': 9},
            )
            self.assertTrue(
                payload['conditions'][0]['condition_validation_pass']
            )
            self.assertTrue(
                payload['operating_points'][0][
                    'operating_point_validation_pass'
                ]
            )

    def test_force_model_mask_is_separate_from_online_availability(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = root / 'run_1' / 'telemetry.csv'
            path.parent.mkdir()
            fields = (
                'eta_l_available', 'eta_l_valid',
                'eta_l_force_model_valid', 'selected_airspeed_mps',
                'wing_lift_gt_valid', 'wing_vertical_support_est_n',
                'wing_lift_gt_vertical_enu_n', 'eta_l', 'eta_l_gt',
                'va_hold_phase',
            )
            with path.open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                for index in range(6):
                    writer.writerow({
                        'eta_l_available': 1,
                        'eta_l_valid': 1,
                        'eta_l_force_model_valid': int(index >= 2),
                        'selected_airspeed_mps': 8.0,
                        'wing_lift_gt_valid': 1,
                        'wing_vertical_support_est_n': 20.0 + index,
                        'wing_lift_gt_vertical_enu_n': 20.0 + index,
                        'eta_l': 0.4,
                        'eta_l_gt': 0.4,
                        'va_hold_phase': 'measurement',
                    })
            payload, rows = validate_eta_l(
                root,
                minimum_samples=4,
                minimum_gt_coverage=1.0,
                minimum_condition_samples=4,
            )
            self.assertEqual(len(rows), 4)
            self.assertEqual(payload['eta_l_available_sample_count'], 6)
            self.assertEqual(
                payload['eta_l_force_model_valid_sample_count'], 4
            )
            self.assertEqual(
                payload['runs'][0]['eta_l_available_sample_count'], 6
            )

    def test_schema18_reports_available_and_force_valid_flow_means(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = root / 'run_1' / 'telemetry.csv'
            path.parent.mkdir()
            fields = (
                'eta_l_available', 'eta_l_valid',
                'eta_l_force_model_valid', 'selected_airspeed_mps',
                'wing_lift_gt_valid', 'wing_vertical_support_est_n',
                'wing_lift_gt_vertical_enu_n', 'eta_l', 'eta_l_gt',
                'va_hold_phase', 'wind_heading_parallel_mps',
                'wind_heading_cross_mps', 'vrel_body_u_mps',
                'vrel_body_v_mps', 'vrel_body_w_mps', 'beta_est_rad',
                'yaw_rad',
            )
            rows = (
                # Online eta_L can be computed, but this sample is outside
                # the strict longitudinal force-model gate.
                (1, 0, 1.0, 0.4),
                (1, 1, 10.0, -0.1),
                (1, 1, 20.0, 0.2),
                # Not available at all; it should not affect either new
                # suffixed diagnostic subset.
                (0, 0, 100.0, 0.9),
            )
            with path.open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                for available, force_valid, body_u, beta in rows:
                    writer.writerow({
                        'eta_l_available': available,
                        'eta_l_valid': available,
                        'eta_l_force_model_valid': force_valid,
                        'selected_airspeed_mps': max(8.0, body_u),
                        'wing_lift_gt_valid': force_valid,
                        'wing_vertical_support_est_n': 25.0,
                        'wing_lift_gt_vertical_enu_n': 25.0,
                        'eta_l': 0.5,
                        'eta_l_gt': 0.5,
                        'va_hold_phase': 'measurement',
                        'wind_heading_parallel_mps': 4.0,
                        'wind_heading_cross_mps': 0.5,
                        'vrel_body_u_mps': body_u,
                        'vrel_body_v_mps': 0.2,
                        'vrel_body_w_mps': -0.3,
                        'beta_est_rad': beta,
                        'yaw_rad': 0.0,
                    })
            payload, rows = validate_eta_l(
                root,
                minimum_samples=2,
                minimum_gt_coverage=1.0,
                minimum_condition_samples=2,
            )
            self.assertEqual(len(rows), 2)
            run = payload['runs'][0]
            self.assertAlmostEqual(
                run['mean_vrel_body_u_available_mps'],
                (1.0 + 10.0 + 20.0) / 3.0,
            )
            self.assertAlmostEqual(
                run['mean_vrel_body_u_force_valid_mps'], 15.0,
            )
            self.assertAlmostEqual(
                run['mean_abs_beta_available_rad'],
                (0.4 + 0.1 + 0.2) / 3.0,
            )
            self.assertAlmostEqual(
                run['mean_abs_beta_force_valid_rad'], 0.15,
            )

    def test_takeoff_inactive_samples_are_excluded_from_force_regression(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = root / 'run_1' / 'telemetry.csv'
            path.parent.mkdir()
            fields = (
                'eta_l_available', 'eta_l_force_model_valid',
                'selected_airspeed_mps', 'wing_lift_gt_valid',
                'wing_vertical_support_est_n',
                'wing_lift_gt_vertical_enu_n', 'eta_l', 'eta_l_gt',
                'va_hold_phase',
            )
            with path.open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                for phase, estimate, truth in (
                    ('inactive', -100.0, -20.0),
                    ('airspeed_settle', 25.0, 25.0),
                    ('lambda_settle', 30.0, 30.0),
                    ('measurement', 35.0, 35.0),
                ):
                    writer.writerow({
                        'eta_l_available': 1,
                        'eta_l_force_model_valid': 1,
                        'selected_airspeed_mps': 10.0,
                        'wing_lift_gt_valid': 1,
                        'wing_vertical_support_est_n': estimate,
                        'wing_lift_gt_vertical_enu_n': truth,
                        'eta_l': estimate / 50.0,
                        'eta_l_gt': truth / 50.0,
                        'va_hold_phase': phase,
                    })
            payload, rows = validate_eta_l(
                root,
                minimum_samples=3,
                minimum_gt_coverage=1.0,
                minimum_condition_samples=3,
            )
            self.assertEqual(len(rows), 3)
            self.assertEqual(payload['eta_l_force_model_valid_sample_count'], 3)
            self.assertEqual(
                payload['eta_l_force_model_valid_raw_sample_count'], 4
            )
            self.assertEqual(
                payload['eta_l_scientific_phase_excluded_sample_count'], 1
            )
            self.assertAlmostEqual(
                payload['combined_metrics']['force_rmse_n'], 0.0
            )

    def test_protocol_invalid_run_is_not_used_for_physical_validation(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fields = (
                'eta_l_valid', 'selected_airspeed_mps',
                'wing_lift_gt_valid', 'wing_vertical_support_est_n',
                'wing_lift_gt_vertical_enu_n', 'eta_l', 'eta_l_gt',
                'va_hold_phase',
            )
            for run_index, quality in ((1, 'valid'), (2, 'protocol_invalid')):
                run_dir = root / f'run_{run_index}'
                run_dir.mkdir()
                (run_dir / 'summary.json').write_text(json.dumps({
                    'f1_attempt_quality': quality,
                    'f1_attempt_quality_reason': (
                        'none' if quality == 'valid'
                        else 'target_condition_not_established'
                    ),
                }) + '\n', encoding='utf-8')
                with (run_dir / 'telemetry.csv').open(
                    'w', newline='', encoding='utf-8'
                ) as stream:
                    writer = csv.DictWriter(stream, fieldnames=fields)
                    writer.writeheader()
                    for index in range(5):
                        truth = 10.0 + index
                        estimate = truth if quality == 'valid' else truth + 100.0
                        writer.writerow({
                            'eta_l_valid': 1,
                            'selected_airspeed_mps': 8.0,
                            'wing_lift_gt_valid': 1,
                            'wing_vertical_support_est_n': estimate,
                            'wing_lift_gt_vertical_enu_n': truth,
                            'eta_l': estimate / 50.0,
                            'eta_l_gt': truth / 50.0,
                            'va_hold_phase': 'measurement',
                        })
            payload, rows = validate_eta_l(
                root,
                minimum_samples=5,
                minimum_gt_coverage=1.0,
                minimum_condition_samples=5,
            )
            self.assertEqual(len(rows), 5)
            self.assertEqual(payload['telemetry_file_count'], 2)
            self.assertEqual(payload['protocol_valid_telemetry_file_count'], 1)
            self.assertEqual(payload['protocol_invalid_telemetry_file_count'], 1)
            self.assertAlmostEqual(
                payload['combined_metrics']['force_rmse_n'], 0.0
            )
            invalid = [
                run for run in payload['runs'] if not run['protocol_valid']
            ][0]
            self.assertFalse(invalid['used_in_validation'])
            self.assertEqual(
                invalid['protocol_invalid_reason'],
                'target_condition_not_established',
            )

    def test_effective_phase_does_not_let_inactive_gust_mask_va_hold(self):
        self.assertEqual(_effective_phase({
            'schedule_mode': 'va_hold_target',
            'gust_phase': 'inactive',
            'va_hold_phase': 'lambda_settle',
            'va_hold_measurement_active': '0',
            'va_hold_measurement_complete': '0',
        }), 'lambda_settle')
        self.assertEqual(_effective_phase({
            'schedule_mode': 'va_hold_target',
            'gust_phase': 'inactive',
            'va_hold_phase': 'measurement',
            'va_hold_measurement_active': '0',
            'va_hold_measurement_complete': '1',
        }), 'post_measurement_release')
        self.assertEqual(_effective_phase({
            'schedule_mode': 'gust_target',
            'gust_phase': 'gust_recovery',
            'va_hold_phase': 'gust_recovery',
        }), 'gust_recovery')
        self.assertFalse(_scientific_force_phase({
            'schedule_mode': 'va_hold_target',
            'va_hold_phase': 'inactive',
        }))
        self.assertTrue(_scientific_force_phase({
            'schedule_mode': 'va_hold_target',
            'va_hold_phase': 'airspeed_settle',
        }))

    def test_outcome_space_accepts_valid_retries_and_maps_classes(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'points.csv'
            fields = (
                'condition_id', 'mass_scale', 'va_target_mps',
                'lambda_target', 'eta_l_proxy_mean', 'physical_class',
                'data_quality',
            )
            with path.open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerow({
                    'condition_id': 'm1', 'mass_scale': 1.0,
                    'va_target_mps': 8.0, 'lambda_target': 0.5,
                    'eta_l_proxy_mean': 0.4,
                    'physical_class': 'nominal_feasible',
                    'data_quality': 'valid_with_retries',
                })
                writer.writerow({
                    'condition_id': 'bad', 'mass_scale': 1.2,
                    'va_target_mps': 8.0, 'lambda_target': 0.7,
                    'eta_l_proxy_mean': 0.3,
                    'physical_class': 'physical_unsafe',
                    'data_quality': 'protocol_invalid',
                })
            points = eta_l_outcome_space([path])
            self.assertEqual(len(points), 1)
            self.assertEqual(points[0]['outcome_class'], 'N')


if __name__ == '__main__':
    unittest.main()
