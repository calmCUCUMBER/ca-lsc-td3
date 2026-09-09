import csv
import json
import tempfile
import unittest
from pathlib import Path

from ca_lsc_td3.evaluation.f3b_moment_validation import (
    validate_elevator_moment,
)


class F3BElevatorMomentValidationTests(unittest.TestCase):
    def _write_rows(
        self,
        root: Path,
        *,
        slope: float = -0.06,
        intercept: float = -0.34,
        valid: bool = True,
    ) -> None:
        run = root / 'run_1'
        run.mkdir(parents=True)
        (run / 'summary.json').write_text(json.dumps({
            'telemetry_schema_version': 20,
            'f1_attempt_quality': 'valid',
            'f1_attempt_quality_reason': 'none',
            'f1_target_condition_established': True,
            'va_target_config': 12.0,
            'lambda_target_config': 0.6,
            'va_hold_measurement_complete': True,
            'va_hold_measurement_protocol_valid': True,
            'va_hold_fixed_window_quality_pass': True,
            'va_hold_airspeed_band_occupancy_pass': True,
            'va_hold_lambda_band_occupancy_pass': True,
            'va_hold_mean_airspeed_error_mps': 0.02,
            'va_hold_std_airspeed_mps': 0.02,
            'va_hold_max_abs_airspeed_error_mps': 0.05,
            'va_hold_altitude_rmse_m': 0.5,
            'va_hold_max_abs_altitude_error_m': 0.8,
            'primary_failure_cause': 'none',
            'va_hold_abort_reason': 'none',
            'any_failsafe': False,
        }) + '\n', encoding='utf-8')
        fields = (
            'va_hold_measurement_active',
            'va_hold_measurement_complete',
            'elevator_aero_wrench_gt_valid',
            'qbar_selected_pa',
            'elevator_actual_angle_rad',
            'elevator_pitch_moment_gt_nm',
            'current_elevator_pitch_moment_nm',
            'elevator_dmoment_ddelta_per_q_m3_config',
            'alpha_est_rad',
        )
        with (run / 'telemetry.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for index in range(80):
                qbar = 70.0 + index * 0.5
                elevator = 0.04 + index * 0.001
                x_value = qbar * elevator
                writer.writerow({
                    'va_hold_measurement_active': 1,
                    'va_hold_measurement_complete': 0,
                    'elevator_aero_wrench_gt_valid': 1 if valid else 0,
                    'qbar_selected_pa': qbar,
                    'elevator_actual_angle_rad': elevator,
                    'elevator_pitch_moment_gt_nm': slope * x_value + intercept,
                    'current_elevator_pitch_moment_nm': slope * x_value,
                    'elevator_dmoment_ddelta_per_q_m3_config': -0.06,
                    'alpha_est_rad': 0.05 + index * 0.0001,
                })

    def test_bias_separated_slope_validation_passes(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self._write_rows(root)
            payload = validate_elevator_moment(root)
            self.assertTrue(payload['f3b1_elevator_moment_validation_pass'])
            self.assertAlmostEqual(
                payload['fit_derivative_per_q_m3'], -0.06, places=9,
            )
            self.assertLess(payload['fit_rmse_nm'], 1.0e-9)
            self.assertLess(payload['raw_estimate_r_squared'], 0.0)
            self.assertGreater(payload['fixed_slope_r_squared'], 0.99)
            self.assertEqual(payload['valid_cell_count'], 1)
            self.assertEqual(payload['excluded_run_count'], 0)

    def test_rejects_missing_elevator_wrench_ground_truth(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self._write_rows(root, valid=False)
            payload = validate_elevator_moment(root)
            self.assertFalse(payload['f3b1_elevator_moment_validation_pass'])
            self.assertFalse(payload['checks']['minimum_samples'])

    def test_rejects_wrong_configured_slope(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self._write_rows(root, slope=-0.03)
            payload = validate_elevator_moment(root)
            self.assertFalse(payload['f3b1_elevator_moment_validation_pass'])
            self.assertFalse(
                payload['checks']['all_quality_valid_cells_derivative_pass']
            )
            self.assertFalse(
                payload['cell_results'][0]['checks']['fit_slope_matches_config']
            )

    def test_excludes_dirty_measurement_window(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self._write_rows(root)
            summary = root / 'run_1' / 'summary.json'
            item = json.loads(summary.read_text(encoding='utf-8'))
            item['va_hold_fixed_window_quality_pass'] = False
            item['va_hold_altitude_rmse_m'] = 4.5
            summary.write_text(json.dumps(item) + '\n', encoding='utf-8')
            payload = validate_elevator_moment(root)
            self.assertFalse(payload['f3b1_elevator_moment_validation_pass'])
            self.assertEqual(payload['valid_cell_count'], 0)
            self.assertEqual(payload['excluded_run_count'], 1)

    def test_f3b_gate_does_not_inherit_f1_occupancy(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self._write_rows(root)
            summary = root / 'run_1' / 'summary.json'
            item = json.loads(summary.read_text(encoding='utf-8'))
            item['va_hold_fixed_window_quality_pass'] = False
            item['va_hold_airspeed_band_occupancy_pass'] = False
            item['va_hold_mean_airspeed_error_mps'] = -0.28
            item['va_hold_std_airspeed_mps'] = 0.04
            item['va_hold_max_abs_airspeed_error_mps'] = 0.32
            summary.write_text(json.dumps(item) + '\n', encoding='utf-8')
            payload = validate_elevator_moment(root)
            self.assertTrue(payload['f3b1_elevator_moment_validation_pass'])
            self.assertEqual(payload['valid_run_count'], 1)
            self.assertEqual(payload['excluded_run_count'], 0)

    def test_required_airspeed_coverage_is_checked(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self._write_rows(root)
            payload = validate_elevator_moment(
                root,
                required_airspeeds_mps=(6.0, 12.0),
            )
            self.assertFalse(payload['f3b1_elevator_moment_validation_pass'])
            self.assertEqual(payload['missing_required_airspeeds_mps'], [6.0])

    def test_per_cell_validation_allows_operating_point_biases(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for run_index, (va, intercept) in enumerate(
                ((10.0, -0.1), (18.0, -2.0)), start=1,
            ):
                run = root / f'run_{run_index}'
                run.mkdir(parents=True)
                (run / 'summary.json').write_text(json.dumps({
                    'telemetry_schema_version': 20,
                    'f1_attempt_quality': 'valid',
                    'f1_attempt_quality_reason': 'none',
                    'f1_target_condition_established': True,
                    'va_target_config': va,
                    'lambda_target_config': 0.6,
                    'va_hold_measurement_complete': True,
                    'va_hold_measurement_protocol_valid': True,
                    'va_hold_fixed_window_quality_pass': True,
                    'va_hold_airspeed_band_occupancy_pass': True,
                    'va_hold_lambda_band_occupancy_pass': True,
                    'va_hold_mean_airspeed_error_mps': 0.02,
                    'va_hold_std_airspeed_mps': 0.02,
                    'va_hold_max_abs_airspeed_error_mps': 0.05,
                    'va_hold_altitude_rmse_m': 0.5,
                    'va_hold_max_abs_altitude_error_m': 0.8,
                    'primary_failure_cause': 'none',
                    'va_hold_abort_reason': 'none',
                    'any_failsafe': False,
                }) + '\n', encoding='utf-8')
                fields = (
                    'va_hold_measurement_active',
                    'va_hold_measurement_complete',
                    'elevator_aero_wrench_gt_valid',
                    'qbar_selected_pa',
                    'elevator_actual_angle_rad',
                    'elevator_pitch_moment_gt_nm',
                    'current_elevator_pitch_moment_nm',
                    'elevator_dmoment_ddelta_per_q_m3_config',
                    'va_target_config',
                    'lambda_target_config',
                    'alpha_est_rad',
                )
                with (run / 'telemetry.csv').open('w', newline='', encoding='utf-8') as stream:
                    writer = csv.DictWriter(stream, fieldnames=fields)
                    writer.writeheader()
                    for index in range(60):
                        qbar = 60.0 + index
                        elevator = 0.03 + index * 0.001
                        x_value = qbar * elevator
                        writer.writerow({
                            'va_hold_measurement_active': 1,
                            'va_hold_measurement_complete': 0,
                            'elevator_aero_wrench_gt_valid': 1,
                            'qbar_selected_pa': qbar,
                            'elevator_actual_angle_rad': elevator,
                            'elevator_pitch_moment_gt_nm': -0.06 * x_value + intercept,
                            'current_elevator_pitch_moment_nm': -0.06 * x_value,
                            'elevator_dmoment_ddelta_per_q_m3_config': -0.06,
                            'va_target_config': va,
                            'lambda_target_config': 0.6,
                            'alpha_est_rad': 0.04 + index * 0.0001,
                        })
            payload = validate_elevator_moment(
                root,
                required_airspeeds_mps=(10.0, 18.0),
                minimum_valid_cells=2,
            )
            self.assertTrue(payload['f3b1_elevator_moment_validation_pass'])
            self.assertEqual(payload['valid_cell_count'], 2)
            self.assertTrue(all(
                cell['cell_validation_pass']
                for cell in payload['cell_results']
            ))
            self.assertTrue(all(
                cell['fit_model']
                == 'single_run_qbar_delta_plus_qbar_alpha_nuisance'
                for cell in payload['cell_results']
            ))
            self.assertLess(payload['fit_r_squared'], 0.95)

    def test_repeated_runs_use_run_specific_intercepts(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for run_index, intercept in enumerate((-0.08, -0.11, -0.06), start=1):
                self._write_rows(root / f'outer_{run_index}', intercept=intercept)
                run = root / f'outer_{run_index}' / 'run_1'
                summary = json.loads((run / 'summary.json').read_text(encoding='utf-8'))
                summary['va_target_config'] = 6.0
                summary['lambda_target_config'] = 0.3
                (run / 'summary.json').write_text(
                    json.dumps(summary) + '\n', encoding='utf-8')
            payload = validate_elevator_moment(
                root,
                required_airspeeds_mps=(6.0,),
                minimum_valid_cells=1,
            )
            self.assertTrue(payload['f3b1_elevator_moment_validation_pass'])
            cell = payload['cell_results'][0]
            self.assertEqual(
                cell['fit_model'],
                'run_intercepts_plus_qbar_delta_plus_qbar_alpha_nuisance',
            )
            self.assertGreater(cell['fit_r_squared'], 0.95)
            self.assertLess(
                cell['pooled_single_intercept_fit_r_squared'],
                cell['fit_r_squared'],
            )


if __name__ == '__main__':
    unittest.main()
