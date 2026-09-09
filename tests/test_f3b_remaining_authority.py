import csv
import json
import tempfile
import unittest
from pathlib import Path

from ca_lsc_td3.evaluation.f3b_remaining_authority import (
    validate_remaining_authority,
)


class F3BRemainingAuthorityValidationTests(unittest.TestCase):
    def _write_run(
        self,
        root: Path,
        *,
        va: float = 10.0,
        lam: float = 0.6,
        true_slope: float = -0.058,
        config_slope: float = -0.06,
        valid: bool = True,
    ) -> None:
        run = root / f'va{va:g}_lam{lam:g}'
        run.mkdir(parents=True)
        (run / 'summary.json').write_text(json.dumps({
            'telemetry_schema_version': 21,
            'f1_attempt_quality': 'valid',
            'f1_attempt_quality_reason': 'none',
            'f1_target_condition_established': True,
            'va_target_config': va,
            'lambda_target_config': lam,
            'va_hold_measurement_complete': True,
            'va_hold_measurement_protocol_valid': True,
            'va_hold_fixed_window_quality_pass': True,
            'va_hold_airspeed_band_occupancy_pass': True,
            'va_hold_lambda_band_occupancy_pass': True,
            'va_hold_mean_airspeed_error_mps': 0.02,
            'va_hold_std_airspeed_mps': 0.02,
            'va_hold_max_abs_airspeed_error_mps': 0.05,
            'va_hold_altitude_rmse_m': 0.1,
            'va_hold_max_abs_altitude_error_m': 0.2,
            'primary_failure_cause': 'none',
            'va_hold_abort_reason': 'none',
            'any_failsafe': False,
        }) + '\n', encoding='utf-8')
        fields = (
            'va_hold_measurement_active',
            'va_hold_measurement_complete',
            'elevator_aero_wrench_gt_valid',
            'eta_c_valid',
            'qbar_selected_pa',
            'alpha_est_rad',
            'elevator_effective_angle_gt_rad',
            'elevator_pitch_moment_gt_nm',
            'current_elevator_pitch_moment_nm',
            'elevator_dmoment_ddelta_per_q_m3_config',
            'shadow_fw_pitch_moment_req_nm',
            'elevator_min_rad_config',
            'elevator_max_rad_config',
            'elevator_remaining_directional_rad',
            'eta_c_available_increment_moment_nm',
        )
        with (run / 'telemetry.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for index in range(80):
                qbar = 70.0 + 0.2 * index
                alpha = 0.04 + 0.0002 * index
                elevator = -0.02 + 0.0005 * index
                request = 0.2 if index % 2 == 0 else -0.15
                derivative = qbar * config_slope
                if request / derivative > 0.0:
                    limit = 0.53
                else:
                    limit = -0.53
                remaining = abs(limit - elevator)
                intercept = -0.25
                alpha_slope = 0.025
                moment_gt = (
                    intercept
                    + true_slope * qbar * elevator
                    + alpha_slope * qbar * alpha
                )
                writer.writerow({
                    'va_hold_measurement_active': 1,
                    'va_hold_measurement_complete': 0,
                    'elevator_aero_wrench_gt_valid': 1 if valid else 0,
                    'eta_c_valid': 1 if valid else 0,
                    'qbar_selected_pa': qbar,
                    'alpha_est_rad': alpha,
                    'elevator_effective_angle_gt_rad': elevator,
                    'elevator_pitch_moment_gt_nm': moment_gt,
                    'current_elevator_pitch_moment_nm': config_slope * qbar * elevator,
                    'elevator_dmoment_ddelta_per_q_m3_config': config_slope,
                    'shadow_fw_pitch_moment_req_nm': request,
                    'elevator_min_rad_config': -0.53,
                    'elevator_max_rad_config': 0.53,
                    'elevator_remaining_directional_rad': remaining,
                    'eta_c_available_increment_moment_nm': abs(derivative) * remaining,
                })

    def test_directional_remaining_authority_passes_with_local_gt_slope(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self._write_run(root)
            payload = validate_remaining_authority(
                root,
                required_airspeeds_mps=(10.0,),
                minimum_valid_cells=1,
            )
            self.assertTrue(payload['f3b1_remaining_authority_validation_pass'])
            cell = payload['cell_results'][0]
            self.assertLess(
                cell['moment_available_median_relative_error'], 0.04)
            self.assertIn('toward_min', cell['direction_counts'])
            self.assertIn('toward_max', cell['direction_counts'])

    def test_wrong_available_moment_scale_fails(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self._write_run(root, true_slope=-0.03, config_slope=-0.06)
            payload = validate_remaining_authority(
                root,
                required_airspeeds_mps=(10.0,),
                minimum_valid_cells=1,
            )
            self.assertFalse(payload['f3b1_remaining_authority_validation_pass'])
            self.assertFalse(
                payload['cell_results'][0]['checks']['median_relative_error']
            )

    def test_requires_valid_eta_c_and_ground_truth_rows(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self._write_run(root, valid=False)
            payload = validate_remaining_authority(
                root,
                required_airspeeds_mps=(10.0,),
                minimum_valid_cells=1,
            )
            self.assertFalse(payload['f3b1_remaining_authority_validation_pass'])
            self.assertEqual(payload['valid_cell_count'], 0)
            self.assertEqual(payload['excluded_run_count'], 1)

    def test_required_airspeed_coverage_is_checked(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self._write_run(root, va=10.0)
            payload = validate_remaining_authority(
                root,
                required_airspeeds_mps=(10.0, 14.0),
                minimum_valid_cells=2,
            )
            self.assertFalse(payload['f3b1_remaining_authority_validation_pass'])
            self.assertEqual(payload['missing_required_airspeeds_mps'], [14.0])


if __name__ == '__main__':
    unittest.main()
