import csv
import json
import math
import tempfile
import unittest
from pathlib import Path

from ca_lsc_td3.evaluation.f3b_eta_c_behavior import (
    validate_eta_c_behavior,
)


def _smoothstep(value: float, lower: float, upper: float) -> float:
    if value <= lower:
        return 0.0
    if value >= upper:
        return 1.0
    t = (value - lower) / (upper - lower)
    return t * t * (3.0 - 2.0 * t)


class F3BEtaCBehaviorTests(unittest.TestCase):
    def _write_run(
        self,
        root: Path,
        *,
        run_name: str = 'run_1',
        va: float = 12.0,
        lam: float = 0.6,
        slope: float = -0.06,
        eta_bias: float = 0.0,
        modifies_blending: float = 0.0,
    ) -> None:
        run = root / run_name
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
            'eta_c_pressure_gate_lower_pa_config',
            'eta_c_pressure_gate_upper_pa_config',
            'eta_c_q_gate',
            'eta_c_raw',
            'eta_c',
            'eta_c_modifies_px4_blending',
        )
        with (run / 'telemetry.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for index in range(80):
                qbar = 80.0 + index * 0.1
                alpha = 0.04 + 0.0001 * index
                elevator = -0.02 + 0.0004 * index
                request = 0.12 if index % 2 == 0 else -0.10
                config_slope = -0.06
                derivative = qbar * config_slope
                limit = 0.53 if request / derivative > 0.0 else -0.53
                remaining = abs(limit - elevator)
                available_est = abs(derivative) * remaining
                q_gate = _smoothstep(qbar, 15.0, 60.0)
                raw = max(0.0, min(1.0, 1.0 - abs(request) / available_est))
                eta_c = max(0.0, min(1.0, q_gate * raw + eta_bias))
                moment_gt = (
                    -0.2
                    + slope * qbar * elevator
                    + 0.02 * qbar * alpha
                )
                writer.writerow({
                    'va_hold_measurement_active': 1,
                    'va_hold_measurement_complete': 0,
                    'elevator_aero_wrench_gt_valid': 1,
                    'eta_c_valid': 1,
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
                    'eta_c_available_increment_moment_nm': available_est,
                    'eta_c_pressure_gate_lower_pa_config': 15.0,
                    'eta_c_pressure_gate_upper_pa_config': 60.0,
                    'eta_c_q_gate': q_gate,
                    'eta_c_raw': raw,
                    'eta_c': eta_c,
                    'eta_c_modifies_px4_blending': modifies_blending,
                })

    def test_offline_presmoke_passes_when_eta_c_matches_gt(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self._write_run(root)
            payload = validate_eta_c_behavior(root, minimum_samples=50)
            self.assertTrue(payload['f3b2_eta_c_offline_presmoke_pass'])
            self.assertIsNone(payload['f3b2_eta_c_behavior_smoke_pass'])
            self.assertLess(payload['eta_c_gt_rmse'], 0.04)

    def test_eta_c_error_fails_presmoke(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self._write_run(root, eta_bias=-0.25)
            payload = validate_eta_c_behavior(root, minimum_samples=50)
            self.assertFalse(payload['f3b2_eta_c_offline_presmoke_pass'])
            self.assertFalse(payload['checks']['eta_c_gt_rmse'])

    def test_eta_c_must_remain_read_only(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self._write_run(root, modifies_blending=1.0)
            payload = validate_eta_c_behavior(root, minimum_samples=50)
            self.assertFalse(payload['f3b2_eta_c_offline_presmoke_pass'])
            self.assertFalse(payload['checks']['eta_c_modifies_px4_blending_pass'])

    def test_behavior_ordering_modes_are_reported(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self._write_run(root)
            telemetry = root / 'run_1' / 'telemetry.csv'
            with telemetry.open(newline='', encoding='utf-8') as stream:
                rows = list(csv.DictReader(stream))
            fields = list(rows[0].keys()) + [
                'f3b2_test_mode',
                'f3b2_stage_index',
            ]
            for index, row in enumerate(rows):
                stage = min(2, index // 27)
                row['f3b2_test_mode'] = 'q_gate'
                row['f3b2_stage_index'] = stage
                row['qbar_selected_pa'] = 20.0 + stage * 30.0 + 0.01 * index
                qbar = float(row['qbar_selected_pa'])
                row['eta_c_q_gate'] = _smoothstep(qbar, 15.0, 60.0)
                row['eta_c'] = row['eta_c_q_gate']
                row['shadow_fw_pitch_moment_req_nm'] = '0.0'
            with telemetry.open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerows(rows)
            payload = validate_eta_c_behavior(root, minimum_samples=50)
            self.assertEqual(payload['q_gate_behavior_pass'], True)
            self.assertFalse(payload['behavior_ordering_available'])

    def test_remaining_travel_requires_fixed_eta_c_demand(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for stage, elevator in enumerate((0.0, -0.05, -0.10), start=1):
                self._write_run(root, run_name=f'run_{stage}')
                telemetry = root / f'run_{stage}' / 'telemetry.csv'
                with telemetry.open(newline='', encoding='utf-8') as stream:
                    rows = list(csv.DictReader(stream))
                fields = list(rows[0].keys())
                for field in [
                    'f3b2_test_mode',
                    'f3b2_stage_index',
                    'shadow_fw_pitch_moment_req_raw_nm',
                    'shadow_fw_pitch_moment_req_for_eta_c_nm',
                    'elevator_control_angle_for_eta_c_rad',
                    'moment_available_est_nm',
                    'moment_available_gt_nm',
                ]:
                    if field not in fields:
                        fields.append(field)
                qbar = 80.0
                request = 0.70
                slope = -0.06
                derivative = abs(qbar * slope)
                remaining = abs(-0.53 - elevator)
                available = derivative * remaining
                eta = max(0.0, min(1.0, 1.0 - request / available))
                for index, row in enumerate(rows):
                    alpha = 0.04 + 0.0001 * index
                    row['f3b2_test_mode'] = 'remaining_travel'
                    row['f3b2_stage_index'] = stage - 1
                    row['qbar_selected_pa'] = qbar
                    row['elevator_effective_angle_gt_rad'] = elevator
                    row['elevator_control_angle_for_eta_c_rad'] = elevator
                    row['shadow_fw_pitch_moment_req_nm'] = '0.2'
                    row['shadow_fw_pitch_moment_req_raw_nm'] = '0.2'
                    row['shadow_fw_pitch_moment_req_for_eta_c_nm'] = request
                    row['eta_c_available_increment_moment_nm'] = available
                    row['moment_available_est_nm'] = available
                    row['moment_available_gt_nm'] = available
                    row['elevator_remaining_directional_rad'] = remaining
                    row['eta_c_raw'] = eta
                    row['eta_c_q_gate'] = 1.0
                    row['eta_c'] = eta
                    row['elevator_pitch_moment_gt_nm'] = (
                        -0.2 + slope * qbar * elevator + 0.02 * qbar * alpha
                    )
                with telemetry.open('w', newline='', encoding='utf-8') as stream:
                    writer = csv.DictWriter(stream, fieldnames=fields)
                    writer.writeheader()
                    writer.writerows(rows)
            payload = validate_eta_c_behavior(root, minimum_samples=50)
            self.assertTrue(payload['remaining_travel_behavior_pass'])
            remaining = [
                item for item in payload['stage_summaries']
                if item['f3b2_test_mode'] == 'remaining_travel'
            ]
            self.assertGreater(
                remaining[0]['directional_remaining_for_eta_c_mean_rad'],
                remaining[1]['directional_remaining_for_eta_c_mean_rad'],
            )
            self.assertGreater(
                remaining[1]['directional_remaining_for_eta_c_mean_rad'],
                remaining[2]['directional_remaining_for_eta_c_mean_rad'],
            )


if __name__ == '__main__':
    unittest.main()
