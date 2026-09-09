import csv
import json
import tempfile
import unittest
from pathlib import Path

from ca_lsc_td3.evaluation.f3b_control_authority import validate_eta_c_smoke


class F3BControlAuthorityTests(unittest.TestCase):
    def _write_smoke(
        self,
        root: Path,
        *,
        schema: int = 19,
        modifies: int = 0,
        f1_measurement_pass: bool = True,
        va_mean_error: float = 0.15,
        va_std: float = 0.22,
        va_max_error: float = 0.39,
    ) -> None:
        run = root / 'run_1'
        run.mkdir(parents=True)
        (run / 'summary.json').write_text(json.dumps({
            'telemetry_schema_version': schema,
            'f1_attempt_quality': 'valid',
            'f1_measurement_pass': f1_measurement_pass,
            'va_target_config': 12.0,
            'va_hold_mean_airspeed_mps': 12.0 + va_mean_error,
            'va_hold_mean_airspeed_error_mps': va_mean_error,
            'va_hold_std_airspeed_mps': va_std,
            'va_hold_max_abs_airspeed_error_mps': va_max_error,
            'va_hold_measurement_complete': True,
            'va_hold_measurement_protocol_valid': True,
            'lambda_target_reached': True,
            'primary_failure_cause': 'none',
            'va_hold_abort_reason': 'none',
            'any_failsafe': False,
        }) + '\n', encoding='utf-8')
        fields = (
            'schema_version', 'schedule_mode', 'va_hold_phase',
            'va_hold_measurement_active', 'va_hold_measurement_complete',
            'eta_c_valid', 'qbar_selected_pa', 'eta_c_dynamic_pressure_pa',
            'eta_c_pressure_gate_lower_pa_config',
            'eta_c_pressure_gate_upper_pa_config', 'eta_c_q_gate',
            'eta_c_pressure_gate', 'shadow_fw_pitch_moment_req_nm',
            'shadow_pitch_moment_increment_nm', 'elevator_actual_angle_rad',
            'elevator_actual_angle_gt_rad', 'moment_available_est_nm',
            'eta_c_available_increment_moment_nm',
            'elevator_remaining_directional_rad',
            'eta_c_remaining_elevator_angle_rad', 'eta_c_raw',
            'eta_c_moment_margin', 'eta_c', 'eta_c_modifies_px4_blending',
        )
        with (run / 'telemetry.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for _ in range(10):
                writer.writerow({
                    'schema_version': schema,
                    'schedule_mode': 'va_hold_target',
                    'va_hold_phase': 'airspeed_settle',
                    'va_hold_measurement_active': 0,
                    'va_hold_measurement_complete': 0,
                    'eta_c_valid': 1,
                    'qbar_selected_pa': 5,
                    'eta_c_dynamic_pressure_pa': 5,
                    'eta_c_pressure_gate_lower_pa_config': 15,
                    'eta_c_pressure_gate_upper_pa_config': 60,
                    'eta_c_q_gate': 0,
                    'eta_c_pressure_gate': 0,
                    'shadow_fw_pitch_moment_req_nm': 0.1,
                    'shadow_pitch_moment_increment_nm': 0.1,
                    'elevator_actual_angle_rad': 0.02,
                    'elevator_actual_angle_gt_rad': 0.02,
                    'moment_available_est_nm': 3.0,
                    'eta_c_available_increment_moment_nm': 3.0,
                    'elevator_remaining_directional_rad': 0.4,
                    'eta_c_remaining_elevator_angle_rad': 0.4,
                    'eta_c_raw': 0.9,
                    'eta_c_moment_margin': 0.9,
                    'eta_c': 0.0,
                    'eta_c_modifies_px4_blending': modifies,
                })
            for _ in range(60):
                writer.writerow({
                    'schema_version': schema,
                    'schedule_mode': 'va_hold_target',
                    'va_hold_phase': 'measurement',
                    'va_hold_measurement_active': 1,
                    'va_hold_measurement_complete': 0,
                    'eta_c_valid': 1,
                    'qbar_selected_pa': 85,
                    'eta_c_dynamic_pressure_pa': 85,
                    'eta_c_pressure_gate_lower_pa_config': 15,
                    'eta_c_pressure_gate_upper_pa_config': 60,
                    'eta_c_q_gate': 1,
                    'eta_c_pressure_gate': 1,
                    'shadow_fw_pitch_moment_req_nm': 0.2,
                    'shadow_pitch_moment_increment_nm': 0.2,
                    'elevator_actual_angle_rad': 0.01,
                    'elevator_actual_angle_gt_rad': 0.01,
                    'moment_available_est_nm': 4.0,
                    'eta_c_available_increment_moment_nm': 4.0,
                    'elevator_remaining_directional_rad': 0.5,
                    'eta_c_remaining_elevator_angle_rad': 0.5,
                    'eta_c_raw': 0.95,
                    'eta_c_moment_margin': 0.95,
                    'eta_c': 0.95,
                    'eta_c_modifies_px4_blending': modifies,
                })

    def test_eta_c_smoke_passes_with_read_only_schema19_fields(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self._write_smoke(root)
            payload = validate_eta_c_smoke(root)
            self.assertTrue(payload['f3b_eta_c_instrumentation_smoke_pass'])
            self.assertEqual(payload['eta_c_valid_measurement_sample_count'], 60)
            self.assertAlmostEqual(payload['low_q_gate_max'], 0.0)
            self.assertAlmostEqual(payload['measurement_q_gate_mean'], 1.0)

    def test_eta_c_smoke_uses_f3b_target_gate_not_f1_occupancy(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self._write_smoke(root, f1_measurement_pass=False)
            payload = validate_eta_c_smoke(root)
            self.assertTrue(payload['f3b_eta_c_instrumentation_smoke_pass'])
            self.assertTrue(payload['checks']['target_condition_established'])

    def test_eta_c_smoke_rejects_poor_f3b_target_condition(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self._write_smoke(root, f1_measurement_pass=True, va_std=0.35)
            payload = validate_eta_c_smoke(root)
            self.assertFalse(payload['f3b_eta_c_instrumentation_smoke_pass'])
            self.assertFalse(payload['checks']['target_condition_established'])
            result = payload['f3b_target_condition_results'][0]
            self.assertFalse(result['checks']['std_airspeed'])

    def test_eta_c_smoke_rejects_old_schema_and_blend_mutation(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self._write_smoke(root, schema=18, modifies=1)
            payload = validate_eta_c_smoke(root)
            self.assertFalse(payload['f3b_eta_c_instrumentation_smoke_pass'])
            self.assertFalse(payload['checks']['schema_ge_19'])
            self.assertFalse(payload['checks']['eta_c_read_only'])


if __name__ == '__main__':
    unittest.main()
