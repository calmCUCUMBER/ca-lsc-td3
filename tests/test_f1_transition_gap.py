import csv
import json
from pathlib import Path
import tempfile
import unittest

from ca_lsc_td3.evaluation.f1_transition_gap import analyze


PROJECT = Path(__file__).resolve().parents[1]
MODEL = PROJECT / 'PX4-Autopilot/Tools/simulation/gz/models/standard_vtol/model.sdf'


class TransitionGapDiagnosticsTests(unittest.TestCase):
    def test_support_proxy_uses_positive_gazebo_a0_offset(self):
        with tempfile.TemporaryDirectory() as directory:
            point = Path(directory)
            run = point / 'run_1'
            run.mkdir()
            (run / 'summary.json').write_text(json.dumps({
                'lambda_target_config': 0.6,
                'telemetry_schema_version': 10,
                'primary_failure_cause': 'none',
                'f1_measurement_pass': False,
                'va_hold_measurement_complete': True,
            }))
            rows = []
            for index in range(4):
                rows.append({
                    'time_s': index,
                    'va_hold_phase': 'measurement',
                    'va_hold_measurement_active': 1,
                    'va_hold_measurement_complete': 0,
                    'lambda_external_active': 1,
                    'lambda_exec': 0.6,
                    'lambda_target_config': 0.6,
                    'airspeed_mps': 10.0,
                    'va_error_mps': 0.0,
                    'altitude_relative_m': 50.0,
                    'altitude_error_m': 0.0,
                    'vz_up_mps': 0.0,
                    'alpha_valid': 1,
                    'alpha_est_rad': 0.0,
                    'gamma_est_rad': 0.0,
                    'roll_rad': 0.0,
                    'pitch_rad': 0.0,
                    'pitch_setpoint_rad': 0.0,
                    'lift_thrust_n': 20.0,
                    'pusher_thrust_n': 5.0,
                    'pusher_throttle_status': 0.3,
                    'pusher_pi_integral_mps_s': 0.0,
                    'servo_2': 0.0,
                    'lift_collective_thrust_setpoint': 0.4,
                    'vt_unload_altitude_pitch_kp_config': 2.0,
                    'vt_unload_vertical_speed_pitch_kd_config': 2.0,
                    'vt_unload_pitch_min_deg_config': -12.0,
                    'vt_unload_pitch_max_deg_config': 10.0,
                })
            with (run / 'telemetry.csv').open(
                'w', newline='', encoding='utf-8'
            ) as stream:
                writer = csv.DictWriter(stream, fieldnames=tuple(rows[0]))
                writer.writeheader()
                writer.writerows(rows)

            summary, runs, samples = analyze(point, MODEL)

            self.assertEqual(summary['run_count'], 1)
            self.assertGreater(
                runs[0]['wing_vertical_support_proxy_mean_n'], 10.0
            )
            self.assertEqual(runs[0]['pusher_throttle_saturation_fraction'], 0.0)
            self.assertEqual(runs[0]['lift_collective_saturation_fraction'], 0.0)
            self.assertTrue(all(
                item['wing_vertical_support_proxy_n'] > 0.0
                for item in samples
            ))


if __name__ == '__main__':
    unittest.main()
