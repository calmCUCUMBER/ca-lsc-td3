import csv
import math
import tempfile
import unittest
from pathlib import Path

from ca_lsc_td3.evaluation.f2_gust import classify_point, evaluate_run


class F2GustTests(unittest.TestCase):
    def _write_run(self, path: Path, *, hard=False, incomplete=False):
        fields = [
            'time_s', 'schema_version', 'condition_id', 'gust_phase',
            'gust_amplitude_mps_config', 'gust_duration_s_config',
            'gust_recovery_s_config', 'gust_bridge_ready',
            'wind_cmd_e_enu_mps', 'wind_cmd_n_enu_mps',
            'wind_cmd_u_enu_mps', 'wind_actual_e_enu_mps',
            'wind_actual_n_enu_mps', 'wind_actual_u_enu_mps',
            'va_target_config', 'lambda_target_config',
            'lambda_target_tolerance', 'lambda_exec',
            'selected_airspeed_mps', 'altitude_error_m', 'vz_up_mps',
            'alpha_valid', 'alpha_est_rad', 'failsafe',
            'va_hold_abort_reason',
        ]
        rows = []
        time_s = 0.0
        phases = [('gust_baseline', 11), ('gust_exposure', 41)]
        if not incomplete:
            phases.append(('gust_recovery', 41))
        for phase, count in phases:
            for index in range(count):
                if phase == 'gust_exposure':
                    elapsed = index * 0.05
                    wind = 2.0 * (1.0 - math.cos(2.0 * math.pi * elapsed / 2.0))
                else:
                    wind = 0.0
                rows.append({
                    'time_s': time_s,
                    'schema_version': 16,
                    'condition_id': 'test',
                    'gust_phase': phase,
                    'gust_amplitude_mps_config': 4.0,
                    'gust_duration_s_config': 2.0,
                    'gust_recovery_s_config': 2.0,
                    'gust_bridge_ready': 1,
                    'wind_cmd_e_enu_mps': wind,
                    'wind_cmd_n_enu_mps': 0.0,
                    'wind_cmd_u_enu_mps': 0.0,
                    'wind_actual_e_enu_mps': wind,
                    'wind_actual_n_enu_mps': 0.0,
                    'wind_actual_u_enu_mps': 0.0,
                    'va_target_config': 10.0,
                    'lambda_target_config': 0.6,
                    'lambda_target_tolerance': 0.03,
                    'lambda_exec': 0.6,
                    'selected_airspeed_mps': 10.0,
                    'altitude_error_m': 5.5 if hard and phase == 'gust_exposure' else 0.1,
                    'vz_up_mps': 0.1,
                    'alpha_valid': 1,
                    'alpha_est_rad': 0.05,
                    'failsafe': 0,
                    'va_hold_abort_reason': '',
                })
                time_s += 0.05
        with path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)

    def test_nominal_profile_and_recovery_are_valid(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'telemetry.csv'
            self._write_run(path)
            result = evaluate_run(path)
            self.assertEqual(result['f2b_attempt_quality'], 'valid')
            self.assertEqual(result['f2b_run_outcome'], 'N')
            self.assertTrue(result['f2b_gust_profile_valid'])
            self.assertTrue(result['f2b_recovery_pass'])

    def test_one_recorder_tick_ack_lag_is_protocol_valid(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'telemetry.csv'
            self._write_run(path)
            with path.open(encoding='utf-8') as stream:
                rows = list(csv.DictReader(stream))
            previous = 0.0
            for row in rows:
                if row['gust_phase'] == 'gust_exposure':
                    current = float(row['wind_cmd_e_enu_mps'])
                    row['wind_actual_e_enu_mps'] = previous
                    previous = current
            with path.open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
            result = evaluate_run(path)
            self.assertTrue(result['f2b_gust_profile_valid'])

    def test_sustained_hard_altitude_error_is_physical_unsafe(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'telemetry.csv'
            self._write_run(path, hard=True)
            result = evaluate_run(path)
            self.assertEqual(result['f2b_attempt_quality'], 'valid')
            self.assertEqual(result['f2b_run_outcome'], 'X')

    def test_incomplete_protocol_is_not_a_physical_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'telemetry.csv'
            self._write_run(path, incomplete=True)
            result = evaluate_run(path)
            self.assertEqual(result['f2b_attempt_quality'], 'protocol_invalid')
            self.assertEqual(result['f2b_run_outcome'], 'unresolved')

    def test_hard_violation_after_gust_onset_remains_valid_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'telemetry.csv'
            self._write_run(path, hard=True, incomplete=True)
            result = evaluate_run(path)
            self.assertEqual(result['f2b_attempt_quality'], 'valid')
            self.assertEqual(result['f2b_run_outcome'], 'X')

    def test_point_classification_requires_repeated_hard_failure(self):
        nominal = {
            'f2b_attempt_quality': 'valid', 'f2b_run_outcome': 'N',
            'f2b_recovery_pass': True, 'f2b_max_abs_delta_va_mps': 0.2,
            'f2b_max_abs_altitude_error_m': 0.2,
        }
        unsafe = {**nominal, 'f2b_run_outcome': 'X', 'f2b_recovery_pass': False}
        self.assertEqual(
            classify_point([nominal] * 4 + [unsafe])['outcome_class'], 'B'
        )
        self.assertEqual(
            classify_point([nominal] * 3 + [unsafe] * 2)['outcome_class'], 'X'
        )


if __name__ == '__main__':
    unittest.main()
