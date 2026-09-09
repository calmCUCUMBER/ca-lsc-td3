import importlib.util
import unittest
from pathlib import Path


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / 'scripts'
    / 'summarize_transition_allocation_verification.py'
)
SPEC = importlib.util.spec_from_file_location(
    'transition_allocation_summary', SCRIPT
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class TransitionAllocationSummaryTests(unittest.TestCase):
    def test_nonzero_target_not_reached_is_not_architecture_evidence(self):
        row = MODULE._attempt_row(Path('/tmp/attempt'), {
            'telemetry_schema_version': 13.0,
            'va_target_config': 14.0,
            'lambda_target_config': 0.7,
            'max_lambda_exec': 0.0,
            'lambda_target_reached': False,
            'target_lambda_attitude_sync_samples': 0,
            'target_lambda_attitude_sync_pass': False,
            # A ramp-wide endpoint comparison may still be self-consistent.
            'transition_lambda_attitude_sync_pass': True,
            'f1_measurement_pass': False,
        })

        self.assertFalse(row['architecture_verified'])
        self.assertEqual(row['verification_status'], 'target_not_reached')

    def test_f1_failure_does_not_override_target_band_synchronization(self):
        row = MODULE._attempt_row(Path('/tmp/attempt'), {
            'telemetry_schema_version': 13.0,
            'va_target_config': 14.0,
            'lambda_target_config': 0.7,
            'max_lambda_exec': 0.7,
            'lambda_target_reached': True,
            'target_lambda_attitude_sync_samples': 20,
            'target_mc_pitch_weight_lambda_sync_rmse': 1e-7,
            'target_fw_pitch_weight_lambda_sync_rmse': 1e-7,
            'target_lambda_attitude_sync_pass': True,
            'f1_measurement_pass': False,
        })

        self.assertTrue(row['architecture_verified'])
        self.assertEqual(row['verification_status'], 'verified')
        self.assertFalse(row['flight_measurement_pass'])


if __name__ == '__main__':
    unittest.main()
