from __future__ import annotations

import csv
import tempfile
from pathlib import Path
import unittest

import numpy as np

from ca_lsc_td3.rl.observations import A0_OBSERVATION_FIELDS
from ca_lsc_td3.rl.ros_contract import (
    action_from_lambda_target,
    analyze_csv,
    analyze_directory,
    row_to_a0_observation,
)


FIELDS = [
    "time_s",
    "schema_version",
    "schedule_mode",
    "command_state",
    "selected_airspeed_mps",
    "alpha_est_rad",
    "gamma_est_rad",
    "altitude_error_m",
    "vz_up_mps",
    "pitch_rad",
    "q_rad_s",
    "lambda_exec",
    "lift_power_w",
    "pusher_power_w",
    "lambda_target_config",
    "lambda_command",
    "va_hold_measurement_active",
    "va_hold_measurement_complete",
    "lambda_characterization_complete",
    "lambda_external_active",
    "failsafe",
    "va_hold_abort_reason",
]


def _row(index: int, *, lam: float = 0.3, complete: bool = False) -> dict[str, object]:
    return {
        "time_s": 0.05 * index,
        "schema_version": 24,
        "schedule_mode": "va_hold_target",
        "command_state": "TRANSITION_FW",
        "selected_airspeed_mps": 12.0,
        "alpha_est_rad": 0.08,
        "gamma_est_rad": 0.01,
        "altitude_error_m": 0.1,
        "vz_up_mps": 0.0,
        "pitch_rad": 0.09,
        "q_rad_s": 0.01,
        "lambda_exec": lam,
        "lift_power_w": 800.0,
        "pusher_power_w": 60.0,
        "lambda_target_config": lam,
        "lambda_command": lam,
        "va_hold_measurement_active": 1,
        "va_hold_measurement_complete": int(complete),
        "lambda_characterization_complete": int(complete),
        "lambda_external_active": 1,
        "failsafe": 0,
        "va_hold_abort_reason": "",
    }


def _write(path: Path, *, lam: float = 0.3, complete: bool = True) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        for index in range(61):
            writer.writerow(_row(index, lam=lam, complete=complete and index == 60))


class A0RosContractTests(unittest.TestCase):
    def test_action_mapping_matches_a0_contract(self) -> None:
        self.assertAlmostEqual(action_from_lambda_target(0.0), -1.0)
        self.assertAlmostEqual(action_from_lambda_target(0.3), -0.4)
        self.assertAlmostEqual(action_from_lambda_target(0.6), 0.2)
        self.assertAlmostEqual(action_from_lambda_target(1.0), 1.0)

    def test_row_to_a0_observation_uses_ten_dimensional_contract(self) -> None:
        obs = row_to_a0_observation({k: str(v) for k, v in _row(0).items()})
        self.assertEqual(obs.shape, (10,))
        self.assertEqual(len(A0_OBSERVATION_FIELDS), 10)
        self.assertTrue(np.all(np.isfinite(obs)))
        self.assertNotIn("eta_l", A0_OBSERVATION_FIELDS)
        self.assertNotIn("eta_c", A0_OBSERVATION_FIELDS)

    def test_csv_contract_passes_with_finite_scripted_trace(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "telemetry.csv"
            _write(path, lam=0.6)
            result = analyze_csv(path)
        self.assertTrue(result["a0_ros_contract_pass"])
        self.assertEqual(result["observation_dim"], 10)
        self.assertEqual(result["energy_reward_weight"], 0.0)
        self.assertFalse(result["contains_eta_l_or_eta_c"])
        self.assertTrue(result["terminated"])
        self.assertFalse(result["truncated"])
        self.assertGreaterEqual(result["reconstructed_step_count"], 20)
        self.assertLessEqual(result["reconstructed_step_count"], 35)

    def test_directory_requires_scripted_cases(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            for name, lam in (
                ("lambda_0p0", 0.0),
                ("lambda_0p3", 0.3),
                ("lambda_0p6", 0.6),
                ("known_safe_va14_lambda_0p5", 0.5),
            ):
                _write(root / name / "telemetry.csv", lam=lam)
            result = analyze_directory(root)
        self.assertTrue(result["a0_ros_contract_repeat_pass"])
        self.assertEqual(result["passes"], 4)
        self.assertEqual(result["total"], 4)


if __name__ == "__main__":
    unittest.main()
