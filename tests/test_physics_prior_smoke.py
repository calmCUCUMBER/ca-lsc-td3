from __future__ import annotations

import csv
import tempfile
from pathlib import Path
import unittest

from ca_lsc_td3.evaluation.physics_prior_smoke import analyze_csv, analyze_directory


FIELDS = [
    "time_s",
    "schema_version",
    "schedule_mode",
    "command_state",
    "vtol_state",
    "failsafe",
    "physics_prior_active",
    "selected_airspeed_mps",
    "eta_l",
    "eta_l_valid",
    "eta_c",
    "eta_c_valid",
    "lambda_phy",
    "lambda_max_hard",
    "lambda_command",
    "lambda_exec",
    "lambda_status_age_s",
    "airspeed_age_s",
    "vt_transition_airspeed_mps_config",
    "physics_prior_release_lambda_config",
    "physics_prior_release_dwell_s_config",
    "eta_l_gt",
    "pusher_throttle_external_active",
    "altitude_error_m",
    "vz_up_mps",
    "alpha_est_rad",
    "alpha_valid",
    "lambda_shield_projection_intervention",
    "lambda_shield_total_intervention",
    "lambda_shield_emergency_recovery",
]


def _row(index: int, total: int) -> dict[str, object]:
    frac = index / max(total - 1, 1)
    lambda_value = min(1.0, 0.02 + 1.20 * frac)
    return {
        "time_s": index * 0.05,
        "schema_version": 24,
        "schedule_mode": "physics_prior_only",
        "command_state": "TRANSITION_FW",
        "vtol_state": 1,
        "failsafe": 0,
        "physics_prior_active": 1,
        "selected_airspeed_mps": 6.0 + 10.0 * frac,
        "eta_l": 0.05 + 0.90 * frac,
        "eta_l_valid": 1,
        "eta_c": 0.10 + 0.85 * frac,
        "eta_c_valid": 1,
        "lambda_phy": lambda_value,
        "lambda_max_hard": 0.95,
        "lambda_command": min(lambda_value, 0.95),
        "lambda_exec": min(lambda_value, 0.95),
        "lambda_status_age_s": 0.02,
        "airspeed_age_s": 0.02,
        "vt_transition_airspeed_mps_config": 13.0,
        "physics_prior_release_lambda_config": 0.95,
        "physics_prior_release_dwell_s_config": 2.0,
        "eta_l_gt": 0.05 + 0.90 * frac,
        "pusher_throttle_external_active": 1,
        "altitude_error_m": 0.10,
        "vz_up_mps": 0.02,
        "alpha_est_rad": 0.12,
        "alpha_valid": 1,
        "lambda_shield_projection_intervention": 0.0,
        "lambda_shield_total_intervention": 0.0,
        "lambda_shield_emergency_recovery": 0,
    }


def _write(path: Path, *, hard_violation: bool = False) -> None:
    rows = []
    rows.append({**_row(0, 1), "command_state": "HOLD_MC", "vtol_state": 3})
    for index in range(180):
        row = _row(index, 180)
        if hard_violation and index == 20:
            row["lambda_max_hard"] = 0.20
            row["lambda_command"] = 0.30
        rows.append(row)
    rows.append({**_row(179, 180), "command_state": "HOLD_FW", "vtol_state": 4})
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


class PhysicsPriorSmokeTests(unittest.TestCase):
    def test_nominal_prior_trace_passes(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            csv_path = Path(temp) / "telemetry.csv"
            _write(csv_path)
            result = analyze_csv(csv_path, plot=False)
        self.assertTrue(result["physics_prior_smoke_pass"])
        self.assertEqual(result["pass_failures"], [])
        self.assertGreater(result["lambda_phy_late_mean"], result["lambda_phy_early_mean"])

    def test_hard_limit_violation_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            csv_path = Path(temp) / "telemetry.csv"
            _write(csv_path, hard_violation=True)
            result = analyze_csv(csv_path, plot=False)
        self.assertFalse(result["physics_prior_smoke_pass"])
        self.assertIn("command_exceeds_hard_limit", result["pass_failures"])

    def test_directory_aggregate(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            for index in (1, 2, 3):
                run = root / f"run_{index}"
                run.mkdir()
                _write(run / "telemetry.csv")
            result = analyze_directory(root, plot=False)
            self.assertTrue(result["physics_prior_repeat_pass"])
            self.assertEqual(result["passes"], 3)
            self.assertEqual(result["total"], 3)


if __name__ == "__main__":
    unittest.main()
