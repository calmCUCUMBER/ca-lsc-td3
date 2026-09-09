"""A0 ROS/PX4 environment-contract checks.

This module does not train a policy.  It verifies that real PX4/Gazebo
telemetry can be interpreted as the frozen A0 TD3 interface:

    state -> scripted action -> reward -> next state -> termination bits

The check is intentionally separate from F1/F2/F3 classification.  A failed
flight can still be useful contract evidence if the observation/action/reward
pipeline is coherent and finite.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import fmean
from typing import Any

import numpy as np

from .config import action_to_lambda
from .observations import A0_OBSERVATION_FIELDS, pack_a0_observation
from .reward import COMMON_REWARD_VERSION, debug_reward


TELEMETRY_TO_A0_FIELD = {
    "airspeed_mps": "selected_airspeed_mps",
    "angle_of_attack_rad": "alpha_est_rad",
    "flight_path_angle_rad": "gamma_est_rad",
    "altitude_error_m": "altitude_error_m",
    "vertical_speed_up_mps": "vz_up_mps",
    "pitch_angle_rad": "pitch_rad",
    "pitch_rate_rps": "q_rad_s",
    "lambda_executed": "lambda_exec",
    "lift_rotor_power_w": "lift_power_w",
    "pusher_power_w": "pusher_power_w",
}

RL_DT_S = 0.1
MIN_RECONSTRUCTED_STEPS_FOR_3S_WINDOW = 20
MAX_RECONSTRUCTED_STEPS_FOR_3S_WINDOW = 35


def _number(row: dict[str, str], key: str) -> float:
    try:
        value = float(row.get(key, "nan"))
    except (TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def _mean(values: list[float]) -> float:
    return fmean(values) if values else math.nan


def _json_safe(value: object) -> object:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def row_to_a0_observation(row: dict[str, str]) -> np.ndarray:
    values = {
        a0_field: _number(row, telemetry_field)
        for a0_field, telemetry_field in TELEMETRY_TO_A0_FIELD.items()
    }
    return pack_a0_observation(values)


def action_from_lambda_target(lambda_target: float) -> float:
    if not math.isfinite(lambda_target):
        raise ValueError("lambda target must be finite")
    if not 0.0 <= lambda_target <= 1.0:
        raise ValueError("lambda target must lie in [0, 1]")
    return float(np.clip(2.0 * lambda_target - 1.0, -1.0, 1.0))


def _transition_rows(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    return [
        row for row in rows
        if row.get("command_state", "").split("|", 1)[0] == "TRANSITION_FW"
    ]


def _a0_measurement_rows(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    """Rows eligible for A0 environment-contract reconstruction.

    The contract is about the real RL-like interaction under an externally
    commanded lambda.  Pre-measurement transition rows are useful for F1/F2
    diagnostics, but they may legitimately contain incomplete derived fields.
    They must not poison A0 step reconstruction.
    """
    return [
        row for row in rows
        if row.get("command_state", "").split("|", 1)[0] == "TRANSITION_FW"
        and _number(row, "va_hold_measurement_active") > 0.5
        and _number(row, "lambda_external_active") > 0.5
    ]


def _nearest_row(rows: list[dict[str, str]], target_time_s: float) -> dict[str, str]:
    return min(rows, key=lambda row: abs(_number(row, "time_s") - target_time_s))


def _resample_step_rows(
    rows: list[dict[str, str]],
    *,
    dt_s: float = RL_DT_S,
) -> list[dict[str, str]]:
    if len(rows) < 2:
        return []
    timed_rows = [
        row for row in rows
        if math.isfinite(_number(row, "time_s"))
    ]
    if len(timed_rows) < 2:
        return []
    timed_rows = sorted(timed_rows, key=lambda row: _number(row, "time_s"))
    start = _number(timed_rows[0], "time_s")
    end = _number(timed_rows[-1], "time_s")
    if not math.isfinite(start) or not math.isfinite(end) or end <= start:
        return []
    sample_times: list[float] = []
    current = start
    # Include the last point if it falls within the fixed window.  Consecutive
    # sampled states then form approximately 10 Hz RL transitions.
    while current <= end + 1.0e-9:
        sample_times.append(current)
        current += dt_s
    sampled: list[dict[str, str]] = []
    last_time = math.nan
    for time_s in sample_times:
        row = _nearest_row(timed_rows, time_s)
        row_time = _number(row, "time_s")
        if math.isfinite(last_time) and abs(row_time - last_time) < 1.0e-9:
            continue
        sampled.append(row)
        last_time = row_time
    return sampled


def _target_lambda(rows: list[dict[str, str]]) -> float:
    for key in ("lambda_target_config", "lambda_command"):
        values = [
            _number(row, key)
            for row in rows
            if math.isfinite(_number(row, key))
        ]
        if values:
            return _mean(values[-max(1, min(20, len(values))):])
    return math.nan


def _terminal_flags(rows: list[dict[str, str]]) -> tuple[bool, bool, str]:
    if any(_number(row, "failsafe") > 0.5 for row in rows):
        return True, False, "px4_failsafe"
    abort_reasons = [
        row.get("va_hold_abort_reason", "")
        for row in rows
        if row.get("va_hold_abort_reason", "")
    ]
    if abort_reasons:
        return True, False, abort_reasons[-1]
    if any(_number(row, "va_hold_measurement_complete") > 0.5 for row in rows):
        return True, False, "measurement_complete"
    if any(_number(row, "lambda_characterization_complete") > 0.5 for row in rows):
        return True, False, "lambda_characterization_complete"
    return False, True, "log_ended_without_terminal_event"


def analyze_csv(path: Path) -> dict[str, Any]:
    rows = read_rows(path)
    transition = _transition_rows(rows)
    measurement = _a0_measurement_rows(rows)
    resampled = _resample_step_rows(measurement)
    target = _target_lambda(measurement or transition or rows)
    action = action_from_lambda_target(target) if math.isfinite(target) else math.nan
    lambda_from_action = action_to_lambda(action) if math.isfinite(action) else math.nan

    pass_failures: list[str] = []
    if len(rows) < 2:
        pass_failures.append("too_few_rows")
    if not transition:
        pass_failures.append("no_transition_rows")
    if not measurement:
        pass_failures.append("no_a0_measurement_segment")
    if not math.isfinite(target):
        pass_failures.append("missing_scripted_lambda_target")
    elif abs(lambda_from_action - target) > 1.0e-6:
        pass_failures.append("action_lambda_mapping_error")

    observations: list[np.ndarray] = []
    rewards: list[float] = []
    lambda_errors: list[float] = []
    step_rows = resampled
    for current, nxt in zip(step_rows[:-1], step_rows[1:]):
        try:
            obs = row_to_a0_observation(current)
            next_obs = row_to_a0_observation(nxt)
        except (KeyError, ValueError):
            pass_failures.append("nonfinite_or_missing_a0_observation")
            break
        observations.append(obs)
        observations.append(next_obs)
        lambda_delta = float(next_obs[7] - obs[7])
        terminated, _, _ = _terminal_flags([nxt])
        reward = debug_reward(
            dt_s=max(1.0e-3, _number(nxt, "time_s") - _number(current, "time_s")),
            previous_airspeed_mps=float(obs[0]),
            airspeed_mps=float(next_obs[0]),
            previous_lambda_executed=float(obs[7]),
            lambda_executed=float(next_obs[7]),
            altitude_error_m=float(next_obs[3]),
            pitch_rate_rps=float(next_obs[6]),
            alpha_rad=float(next_obs[1]),
            lambda_delta=lambda_delta,
            success=_number(nxt, "va_hold_measurement_complete") > 0.5,
            failure=terminated and _number(nxt, "va_hold_measurement_complete") <= 0.5,
        )
        if not math.isfinite(reward):
            pass_failures.append("nonfinite_reward")
            break
        rewards.append(reward)
        if math.isfinite(target):
            lambda_errors.append(abs(float(next_obs[7]) - target))

    terminated, truncated, reason = _terminal_flags(rows)
    unique_schedules = sorted({
        row.get("schedule_mode", "")
        for row in rows
        if row.get("schedule_mode", "")
    })
    schema_versions = sorted({
        _number(row, "schema_version")
        for row in rows
        if math.isfinite(_number(row, "schema_version"))
    })
    if any(version < 13.0 for version in schema_versions):
        pass_failures.append("schema_before_transition_allocator")
    if "eta_l" in A0_OBSERVATION_FIELDS or "eta_c" in A0_OBSERVATION_FIELDS:
        pass_failures.append("a0_observation_contains_capability_feature")
    if not observations:
        pass_failures.append("no_a0_steps_reconstructed")
    if not rewards:
        pass_failures.append("no_rewards_reconstructed")
    if rewards and not (
        MIN_RECONSTRUCTED_STEPS_FOR_3S_WINDOW
        <= len(rewards)
        <= MAX_RECONSTRUCTED_STEPS_FOR_3S_WINDOW
    ):
        pass_failures.append("unexpected_reconstructed_step_count")

    summary = {
        "stage": "A0 ROS/PX4 environment contract",
        "source_csv": str(path),
        "a0_ros_contract_pass": not pass_failures,
        "pass_failures": sorted(set(pass_failures)),
        "schema_versions": schema_versions,
        "schedule_modes_seen": unique_schedules,
        "observation_fields": list(A0_OBSERVATION_FIELDS),
        "observation_dim": len(A0_OBSERVATION_FIELDS),
        "contains_eta_l_or_eta_c": (
            "eta_l" in A0_OBSERVATION_FIELDS or "eta_c" in A0_OBSERVATION_FIELDS
        ),
        "telemetry_field_mapping": TELEMETRY_TO_A0_FIELD,
        "row_count": len(rows),
        "transition_row_count": len(transition),
        "a0_measurement_row_count": len(measurement),
        "resampled_state_count": len(resampled),
        "rl_dt_s": RL_DT_S,
        "reconstructed_step_count": len(rewards),
        "scripted_lambda_target": target,
        "scripted_action": action,
        "lambda_from_action": lambda_from_action,
        "mean_lambda_tracking_error": _mean(lambda_errors),
        "max_lambda_tracking_error": max(lambda_errors) if lambda_errors else math.nan,
        "mean_step_reward": _mean(rewards),
        "terminated": terminated,
        "truncated": truncated,
        "termination_reason": reason,
        "energy_reward_weight": 0.0,
        "reward_version": COMMON_REWARD_VERSION,
    }
    return _json_safe(summary)  # type: ignore[return-value]


def analyze_directory(root: Path) -> dict[str, Any]:
    csv_paths = sorted(root.glob("*/telemetry.csv"))
    runs = [analyze_csv(path) for path in csv_paths]
    required_cases = {
        "lambda_0p0",
        "lambda_0p3",
        "lambda_0p6",
        "known_safe_va14_lambda_0p5",
    }
    seen_cases = {path.parent.name for path in csv_paths}
    pass_failures: list[str] = []
    missing = sorted(required_cases - seen_cases)
    if missing:
        pass_failures.append("missing_required_scripted_cases")
    if any(not run["a0_ros_contract_pass"] for run in runs):
        pass_failures.append("one_or_more_contract_runs_failed")
    summary = {
        "stage": "A0 ROS/PX4 environment contract",
        "root": str(root),
        "a0_ros_contract_repeat_pass": not pass_failures,
        "pass_failures": pass_failures,
        "required_cases": sorted(required_cases),
        "seen_cases": sorted(seen_cases),
        "passes": sum(bool(run["a0_ros_contract_pass"]) for run in runs),
        "total": len(runs),
        "runs": runs,
    }
    return _json_safe(summary)  # type: ignore[return-value]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = analyze_directory(args.path) if args.path.is_dir() else analyze_csv(args.path)
    text = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    print(text)
    key = "a0_ros_contract_repeat_pass" if args.path.is_dir() else "a0_ros_contract_pass"
    return 0 if result.get(key) else 1


if __name__ == "__main__":
    raise SystemExit(main())
