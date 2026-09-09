"""Summarize A0 ROS/PX4 simulation-speed qualification runs."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


def _finite(value: object, default: float = math.nan) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _json_safe(value: object) -> object:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value


def _mean(values: list[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return float(np.mean(finite)) if finite else math.nan


def _median(values: list[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return float(np.median(finite)) if finite else math.nan


def _min(values: list[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return float(np.min(finite)) if finite else math.nan


def _max(values: list[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return float(np.max(finite)) if finite else math.nan


def summarize_run(summary_path: Path) -> dict[str, Any]:
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    episodes = list(summary.get("episode_rows", []))
    factor = _finite(summary.get("sim_speed_factor"), 1.0)
    rtfs = [_finite(row.get("rl_active_real_time_factor")) for row in episodes]
    stale = [_finite(row.get("rl_active_stale_fraction"), 1.0) for row in episodes]
    ratios = [_finite(row.get("action_step_ratio"), 0.0) for row in episodes]
    obs_sim_dt = [_finite(row.get("observation_sim_dt_mean")) for row in episodes]
    obs_wall_dt = [_finite(row.get("observation_wall_dt_mean")) for row in episodes]
    rl_sim_dt = [_finite(row.get("rl_step_sim_dt_mean")) for row in episodes]
    rl_wall_dt = [_finite(row.get("rl_step_wall_dt_mean")) for row in episodes]
    terminal_seen = [bool(row.get("terminal_seen")) for row in episodes]
    terminal_running = [
        str(row.get("termination_reason", "running")) == "running"
        for row in episodes
    ]
    terminal_reasons = [
        str(row.get("termination_reason", "running")) for row in episodes
    ]
    pass_failures = list(summary.get("pass_failures", []))
    observability_failures: list[str] = []
    if not bool(summary.get("a0_ros_training_smoke_pass")):
        observability_failures.append("training_smoke_failed")
    if any(not item for item in terminal_seen):
        observability_failures.append("terminal_not_observed")
    if any(terminal_running):
        observability_failures.append("terminal_reason_running")
    if any(reason == "a0_rl_action_timeout" for reason in terminal_reasons):
        observability_failures.append("a0_rl_action_timeout")
    if any("failsafe" in reason for reason in terminal_reasons):
        observability_failures.append("px4_failsafe")
    if any("preflight" in reason or "estimator" in reason for reason in terminal_reasons):
        observability_failures.append("persistent_startup_estimator_failure")
    if not rtfs or not all(math.isfinite(item) and item > 0.0 for item in rtfs):
        observability_failures.append("missing_rl_active_rtf")
    if stale and _max(stale) > 0.01:
        observability_failures.append("rl_active_stale_fraction_high")
    if ratios and _min(ratios) < 0.9:
        observability_failures.append("action_step_ratio_low")
    if not obs_sim_dt or not all(0.045 <= item <= 0.055 for item in obs_sim_dt):
        observability_failures.append("observation_sim_dt_not_20hz")
    if not rl_sim_dt or not all(0.09 <= item <= 0.11 for item in rl_sim_dt):
        observability_failures.append("rl_step_sim_dt_not_10hz")

    median_rtf = _median(rtfs)
    return {
        "run_dir": str(summary_path.parent),
        "target_sim_speed_factor": factor,
        "episodes": int(summary.get("episodes", len(episodes))),
        "learning_enabled": bool(summary.get("learning_enabled", True)),
        "training_smoke_pass": bool(summary.get("a0_ros_training_smoke_pass")),
        "observability_pass": not observability_failures,
        "pass_failures": ";".join(str(item) for item in pass_failures),
        "observability_failures": ";".join(observability_failures),
        "terminal_reasons": ";".join(terminal_reasons),
        "median_rl_active_rtf": median_rtf,
        "mean_rl_active_rtf": _mean(rtfs),
        "min_rl_active_rtf": _min(rtfs),
        "max_rl_active_rtf": _max(rtfs),
        "median_rtf_to_target": (
            median_rtf / factor
            if math.isfinite(median_rtf) and factor > 0.0 else math.nan
        ),
        "min_action_step_ratio": _min(ratios),
        "max_rl_active_stale_fraction": _max(stale),
        "mean_observation_sim_dt_s": _mean(obs_sim_dt),
        "mean_observation_sim_hz": (
            1.0 / _mean(obs_sim_dt)
            if math.isfinite(_mean(obs_sim_dt)) and _mean(obs_sim_dt) > 0.0
            else math.nan
        ),
        "mean_observation_wall_dt_s": _mean(obs_wall_dt),
        "mean_rl_step_sim_dt_s": _mean(rl_sim_dt),
        "mean_rl_step_sim_hz": (
            1.0 / _mean(rl_sim_dt)
            if math.isfinite(_mean(rl_sim_dt)) and _mean(rl_sim_dt) > 0.0
            else math.nan
        ),
        "mean_rl_step_wall_dt_s": _mean(rl_wall_dt),
        "mean_episode_wall_time_s": _mean([
            _finite(row.get("episode_wall_time_s")) for row in episodes
        ]),
        "replay_size": int(summary.get("replay_size", 0)),
        "critic_update_count": int(summary.get("critic_update_count", 0)),
        "actor_update_count": int(summary.get("actor_update_count", 0)),
    }


def analyze(root: Path) -> dict[str, Any]:
    summary_paths = sorted(root.glob("*/a0_ros_training_smoke_summary.json"))
    rows = [summarize_run(path) for path in summary_paths]
    root.mkdir(parents=True, exist_ok=True)
    csv_path = root / "a0_sim_speed_qualification.csv"
    if rows:
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
    result = {
        "stage": "A0 simulation-speed qualification",
        "root": str(root),
        "run_count": len(rows),
        "a0_sim_speed_qualification_pass": bool(rows) and all(
            row["observability_pass"] for row in rows
        ),
        "rows": rows,
        "csv": str(csv_path),
        "note": (
            "The pass flag checks A0 observability/action/terminal integrity at "
            "each requested speed.  The measured RTF is reported separately and "
            "is not forced to equal the target on overloaded hardware."
        ),
    }
    json_path = root / "a0_sim_speed_qualification.json"
    json_path.write_text(
        json.dumps(_json_safe(result), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    args = parser.parse_args()
    result = analyze(args.root)
    print(json.dumps(_json_safe(result), indent=2, sort_keys=True))
    return 0 if result["a0_sim_speed_qualification_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
