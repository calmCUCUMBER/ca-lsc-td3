"""Recompute the common reward on archived A0 ROS/PX4 trajectories."""

from __future__ import annotations

import argparse
from collections import defaultdict
import csv
import json
import math
from pathlib import Path
from dataclasses import replace
from statistics import fmean
from typing import Any

from .reward import (
    COMMON_REWARD_VERSION,
    DebugRewardWeights,
    common_reward_terms,
)
from .ros_contract import row_to_a0_observation


def _terminal_reason(row: dict[str, str]) -> str:
    reason = row.get("a0_rl_terminal_reason", "").strip()
    if not reason or reason == "running":
        reason = row.get("va_hold_abort_reason", "").strip()
    return "" if reason == "running" else reason


def _advance_deadline(deadline_s: float, time_s: float) -> float:
    while deadline_s <= time_s + 1.0e-9:
        deadline_s += 0.1
    return deadline_s


def _reconstruct_episode(
    path: Path,
    *,
    weights: DebugRewardWeights,
) -> dict[str, float | int]:
    previous = None
    previous_time_s: float | None = None
    deadline_s: float | None = None
    running_return = 0.0
    shaping_return = 0.0
    terminal_return = 0.0
    step_count = 0

    with path.open("r", newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            time_s = float(row["time_s"])
            reason = _terminal_reason(row)
            try:
                observation = row_to_a0_observation(row)
            except (KeyError, ValueError):
                observation = None

            if reason and previous is not None and observation is not None:
                assert previous_time_s is not None
                terms = common_reward_terms(
                    dt_s=max(1.0e-3, min(0.2, time_s - previous_time_s)),
                    previous_airspeed_mps=float(previous[0]),
                    airspeed_mps=float(observation[0]),
                    previous_lambda_executed=float(previous[7]),
                    lambda_executed=float(observation[7]),
                    altitude_error_m=float(observation[3]),
                    pitch_rate_rps=float(observation[6]),
                    alpha_rad=float(observation[1]),
                    lambda_delta=float(observation[7] - previous[7]),
                    success=reason == "success",
                    failure=reason != "success",
                    unsafe_failure=(
                        reason not in {"", "success", "forward_transition_timeout"}
                    ),
                    weights=weights,
                )
                running_return += terms.running
                shaping_return += terms.progress_shaping
                terminal_return += terms.terminal
                step_count += 1
                break

            command_state = row.get("command_state", "").split("|", 1)[0]
            try:
                action_stale = float(row.get("a0_rl_action_stale", "0")) > 0.5
            except ValueError:
                action_stale = False
            try:
                handover_ready = float(
                    row.get("a0_rl_handover_ready", "0")
                ) > 0.5
            except ValueError:
                handover_ready = False
            eligible = (
                command_state == "TRANSITION_FW"
                and row.get("a0_rl_control_phase", "") == "rl_active"
                and not action_stale
                # The online observation publisher suppresses non-terminal
                # rows after handover-ready.  Mirror that contract exactly.
                and not handover_ready
                and observation is not None
            )
            if not eligible:
                continue
            if previous is None:
                previous = observation
                previous_time_s = time_s
                deadline_s = time_s + 0.1
                continue
            assert previous_time_s is not None and deadline_s is not None
            if time_s < deadline_s - 1.0e-9:
                continue
            terms = common_reward_terms(
                dt_s=max(1.0e-3, min(0.2, time_s - previous_time_s)),
                previous_airspeed_mps=float(previous[0]),
                airspeed_mps=float(observation[0]),
                previous_lambda_executed=float(previous[7]),
                lambda_executed=float(observation[7]),
                altitude_error_m=float(observation[3]),
                pitch_rate_rps=float(observation[6]),
                alpha_rad=float(observation[1]),
                lambda_delta=float(observation[7] - previous[7]),
                success=False,
                failure=False,
                weights=weights,
            )
            running_return += terms.running
            shaping_return += terms.progress_shaping
            previous = observation
            previous_time_s = time_s
            deadline_s = _advance_deadline(deadline_s, time_s)
            step_count += 1

    return {
        "reconstructed_step_count": step_count,
        "running_reward_return": running_return,
        "progress_shaping_return": shaping_return,
        "terminal_reward_return": terminal_return,
        "reconstructed_legacy_return": running_return + terminal_return,
        "reconstructed_shaped_return": (
            running_return + shaping_return + terminal_return
        ),
    }


def _group_summary(rows: list[dict[str, Any]]) -> dict[str, dict[str, float | int]]:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[str(row["termination_reason"])].append(row)
    result: dict[str, dict[str, float | int]] = {}
    for reason, group in sorted(grouped.items()):
        result[reason] = {
            "episode_count": len(group),
            "archived_legacy_return_mean": fmean(
                float(row["archived_legacy_return"]) for row in group
            ),
            "reconstructed_legacy_return_mean": fmean(
                float(row["reconstructed_legacy_return"]) for row in group
            ),
            "progress_shaping_return_mean": fmean(
                float(row["progress_shaping_return"]) for row in group
            ),
            "reconstructed_shaped_return_mean": fmean(
                float(row["reconstructed_shaped_return"]) for row in group
            ),
        }
    return result


def audit_training_root(root: Path) -> dict[str, Any]:
    summary_path = root / "a0_ros_training_smoke_summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    # Reconstruct an archive with the weights that generated it.  This keeps
    # the audit a fidelity check even after the formal common reward advances.
    archived_weights = DebugRewardWeights()
    if summary.get("reward_version") == "capability_free_transition_potential_v1":
        archived_weights = replace(
            archived_weights,
            unsafe_failure_penalty=archived_weights.failure_penalty,
        )
    rows: list[dict[str, Any]] = []
    reconstruction_errors: list[float] = []
    for archived in summary["episode_rows"]:
        episode = int(archived["episode"])
        attempt = int(archived["attempt"])
        telemetry = root / (
            f"attempt_{attempt:03d}_episode_{episode:03d}/telemetry.csv"
        )
        reconstructed = _reconstruct_episode(
            telemetry,
            weights=archived_weights,
        )
        row = {
            "episode": episode,
            "attempt": attempt,
            "termination_reason": archived["termination_reason"],
            "archived_legacy_return": float(archived["episode_return"]),
            "archived_step_count": int(archived["episode_length"]),
            "lambda_exec_ge_0p9_max_dwell_s": float(
                archived["lambda_exec_ge_0p9_max_dwell_s"]
            ),
            **reconstructed,
        }
        reconstruction_errors.append(abs(
            float(row["archived_legacy_return"])
            - float(row["reconstructed_shaped_return"])
        ))
        rows.append(row)

    groups = _group_summary(rows)
    runaway_rows = [row for row in rows if "runaway" in row["termination_reason"]]
    runaway_mean = fmean(
        float(row["reconstructed_shaped_return"]) for row in runaway_rows
    )
    timeout_mean = float(
        groups["forward_transition_timeout"]["reconstructed_shaped_return_mean"]
    )
    success_mean = float(
        groups["success"]["reconstructed_shaped_return_mean"]
    )
    high_lambda_timeouts = [
        row for row in rows
        if row["termination_reason"] == "forward_transition_timeout"
        and float(row["lambda_exec_ge_0p9_max_dwell_s"]) >= 1.0
    ]
    ordering_pass = runaway_mean < timeout_mean < success_mean
    high_lambda_timeout_guard_pass = all(
        float(row["progress_shaping_return"]) <= 0.0
        for row in high_lambda_timeouts
    )
    reconstruction_pass = (
        fmean(reconstruction_errors) <= 0.01
        and max(reconstruction_errors, default=0.0) <= 0.25
    )
    return {
        "stage": "A0 common reward archived-trajectory audit",
        "source_training_root": str(root),
        "reward_version": COMMON_REWARD_VERSION,
        "episode_count": len(rows),
        "reward_ordering": {
            "runaway_shaped_return_mean": runaway_mean,
            "timeout_shaped_return_mean": timeout_mean,
            "success_shaped_return_mean": success_mean,
            "ordering": "runaway < timeout < success",
        },
        "group_summary": groups,
        "high_lambda_timeout_count": len(high_lambda_timeouts),
        "high_lambda_timeout_progress_shaping_max": max(
            (float(row["progress_shaping_return"]) for row in high_lambda_timeouts),
            default=math.nan,
        ),
        "legacy_return_reconstruction_abs_error_mean": fmean(
            reconstruction_errors
        ),
        "legacy_return_reconstruction_abs_error_max": max(
            reconstruction_errors, default=math.nan
        ),
        "reward_ordering_pass": ordering_pass,
        "high_lambda_timeout_guard_pass": high_lambda_timeout_guard_pass,
        "legacy_return_reconstruction_pass": reconstruction_pass,
        "a0_common_reward_archived_trajectory_audit_pass": (
            ordering_pass
            and high_lambda_timeout_guard_pass
            and reconstruction_pass
        ),
        "episode_rows": rows,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("training_root", type=Path)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    output = args.output or args.training_root / "analysis/reward_ordering_audit.json"
    result = audit_training_root(args.training_root)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["a0_common_reward_archived_trajectory_audit_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
