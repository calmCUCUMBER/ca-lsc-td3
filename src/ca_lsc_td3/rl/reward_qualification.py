"""Offline qualification of the final capability-free common reward."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, replace
import json
import math
from pathlib import Path
from statistics import fmean
from typing import Any

from .reward import DebugRewardWeights, common_reward_terms
from .ros_contract import row_to_a0_observation


@dataclass(frozen=True)
class Candidate:
    name: str
    unsafe_penalty: float
    vertical_speed_weight: float


CANDIDATES = (
    Candidate("potential_v1", 20.0, 0.0),
    Candidate("dense_vz_only", 20.0, 0.10),
    Candidate("terminal_unsafe_30", 30.0, 0.0),
    Candidate("hybrid_unsafe_30_vz_0p025", 30.0, 0.025),
    Candidate("terminal_unsafe_35", 35.0, 0.0),
    Candidate("dense_vz_0p2", 20.0, 0.20),
)
SELECTED_CANDIDATE = "terminal_unsafe_35"
MIN_SAFETY_ORDERING_MARGIN = 5.0


def _terminal_reason(row: dict[str, str]) -> str:
    reason = row.get("a0_rl_terminal_reason", "").strip()
    if not reason or reason == "running":
        reason = row.get("va_hold_abort_reason", "").strip()
    return "" if reason == "running" else reason


def _advance_deadline(deadline_s: float, time_s: float) -> float:
    while deadline_s <= time_s + 1.0e-9:
        deadline_s += 0.1
    return deadline_s


def _trajectory_return(path: Path, candidate: Candidate) -> float:
    weights = replace(
        DebugRewardWeights(),
        unsafe_failure_penalty=candidate.unsafe_penalty,
    )
    previous = None
    previous_time_s: float | None = None
    deadline_s: float | None = None
    total = 0.0
    with path.open("r", newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            time_s = float(row["time_s"])
            reason = _terminal_reason(row)
            try:
                observation = row_to_a0_observation(row)
            except (KeyError, ValueError):
                observation = None

            terminal = bool(reason and previous is not None and observation is not None)
            if terminal:
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
                        reason not in {"success", "forward_transition_timeout"}
                    ),
                    weights=weights,
                )
                return total + terms.total - candidate.vertical_speed_weight * float(
                    observation[4]
                ) ** 2

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
            total += terms.total - candidate.vertical_speed_weight * float(
                observation[4]
            ) ** 2
            previous = observation
            previous_time_s = time_s
            deadline_s = _advance_deadline(deadline_s, time_s)
    raise ValueError(f"trajectory has no reconstructable terminal: {path}")


def qualify(training_root: Path) -> dict[str, Any]:
    summary = json.loads(
        (training_root / "a0_ros_training_smoke_summary.json").read_text(
            encoding="utf-8"
        )
    )
    trajectories = []
    for row in summary["episode_rows"]:
        episode = int(row["episode"])
        attempt = int(row["attempt"])
        trajectories.append((
            str(row["termination_reason"]),
            training_root
            / f"attempt_{attempt:03d}_episode_{episode:03d}/telemetry.csv",
        ))

    candidate_rows = []
    for candidate in CANDIDATES:
        grouped: dict[str, list[float]] = {}
        for reason, path in trajectories:
            grouped.setdefault(reason, []).append(
                _trajectory_return(path, candidate)
            )
        means = {reason: fmean(values) for reason, values in grouped.items()}
        timeout = means["forward_transition_timeout"]
        success = means["success"]
        unsafe_means = {
            reason: value for reason, value in means.items()
            if reason not in {"success", "forward_transition_timeout"}
        }
        safety_margin = min(timeout - value for value in unsafe_means.values())
        ordering_pass = all(value < timeout for value in unsafe_means.values())
        ordering_pass = ordering_pass and timeout < success
        candidate_rows.append({
            "candidate": candidate.name,
            "unsafe_failure_penalty": candidate.unsafe_penalty,
            "vertical_speed_weight": candidate.vertical_speed_weight,
            "success_return_mean": success,
            "timeout_return_mean": timeout,
            "altitude_runaway_return_mean": means["va_hold_altitude_runaway"],
            "vertical_speed_runaway_return_mean": means[
                "va_hold_vertical_speed_runaway"
            ],
            "minimum_safety_ordering_margin": safety_margin,
            "ordering_pass": ordering_pass,
            "qualification_margin_pass": (
                ordering_pass and safety_margin >= MIN_SAFETY_ORDERING_MARGIN
            ),
        })

    selected = next(
        row for row in candidate_rows if row["candidate"] == SELECTED_CANDIDATE
    )
    return {
        "stage": "A0 common reward offline qualification",
        "source_training_root": str(training_root),
        "source_reward_version": summary.get("reward_version"),
        "episode_count": len(trajectories),
        "minimum_required_safety_ordering_margin": (
            MIN_SAFETY_ORDERING_MARGIN
        ),
        "selected_candidate": SELECTED_CANDIDATE,
        "selection_reason": (
            "Smallest tested terminal-only change with at least 5 return units "
            "of unsafe-below-timeout ordering margin; leaves success and "
            "timeout trajectory returns unchanged."
        ),
        "candidate_rows": candidate_rows,
        "selected_candidate_row": selected,
        "common_reward_offline_qualification_pass": bool(
            selected["qualification_margin_pass"]
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("training_root", type=Path)
    parser.add_argument("--output-dir", type=Path, default=None)
    args = parser.parse_args()
    output_dir = args.output_dir or args.training_root / "analysis"
    result = qualify(args.training_root)
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "common_reward_qualification.json"
    csv_path = output_dir / "common_reward_candidates.csv"
    json_path.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=list(result["candidate_rows"][0].keys()),
        )
        writer.writeheader()
        writer.writerows(result["candidate_rows"])
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["common_reward_offline_qualification_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
