#!/usr/bin/env python3
"""Re-evaluate and summarize the nine schema-13 allocation checks.

Flight feasibility and PX4 allocation synchronization are deliberately kept
separate.  A point verifies the architecture only after its requested lambda
has actually been reached for enough samples; a self-consistent lambda=0
prefix cannot validate a non-zero target.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ROS_PACKAGE = PROJECT_ROOT / "ros2_ws" / "src" / "ca_lsc_transition"
sys.path.insert(0, str(ROS_PACKAGE))

from ca_lsc_transition.evaluate_run import evaluate  # noqa: E402


EXPECTED_POINTS = (
    ("va8_lam0p2", 8.0, 0.2),
    ("va8_lam0p4", 8.0, 0.4),
    ("va10_lam0p3", 10.0, 0.3),
    ("va10_lam0p5", 10.0, 0.5),
    ("va12_lam0p4", 12.0, 0.4),
    ("va12_lam0p6", 12.0, 0.6),
    ("va14_lam0p5", 14.0, 0.5),
    ("va14_lam0p7", 14.0, 0.7),
    ("va18_lam0p8", 18.0, 0.8),
)
MIN_TARGET_SYNC_SAMPLES = 10


def _json_safe(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _number(value: Any) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def _matches(value: Any, expected: float) -> bool:
    number = _number(value)
    return math.isfinite(number) and abs(number - expected) <= 1e-6


def _retry_index(row: dict[str, Any]) -> int:
    match = re.search(r'_retry_(\d+)$', Path(row['run_dir']).name)
    return int(match.group(1)) if match else 0


def _attempt_row(run_dir: Path, summary: dict[str, Any]) -> dict[str, Any]:
    schema = _number(summary.get("telemetry_schema_version"))
    target_reached = bool(summary.get("lambda_target_reached"))
    samples = int(summary.get("target_lambda_attitude_sync_samples") or 0)
    target_sync_pass = bool(summary.get("target_lambda_attitude_sync_pass"))
    architecture_verified = bool(
        schema == 13.0
        and target_reached
        and samples >= MIN_TARGET_SYNC_SAMPLES
        and target_sync_pass
    )
    if architecture_verified:
        status = "verified"
    elif schema != 13.0:
        status = "wrong_schema"
    elif not target_reached:
        status = "target_not_reached"
    elif samples < MIN_TARGET_SYNC_SAMPLES:
        status = "insufficient_target_samples"
    else:
        status = "weight_mismatch"
    return {
        "run_dir": str(run_dir.resolve()),
        "telemetry_schema_version": schema,
        "va_target_config": summary.get("va_target_config"),
        "lambda_target_config": summary.get("lambda_target_config"),
        "max_lambda_exec": summary.get("max_lambda_exec"),
        "lambda_target_reached": target_reached,
        "target_sync_samples": samples,
        "target_mc_weight_rmse": summary.get(
            "target_mc_pitch_weight_lambda_sync_rmse"
        ),
        "target_fw_weight_rmse": summary.get(
            "target_fw_pitch_weight_lambda_sync_rmse"
        ),
        "target_sync_pass": target_sync_pass,
        "architecture_verified": architecture_verified,
        "verification_status": status,
        "flight_measurement_pass": bool(summary.get("f1_measurement_pass")),
        "measurement_complete": bool(
            summary.get("va_hold_measurement_complete")
        ),
        "physical_failure_cause": summary.get("primary_failure_cause"),
    }


def summarize(roots: list[Path]) -> dict[str, Any]:
    attempts: list[dict[str, Any]] = []
    seen: set[Path] = set()
    for root in roots:
        for telemetry_path in sorted(root.resolve().rglob("telemetry.csv")):
            telemetry_path = telemetry_path.resolve()
            if telemetry_path in seen:
                continue
            seen.add(telemetry_path)
            summary = evaluate(telemetry_path)
            summary_path = telemetry_path.parent / "summary.json"
            summary_path.write_text(
                json.dumps(_json_safe(summary), indent=2, allow_nan=False)
                + "\n",
                encoding="utf-8",
            )
            attempts.append(_attempt_row(telemetry_path.parent, summary))

    points = []
    for slug, va_target, lambda_target in EXPECTED_POINTS:
        candidates = [
            row for row in attempts
            if _matches(row["va_target_config"], va_target)
            and _matches(row["lambda_target_config"], lambda_target)
        ]
        # Prefer actual target verification and then the latest numbered
        # retry.  This lets the explicit closure run supersede a pre-target
        # transient without deleting or rewriting any engineering attempt.
        candidates.sort(
            key=lambda row: (
                bool(row["architecture_verified"]),
                _retry_index(row),
                int(row["target_sync_samples"]),
                bool(row["flight_measurement_pass"]),
                row["run_dir"],
            ),
            reverse=True,
        )
        selected = candidates[0] if candidates else None
        points.append({
            "point": slug,
            "va_target_mps": va_target,
            "lambda_target": lambda_target,
            "architecture_verified": bool(
                selected and selected["architecture_verified"]
            ),
            "selected_attempt": selected,
            "attempts": candidates,
        })

    completed = [point for point in points if point["architecture_verified"]]
    exercised_attempts = [
        row for row in attempts if row["lambda_target_reached"]
    ]
    synchronized_exercised_attempts = [
        row for row in exercised_attempts if row["architecture_verified"]
    ]
    report = {
        "experiment": "transition_allocation_px4_synchronization",
        "definition": {
            "lambda": "continuous_transition_allocation_factor",
            "lift_collective_weight": "1-lambda_exec",
            "fw_attitude_weight": (
                "clip((lambda_exec-0.1)/(0.9-0.1),0,1)"
            ),
            "mc_attitude_weight": "1-fw_attitude_weight",
            "pusher": "independent_airspeed_controller",
        },
        "verification_rule": {
            "schema_version": 13,
            "target_must_be_reached": True,
            "minimum_target_band_samples": MIN_TARGET_SYNC_SAMPLES,
            "maximum_weight_rmse": 0.02,
            "flight_measurement_pass_required": False,
        },
        "expected_point_count": len(EXPECTED_POINTS),
        "completed_point_count": len(completed),
        "completed_point_display": f"{len(completed)}/{len(EXPECTED_POINTS)}",
        "exercised_attempt_count": len(exercised_attempts),
        "synchronized_exercised_attempt_count": len(
            synchronized_exercised_attempts
        ),
        "exercised_sync_display": (
            f"{len(synchronized_exercised_attempts)}/"
            f"{len(exercised_attempts)}"
        ),
        "incomplete_points": [
            point["point"] for point in points
            if not point["architecture_verified"]
        ],
        "points": points,
        "synchronization_pass": len(completed) == len(EXPECTED_POINTS),
    }
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("roots", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--allow-incomplete",
        action="store_true",
        help="write the report without returning failure for an incomplete matrix",
    )
    args = parser.parse_args()
    report = summarize(args.roots)
    output = args.output or args.roots[0] / "transition_allocation_summary.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(_json_safe(report), indent=2, allow_nan=False) + "\n"
    output.write_text(payload, encoding="utf-8")
    print(payload, end="")
    if not report["synchronization_pass"] and not args.allow_incomplete:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
