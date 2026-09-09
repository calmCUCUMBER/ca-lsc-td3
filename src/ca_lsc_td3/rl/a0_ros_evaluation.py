"""Run a frozen A0 TD3 policy against nominal PX4/Gazebo transitions."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from .a0_ros_training import run_training_smoke


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--policy-checkpoint", type=Path, required=True)
    parser.add_argument("--normalizer-checkpoint", type=Path, required=True)
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--timeout-seconds", type=float, default=180.0)
    parser.add_argument("--seed", type=int, default=20260908)
    parser.add_argument("--device", default=None)
    parser.add_argument("--sim-speed-factor", type=float, default=1.0)
    parser.add_argument(
        "--max-attempts",
        type=int,
        default=None,
        help="Maximum cold simulator launches used to obtain valid episodes.",
    )
    args = parser.parse_args()
    try:
        summary = run_training_smoke(
            args.output_dir,
            episodes=args.episodes,
            timeout_seconds=args.timeout_seconds,
            seed=args.seed,
            device=args.device,
            sim_speed_factor=args.sim_speed_factor,
            enable_learning=False,
            fixed_action=None,
            max_attempts=args.max_attempts,
            deterministic_evaluation=True,
            policy_checkpoint=args.policy_checkpoint,
            normalizer_checkpoint=args.normalizer_checkpoint,
        )
    except (FileExistsError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    return 0 if summary["a0_deterministic_evaluation_protocol_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
