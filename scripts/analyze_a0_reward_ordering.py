#!/usr/bin/env python3
"""CLI wrapper for the archived A0 reward-ordering audit."""

from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from ca_lsc_td3.rl.reward_audit import main


if __name__ == "__main__":
    raise SystemExit(main())
