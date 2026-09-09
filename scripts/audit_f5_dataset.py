#!/usr/bin/env python3
"""CLI wrapper for the F5 dataset audit and builder."""

import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / 'src'))
os.environ.setdefault('PYTHONNOUSERSITE', '1')

from ca_lsc_td3.evaluation.f5_dataset import main


if __name__ == '__main__':
    raise SystemExit(main())
