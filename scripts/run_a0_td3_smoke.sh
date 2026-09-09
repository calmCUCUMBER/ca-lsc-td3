#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 3 ]]; then
  echo "usage: $0 OUTPUT_DIR [EPISODES=20] [SEED=20260906]" >&2
  exit 2
fi

OUTPUT_DIR="$1"
EPISODES="${2:-20}"
SEED="${3:-20260906}"

cd /home/weicheng/ca_lsc_td3
export PYTHONNOUSERSITE="${PYTHONNOUSERSITE:-1}"
export PYTHONPATH="/home/weicheng/ca_lsc_td3/src${PYTHONPATH:+:${PYTHONPATH}}"

if python - <<'PY' >/dev/null 2>&1
import torch
PY
then
  python -m ca_lsc_td3.rl.a0_smoke \
    "${OUTPUT_DIR}" \
    --episodes "${EPISODES}" \
    --seed "${SEED}"
elif command -v conda >/dev/null 2>&1; then
  conda run -n vtol_nav env \
    PYTHONPATH="${PYTHONPATH}" \
    PYTHONNOUSERSITE="${PYTHONNOUSERSITE}" \
    python -m ca_lsc_td3.rl.a0_smoke \
      "${OUTPUT_DIR}" \
      --episodes "${EPISODES}" \
      --seed "${SEED}"
else
  echo "torch is not importable; activate vtol_nav before running this script" >&2
  exit 1
fi
