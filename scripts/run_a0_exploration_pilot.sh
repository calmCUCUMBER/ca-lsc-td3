#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: $0 OUTPUT_DIR [EPISODES=10] [TIMEOUT_SECONDS=180] [SEED=20260909] [SIM_SPEED_FACTOR=4] [MAX_ATTEMPTS=15]" >&2
  exit 2
fi

OUTPUT_DIR="$1"
EPISODES="${2:-10}"
TIMEOUT_SECONDS="${3:-180}"
SEED="${4:-20260909}"
SIM_SPEED_FACTOR="${5:-4}"
MAX_ATTEMPTS="${6:-15}"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

exec "${PROJECT_ROOT}/scripts/run_a0_ros_training_smoke.sh" \
  "${OUTPUT_DIR}" \
  "${EPISODES}" \
  "${TIMEOUT_SECONDS}" \
  "${SEED}" \
  "${SIM_SPEED_FACTOR}" \
  --max-attempts "${MAX_ATTEMPTS}" \
  --learning-starts 2000 \
  --exploration-hold-s 5.0
