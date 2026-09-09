#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 5 ]]; then
  echo "usage: $0 OUTPUT_ROOT [EPISODES=2] [TIMEOUT_SECONDS=180] [SEED=20260907] [FACTORS='1 2 4']" >&2
  exit 2
fi

OUTPUT_ROOT="$1"
EPISODES="${2:-2}"
TIMEOUT_SECONDS="${3:-180}"
SEED="${4:-20260907}"
FACTORS="${5:-1 2 4}"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "${PROJECT_ROOT}"
mkdir -p "${OUTPUT_ROOT}"
export PYTHONNOUSERSITE="${PYTHONNOUSERSITE:-1}"
export PYTHONPATH="${PROJECT_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"

for FACTOR in ${FACTORS}; do
  TOKEN="${FACTOR//./p}"
  TOKEN="${TOKEN//-/m}"
  RUN_DIR="${OUTPUT_ROOT}/x${TOKEN}"
  echo "[SIM-SPEED] factor=${FACTOR} output=${RUN_DIR}"
  ./scripts/run_a0_ros_training_smoke.sh \
    "${RUN_DIR}" \
    "${EPISODES}" \
    "${TIMEOUT_SECONDS}" \
    "${SEED}" \
    "${FACTOR}" \
    --disable-learning \
    --fixed-action 0.0
done

python -m ca_lsc_td3.evaluation.a0_sim_speed_qualification "${OUTPUT_ROOT}"
