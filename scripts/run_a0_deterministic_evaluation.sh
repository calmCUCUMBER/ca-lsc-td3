#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 ]]; then
  echo "usage: $0 OUTPUT_DIR POLICY_CHECKPOINT NORMALIZER_CHECKPOINT [EPISODES=10] [TIMEOUT_SECONDS=180] [SEED=20260908] [MAX_ATTEMPTS=15]" >&2
  exit 2
fi

OUTPUT_DIR="$1"
POLICY_CHECKPOINT="$2"
NORMALIZER_CHECKPOINT="$3"
EPISODES="${4:-10}"
TIMEOUT_SECONDS="${5:-180}"
SEED="${6:-20260908}"
MAX_ATTEMPTS="${7:-15}"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "${PROJECT_ROOT}"
export PYTHONNOUSERSITE="${PYTHONNOUSERSITE:-1}"
export PYTHONPATH="${PROJECT_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"

set +u
source /opt/ros/humble/setup.bash
source "${PROJECT_ROOT}/ros2_ws/install_ca/setup.bash"
set -u

python -m ca_lsc_td3.rl.a0_ros_evaluation \
  "${OUTPUT_DIR}" \
  --policy-checkpoint "${POLICY_CHECKPOINT}" \
  --normalizer-checkpoint "${NORMALIZER_CHECKPOINT}" \
  --episodes "${EPISODES}" \
  --timeout-seconds "${TIMEOUT_SECONDS}" \
  --seed "${SEED}" \
  --sim-speed-factor 1 \
  --max-attempts "${MAX_ATTEMPTS}"
