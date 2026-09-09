#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: $0 OUTPUT_DIR [EPISODES=3] [TIMEOUT_SECONDS=180] [SEED=20260907] [SIM_SPEED_FACTOR=1.0] [EXTRA_ARGS...]" >&2
  exit 2
fi

OUTPUT_DIR="$1"
EPISODES="${2:-3}"
TIMEOUT_SECONDS="${3:-180}"
SEED="${4:-20260907}"
SIM_SPEED_FACTOR="${5:-1.0}"
shift $(( $# < 5 ? $# : 5 ))
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "${PROJECT_ROOT}"
export PYTHONNOUSERSITE="${PYTHONNOUSERSITE:-1}"
export PYTHONPATH="${PROJECT_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"

set +u
source /opt/ros/humble/setup.bash
source "${PROJECT_ROOT}/ros2_ws/install_ca/setup.bash"
set -u

python -m ca_lsc_td3.rl.a0_ros_training \
  "${OUTPUT_DIR}" \
  --episodes "${EPISODES}" \
  --timeout-seconds "${TIMEOUT_SECONDS}" \
  --seed "${SEED}" \
  --sim-speed-factor "${SIM_SPEED_FACTOR}" \
  "$@"
