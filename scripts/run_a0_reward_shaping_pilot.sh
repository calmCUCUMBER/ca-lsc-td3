#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: $0 OUTPUT_DIR [EPISODES=20] [TIMEOUT_SECONDS=180] [SEED=20260912] [SIM_SPEED_FACTOR=4] [MAX_ATTEMPTS=30]" >&2
  exit 2
fi

OUTPUT_DIR="$1"
EPISODES="${2:-20}"
TIMEOUT_SECONDS="${3:-180}"
SEED="${4:-20260912}"
SIM_SPEED_FACTOR="${5:-4}"
MAX_ATTEMPTS="${6:-30}"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set +e
"${PROJECT_ROOT}/scripts/run_a0_ros_training_smoke.sh" \
  "${OUTPUT_DIR}" \
  "${EPISODES}" \
  "${TIMEOUT_SECONDS}" \
  "${SEED}" \
  "${SIM_SPEED_FACTOR}" \
  --max-attempts "${MAX_ATTEMPTS}" \
  --learning-starts 2000 \
  --exploration-hold-s 5.0
TRAIN_STATUS=$?
set -e

if [[ "${TRAIN_STATUS}" -eq 0 ]]; then
  exit 0
fi

# The generic integration-smoke flag is intentionally strict: even a cleanly
# excluded pre-RL retry makes it return one.  For this learning pilot, accept
# only that single known qualification flag; never hide learner, timing, or
# episode-count failures.
SUMMARY_PATH="${OUTPUT_DIR}/a0_ros_training_smoke_summary.json"
if [[ -f "${SUMMARY_PATH}" ]] && python - "${SUMMARY_PATH}" "${EPISODES}" <<'PY'
import json
import sys

summary = json.load(open(sys.argv[1], encoding="utf-8"))
failures = set(summary.get("pass_failures", []))
accepted = failures <= {"infrastructure_invalid_attempt_observed"}
accepted = accepted and int(summary.get("episodes", -1)) == int(sys.argv[2])
accepted = accepted and summary.get("learner_exception") is None
accepted = accepted and (
    summary.get("reward_version")
    == "capability_free_transition_potential_safety_v2"
)
raise SystemExit(0 if accepted else 1)
PY
then
  echo "Reward pilot completed; pre-RL retries were excluded from replay." >&2
  exit 0
fi

exit "${TRAIN_STATUS}"
