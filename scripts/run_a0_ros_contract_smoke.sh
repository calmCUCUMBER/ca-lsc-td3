#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "usage: $0 OUTPUT_ROOT [TIMEOUT_SECONDS=220]" >&2
  exit 2
fi

OUTPUT_ROOT="$1"
TIMEOUT_SECONDS="${2:-220}"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "${PROJECT_ROOT}"
export PYTHONNOUSERSITE="${PYTHONNOUSERSITE:-1}"
export PYTHONPATH="${PROJECT_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"

if [[ -e "${OUTPUT_ROOT}/a0_ros_contract_summary.json" ]]; then
  echo "refusing to overwrite ${OUTPUT_ROOT}/a0_ros_contract_summary.json" >&2
  exit 2
fi
mkdir -p "${OUTPUT_ROOT}"

declare -a CASE_NAMES=(
  "lambda_0p0"
  "lambda_0p3"
  "lambda_0p6"
  "known_safe_va14_lambda_0p5"
)
declare -a CASE_VA=(
  "12.0"
  "12.0"
  "12.0"
  "14.0"
)
declare -a CASE_LAMBDA=(
  "0.0"
  "0.3"
  "0.6"
  "0.5"
)

for index in "${!CASE_NAMES[@]}"; do
  name="${CASE_NAMES[$index]}"
  va="${CASE_VA[$index]}"
  lambda="${CASE_LAMBDA[$index]}"
  run_dir="${OUTPUT_ROOT}/${name}"
  echo "A0 ROS contract scripted run: ${name} Va=${va} lambda=${lambda}"
  set +e
  CA_LSC_CONDITION_ID="a0_ros_contract_${name}" \
    "${PROJECT_ROOT}/scripts/run_va_hold_characterization.sh" \
      "${va}" \
      "${lambda}" \
      "${run_dir}" \
      "${TIMEOUT_SECONDS}" \
      "va_hold"
  run_status=$?
  set -e
  echo "${run_status}" > "${run_dir}/runner_exit_status.txt" 2>/dev/null || true
  if [[ ${run_status} -ne 0 ]]; then
    echo "scripted run failed but contract analysis will continue if telemetry exists: ${name}" >&2
  fi
done

python -m ca_lsc_td3.rl.ros_contract \
  "${OUTPUT_ROOT}" \
  --output "${OUTPUT_ROOT}/a0_ros_contract_summary.json"
analysis_status=$?

exit "${analysis_status}"
