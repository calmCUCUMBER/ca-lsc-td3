#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
    echo "usage: $0 LAMBDA_TARGET OUTPUT_DIR [TIMEOUT_SECONDS]" >&2
    exit 2
fi

lambda_target="$1"
output_dir="$2"
timeout_seconds="${3:-180}"
if ! python -c 'import sys; value=float(sys.argv[1]); raise SystemExit(0 if 0.0 <= value <= 1.0 else 1)' "${lambda_target}"; then
    echo "LAMBDA_TARGET must be a finite number in [0, 1]" >&2
    exit 2
fi
if [[ -e "${output_dir}/telemetry.csv" ]]; then
    echo "refusing to overwrite ${output_dir}/telemetry.csv" >&2
    exit 2
fi

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "${output_dir}/px4_workdir"

set +u
source /opt/ros/humble/setup.bash
source "${project_root}/ros2_ws/install_ca/setup.bash"
set -u
export PYTHONNOUSERSITE=1

set +e
timeout --signal=INT --kill-after=15s "${timeout_seconds}s" \
    ros2 launch ca_lsc_transition nominal_transition.launch.py \
    px4_dir:="${project_root}/PX4-Autopilot" \
    px4_build_dir:="${project_root}/PX4-Autopilot/build_ca_make" \
    schedule_mode:=manual_target \
    lambda_target:="${lambda_target}" \
    vt_unload_test_enable:=1 \
    output_csv:="${output_dir}/telemetry.csv" \
    px4_workdir:="${output_dir}/px4_workdir" \
    2>&1 | tee "${output_dir}/console.log"
launch_status=${PIPESTATUS[0]}
set -e

if [[ ! -s "${output_dir}/telemetry.csv" ]]; then
    echo "telemetry file was not produced" >&2
    exit 1
fi

set +e
ros2 run ca_lsc_transition evaluate_transition \
    "${output_dir}/telemetry.csv" \
    --output "${output_dir}/summary.json" \
    --require characterization \
    | tee "${output_dir}/summary.stdout.json"
evaluation_status=${PIPESTATUS[0]}
set -e

if [[ ${launch_status} -ne 0 ]]; then
    exit "${launch_status}"
fi
if [[ ${evaluation_status} -ne 0 ]]; then
    exit "${evaluation_status}"
fi
