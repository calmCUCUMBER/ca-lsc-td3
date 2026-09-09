#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "usage: $0 INPUT_ROOT [OUTPUT_DIR]" >&2
    exit 2
fi

input_root="$1"
output_dir="${2:-${input_root}/analysis_eta_c_behavior}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "${project_root}"
export PYTHONNOUSERSITE=1
export PYTHONPATH="${project_root}/src${PYTHONPATH:+:${PYTHONPATH}}"

python -m ca_lsc_td3.evaluation.f3b_eta_c_behavior \
    "${input_root}" \
    --output-dir "${output_dir}" \
    | tee "${output_dir}.stdout.json"
