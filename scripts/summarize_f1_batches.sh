#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 ]]; then
    echo "usage: $0 OUTPUT_ROOT INPUT_BATCH_1 INPUT_BATCH_2 [INPUT_BATCH ...]" >&2
    exit 2
fi

output_root="$1"
shift
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "${output_root}"
export PYTHONNOUSERSITE=1
export PYTHONPATH="${project_root}/src${PYTHONPATH:+:${PYTHONPATH}}"
duplicate_policy="${F1_DUPLICATE_POLICY:-error}"

python -m ca_lsc_td3.evaluation.f1_grid "$@" \
    --duplicate-policy "${duplicate_policy}" \
    --calibration "${project_root}/data/calibration/model_based/airframe_calibration.json" \
    --output-json "${output_root}/f1_grid_summary.json" \
    --output-csv "${output_root}/f1_grid_points.csv" \
    --output-boundary-csv "${output_root}/f1_boundaries.csv"

python -m ca_lsc_td3.evaluation.f1_heatmaps \
    "${output_root}/f1_grid_summary.json" --output-dir "${output_root}"
