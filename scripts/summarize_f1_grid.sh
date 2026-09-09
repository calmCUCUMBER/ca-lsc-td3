#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "usage: $0 F1_OUTPUT_ROOT" >&2
    exit 2
fi

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1
export PYTHONPATH="${project_root}/src${PYTHONPATH:+:${PYTHONPATH}}"

python -m ca_lsc_td3.evaluation.f1_grid "$1" \
    --calibration "${project_root}/data/calibration/model_based/airframe_calibration.json" \
    --output-json "$1/f1_grid_summary.json" \
    --output-csv "$1/f1_grid_points.csv" \
    --output-boundary-csv "$1/f1_boundaries.csv"

python -m ca_lsc_td3.evaluation.f1_heatmaps \
    "$1/f1_grid_summary.json" --output-dir "$1"
