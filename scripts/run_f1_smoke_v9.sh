#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 3 ]]; then
    echo "usage: $0 OUTPUT_ROOT [TIMEOUT_SECONDS] [SEED]" >&2
    exit 2
fi

output_root="$1"
timeout_seconds="${2:-220}"
seed="${3:-20260902}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "${output_root}"

run_slice() {
    local name="$1"
    local va="$2"
    local lambdas="$3"
    local offset="$4"
    F1_MAX_ATTEMPTS=10 "${project_root}/scripts/run_f1_va_lambda_grid.sh" \
        "${output_root}/${name}" "${timeout_seconds}" "${va}" \
        "${lambdas}" "$((seed + offset))"
}

run_slice va6  "6"  "0.1 0.3 0.8" 0
run_slice va10 "10" "0.3 0.6 0.9" 1
run_slice va14 "14" "0.3 0.6 0.9" 2
run_slice va18 "18" "0.3 0.6 1.0" 3

export PYTHONNOUSERSITE=1
export PYTHONPATH="${project_root}/src${PYTHONPATH:+:${PYTHONPATH}}"
python -m ca_lsc_td3.evaluation.f1_grid "${output_root}" \
    --calibration "${project_root}/data/calibration/model_based/airframe_calibration.json" \
    --output-json "${output_root}/f1_grid_summary.json" \
    --output-csv "${output_root}/f1_grid_points.csv" \
    --output-boundary-csv "${output_root}/f1_boundaries.csv" \
    --output-quality-csv "${output_root}/f1_data_quality_cases.csv" \
    | tee "${output_root}/f1_grid_summary.stdout.json"
python -m ca_lsc_td3.evaluation.f1_heatmaps \
    "${output_root}/f1_grid_summary.json" --output-dir "${output_root}"

SUMMARY_JSON="${output_root}/f1_grid_summary.json" python - <<'PY'
import json
import os
from pathlib import Path

payload = json.loads(Path(os.environ['SUMMARY_JSON']).read_text())
errors = []
if payload.get('point_count') != 12:
    errors.append(f"expected 12 smoke cells, got {payload.get('point_count')}")
if payload.get('telemetry_schema_versions') != [13.0]:
    errors.append(
        'expected transition-allocation telemetry schema [13.0], got '
        f"{payload.get('telemetry_schema_versions')}"
    )
for point in payload.get('points', []):
    if point.get('valid_f1_total', 0) < 5:
        errors.append(
            f"Va={point.get('va_target_mps')}, lambda={point.get('lambda_target')}: "
            'fewer than five valid repeats'
        )
if errors:
    raise SystemExit('\n'.join(errors))
print(
    'Transition-allocation smoke / telemetry-schema-13 audit passed: '
    '12 cells with five valid repeats.'
)
PY
