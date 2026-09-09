#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 5 ]]; then
    cat >&2 <<'EOF'
usage: scripts/run_f1_va_lambda_grid.sh OUTPUT_ROOT [TIMEOUT_SECONDS] [VA_LIST] [LAMBDA_LIST] [SEED]

examples:
  scripts/run_f1_va_lambda_grid.sh data/evaluation/f1_coarse_v1
  scripts/run_f1_va_lambda_grid.sh data/evaluation/f1_smoke_v1 220 "10 12 14" "0.0 0.3 0.7"

The default coarse F1 grid is:
  Va:     6 8 10 12 14 16 18 20
  lambda: 0.0 0.1 0.3 0.5 0.7 0.9

Each grid point targets 5 valid independent restarts through
scripts/run_phase0_75_va_hold_repeats.sh, with every protocol-invalid attempt
retained and a predeclared maximum of 10 attempts.  Point-level failures are
retained as F1 boundary/non-convergent/unsafe/protocol data rather than
stopping the full grid.
EOF
    exit 2
fi

output_root="$1"
timeout_seconds="${2:-220}"
va_list="${3:-6 8 10 12 14 16 18 20}"
lambda_list="${4:-0.0 0.1 0.3 0.5 0.7 0.9}"
seed="${5:-20260830}"

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "${output_root}"

# Preserve the exact low-level configuration beside every grid.  Formal
# comparability requires this snapshot to remain identical across F1,
# training and all baselines.
LOW_LEVEL_CONFIG="${project_root}/config/nominal_low_level.env" \
OUTPUT_ROOT="${output_root}" python - <<'PY'
import hashlib
import json
import os
from pathlib import Path

source = Path(os.environ['LOW_LEVEL_CONFIG']).resolve()
root = Path(os.environ['OUTPUT_ROOT']).resolve()
text = source.read_text(encoding='utf-8')
payload = {
    'source': str(source),
    'sha256': hashlib.sha256(text.encode()).hexdigest(),
    'target_airspeed_dependent_px4_parameters': False,
    'contents': [
        line for line in text.splitlines()
        if line and not line.lstrip().startswith('#')
    ],
}
path = root / 'low_level_config_snapshot.json'
if path.exists() and json.loads(path.read_text(encoding='utf-8')) != payload:
    raise SystemExit(f'refusing low-level configuration drift: {path}')
path.write_text(
    json.dumps(payload, indent=2, allow_nan=False) + '\n',
    encoding='utf-8',
)
PY

export PYTHONNOUSERSITE=1
export PYTHONPATH="${project_root}/src${PYTHONPATH:+:${PYTHONPATH}}"

plan_file="${output_root}/f1_run_order.tsv"
VA_LIST="${va_list}" LAMBDA_LIST="${lambda_list}" F1_SEED="${seed}" \
    python - <<'PY' > "${plan_file}"
import os
import random

vas = [float(item) for item in os.environ['VA_LIST'].split()]
lambdas = [float(item) for item in os.environ['LAMBDA_LIST'].split()]
rng = random.Random(int(os.environ['F1_SEED']))
rows = [(va, lam) for va in vas for lam in lambdas]
rng.shuffle(rows)

def slug(value: float) -> str:
    text = f'{value:.3f}'.rstrip('0').rstrip('.')
    return text.replace('-', 'm').replace('.', 'p')

print('index\tva_target_mps\tlambda_target\tpoint_dir')
for index, (va, lam) in enumerate(rows, start=1):
    print(f'{index}\t{va:g}\t{lam:g}\tva_{slug(va)}_lambda_{slug(lam)}')
PY

echo "F1 randomized run order written to ${plan_file}"
failures=0
total_points=0
status_file="${output_root}/f1_point_status.tsv"
: > "${status_file}"

while IFS=$'\t' read -r index va_target lambda_target point_dir; do
    total_points=$((total_points + 1))
    run_root="${output_root}/${point_dir}"
    if [[ -e "${run_root}/repeat_summary.json" ]]; then
        point_status="BOUNDARY_OR_FAIL"
        if REPEAT_SUMMARY="${run_root}/repeat_summary.json" python - <<'PY'
import json
import os

summary = json.load(open(os.environ['REPEAT_SUMMARY'], encoding='utf-8'))
target = summary.get('valid_target', 5)
raise SystemExit(0 if (
    summary.get('valid_runs', summary.get('total')) == target
    and summary.get('valid_passes', summary.get('passes')) == target
) else 1)
PY
        then
            point_status="PASS"
        else
            failures=$((failures + 1))
        fi
        echo "resume: keeping completed point ${run_root}"
        echo -e "${point_dir}\t${point_status}" | tee -a "${status_file}"
        continue
    fi
    if find "${run_root}" -mindepth 1 -maxdepth 2 -name telemetry.csv -print -quit 2>/dev/null | grep -q .; then
        echo "refusing to overwrite existing telemetry under ${run_root}" >&2
        exit 2
    fi

    echo "=== F1 point ${index}: Va=${va_target} m/s, lambda=${lambda_target} ==="
    if "${project_root}/scripts/run_phase0_75_va_hold_repeats.sh" \
        "${run_root}" "${va_target}" "${lambda_target}" "${timeout_seconds}" f1; then
        echo -e "${point_dir}\tPASS" | tee -a "${status_file}"
    else
        failures=$((failures + 1))
        echo -e "${point_dir}\tBOUNDARY_OR_FAIL" | tee -a "${status_file}"
    fi
done < <(tail -n +2 "${plan_file}")

python -m ca_lsc_td3.evaluation.f1_grid "${output_root}" \
    --calibration "${project_root}/data/calibration/model_based/airframe_calibration.json" \
    --output-json "${output_root}/f1_grid_summary.json" \
    --output-csv "${output_root}/f1_grid_points.csv" \
    --output-boundary-csv "${output_root}/f1_boundaries.csv" \
    --output-quality-csv "${output_root}/f1_data_quality_cases.csv" \
    | tee "${output_root}/f1_grid_summary.stdout.json"

python -m ca_lsc_td3.evaluation.f1_heatmaps \
    "${output_root}/f1_grid_summary.json" --output-dir "${output_root}"

echo "F1 grid complete.  Boundary/fail point count during execution: ${failures}"
