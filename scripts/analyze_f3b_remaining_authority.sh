#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    cat >&2 <<'EOF'
usage: scripts/analyze_f3b_remaining_authority.sh ROOT [OUTPUT_DIR]

Validates F3-B1 directional remaining elevator authority:
  M_ava_est = qbar * |K_delta_config| * directional_remaining_angle
against the local GT elevator moment model fitted from instrumented telemetry.
EOF
    exit 2
fi

root="$1"
output_dir="${2:-${root}/analysis_remaining_authority}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export PYTHONNOUSERSITE="${PYTHONNOUSERSITE:-1}"
export PYTHONPATH="${project_root}/src${PYTHONPATH:+:${PYTHONPATH}}"

python -m ca_lsc_td3.evaluation.f3b_remaining_authority \
    "${root}" \
    --output-dir "${output_dir}" \
    --minimum-valid-cells 4 \
    --required-va 6 10 14 18 \
    | tee "${output_dir}.stdout.json"

python - "${output_dir}/f3b1_remaining_authority_validation.json" <<'PY'
import json
import sys

item = json.load(open(sys.argv[1], encoding='utf-8'))
print(
    'F3-B1 remaining authority validation: '
    f"pass={item['f3b1_remaining_authority_validation_pass']}, "
    f"cells={item['authority_pass_cell_count']}/{item['valid_cell_count']}, "
    f"max_norm_rmse={item['moment_available_normalized_rmse_max']}, "
    f"max_median_rel={item['moment_available_median_relative_error_max']}, "
    f"max_p95_rel={item['moment_available_p95_relative_error_max']}"
)
if not item['f3b1_remaining_authority_validation_pass']:
    raise SystemExit('F3-B1 remaining authority validation failed')
PY
