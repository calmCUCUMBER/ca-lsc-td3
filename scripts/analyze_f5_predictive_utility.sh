#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    cat >&2 <<'EOF'
usage: scripts/analyze_f5_predictive_utility.sh DATASET_CSV [OUTPUT_DIR]

Runs the leakage-safe grouped logistic comparison:
  Model A: [Va, lambda]
  Model B: [Va, eta_L, eta_C, lambda]
EOF
    exit 2
fi

dataset_csv="$1"
output_dir="${2:-$(dirname "${dataset_csv}")/predictive_utility}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export PYTHONNOUSERSITE="${PYTHONNOUSERSITE:-1}"
export PYTHONPATH="${project_root}/src${PYTHONPATH:+:${PYTHONPATH}}"

python -m ca_lsc_td3.evaluation.f5_predictive_utility \
    "${dataset_csv}" \
    --output-dir "${output_dir}"
