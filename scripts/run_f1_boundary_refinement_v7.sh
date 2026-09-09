#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 3 ]]; then
    cat >&2 <<'EOF'
usage: scripts/run_f1_boundary_refinement_v7.sh OUTPUT_ROOT [TIMEOUT_SECONDS] [SEED]

Runs the evidence-driven refinement selected from the schema-v7 coarse map:
  * lambda=1.0 at every airspeed (upper-bound closure),
  * lambda=0.2 at Va=6,
  * lambda=0.6 at Va=8 and 10,
  * lambda=0.8 at Va=12..20,
  * an independent replacement repeat at the protocol-quality point (16,0.7).

This is 17 points x 5 independent restarts = 85 runs.  It intentionally does
not spend runs filling cells already well beyond the observed unsafe boundary.
EOF
    exit 2
fi

output_root="$1"
timeout_seconds="${2:-220}"
seed="${3:-20260831}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

"${project_root}/scripts/run_f1_va_lambda_grid.sh" \
    "${output_root}/lambda_1_all" "${timeout_seconds}" \
    "6 8 10 12 14 16 18 20" "1.0" "${seed}"

"${project_root}/scripts/run_f1_va_lambda_grid.sh" \
    "${output_root}/va6_boundary" "${timeout_seconds}" \
    "6" "0.2" "$((seed + 1))"

"${project_root}/scripts/run_f1_va_lambda_grid.sh" \
    "${output_root}/va8_10_boundary" "${timeout_seconds}" \
    "8 10" "0.6" "$((seed + 2))"

"${project_root}/scripts/run_f1_va_lambda_grid.sh" \
    "${output_root}/high_boundary" "${timeout_seconds}" \
    "12 14 16 18 20" "0.8" "$((seed + 3))"

"${project_root}/scripts/run_f1_va_lambda_grid.sh" \
    "${output_root}/va16_lambda_0p7_recheck" "${timeout_seconds}" \
    "16" "0.7" "$((seed + 4))"

echo "F1 targeted boundary refinement complete under ${output_root}"
echo "When merging, pass the recheck batch last and set F1_DUPLICATE_POLICY=last."
