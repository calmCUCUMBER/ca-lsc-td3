#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    cat >&2 <<'EOF'
usage: scripts/run_f2a_payload_formal.sh OUTPUT_ROOT [TIMEOUT_SECONDS]

Runs the frozen formal F2-A partial Va-lambda grid at mass scales
1.0, 1.1, and 1.2. The paper grid contains 18 cells per mass and requests five
data-quality-valid independent restarts per cell (270 valid runs total).

The output root must be new or an intentional resume of the same protocol.
EOF
    exit 2
fi

output_root="$1"
timeout_seconds="${2:-220}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

formal_points="6:0.3 6:0.4 6:0.5 6:0.6 \
8:0.5 8:0.6 8:0.7 8:0.8 \
10:0.7 10:0.8 10:0.9 \
12:0.6 12:0.8 12:1.0 \
14:0.7 14:0.8 14:0.9 14:1.0"

mkdir -p "${output_root}"
"${project_root}/scripts/run_f2a_payload_smoke.sh" \
    "${output_root}" \
    "${timeout_seconds}" \
    "${formal_points}" \
    "1.0 1.1 1.2"

export PYTHONNOUSERSITE=1
export PYTHONPATH="${project_root}/src${PYTHONPATH:+:${PYTHONPATH}}"
mkdir -p "${output_root}/combined"
python -m ca_lsc_td3.evaluation.f2_payload "${output_root}" \
    --output-dir "${output_root}/combined" \
    | tee "${output_root}/combined/f2a_payload_summary.stdout.json"

echo "Formal F2-A payload partial scan and combined analysis complete."
