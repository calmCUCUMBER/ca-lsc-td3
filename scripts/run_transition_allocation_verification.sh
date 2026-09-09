#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "usage: $0 OUTPUT_ROOT [TIMEOUT_SECONDS]" >&2
    exit 2
fi

output_root="$1"
timeout_seconds="${2:-220}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "${output_root}"

# Architecture-freeze verification only: the same executed lambda must drive
# both lift collective unloading and PX4 MC/FW attitude allocation.  These
# nine points are not formal F1 data and a physically infeasible point does
# not invalidate the command/weight synchronization evidence.
points=(
    "va8_lam0p2 8.0 0.2"
    "va8_lam0p4 8.0 0.4"
    "va10_lam0p3 10.0 0.3"
    "va10_lam0p5 10.0 0.5"
    "va12_lam0p4 12.0 0.4"
    "va12_lam0p6 12.0 0.6"
    "va14_lam0p5 14.0 0.5"
    "va14_lam0p7 14.0 0.7"
    "va18_lam0p8 18.0 0.8"
)

for entry in "${points[@]}"; do
    read -r slug va_target lambda_target <<<"${entry}"
    run_dir="${output_root}/${slug}"
    if [[ -e "${run_dir}/telemetry.csv" ]]; then
        echo "resume: keeping ${run_dir}"
        continue
    fi
    set +e
    "${project_root}/scripts/run_va_hold_characterization.sh" \
        "${va_target}" "${lambda_target}" "${run_dir}" \
        "${timeout_seconds}" f1
    set -e
done

python "${project_root}/scripts/summarize_transition_allocation_verification.py" \
    "${output_root}" \
    --output "${output_root}/transition_allocation_summary.json"
