#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "usage: $0 VERIFICATION_ROOT [TIMEOUT_SECONDS]" >&2
    exit 2
fi

verification_root="$1"
timeout_seconds="${2:-220}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -d "${verification_root}" ]]; then
    echo "verification root does not exist: ${verification_root}" >&2
    exit 2
fi

retry_index=1
while [[ -e "${verification_root}/va14_lam0p7_retry_${retry_index}" ]]; do
    retry_index=$((retry_index + 1))
done
retry_dir="${verification_root}/va14_lam0p7_retry_${retry_index}"

echo "Running only the missing architecture-verification point: Va=14.0, lambda=0.7"
echo "Retry evidence will be preserved in: ${retry_dir}"

# The flight evaluator is intentionally allowed to return non-zero here.  A
# physically poor F1 window does not invalidate architecture synchronization
# if lambda=0.7 was reached and the target-band MC/FW weights match.  The
# dedicated summary below makes that distinction and supplies the final exit
# status.
set +e
"${project_root}/scripts/run_va_hold_characterization.sh" \
    14.0 0.7 "${retry_dir}" "${timeout_seconds}" f1
flight_status=$?
set -e
echo "F1 flight-evaluator exit status: ${flight_status} (diagnostic only for this verification)"

python "${project_root}/scripts/summarize_transition_allocation_verification.py" \
    "${verification_root}" \
    --output "${verification_root}/transition_allocation_summary.json"
