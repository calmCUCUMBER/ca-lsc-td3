#!/usr/bin/env bash
set -euo pipefail

# DEVELOPMENT ARCHIVE ONLY; not part of the formal paper experiment chain.

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "usage: $0 OUTPUT_ROOT [TIMEOUT_SECONDS]" >&2
    exit 2
fi

output_root="$1"
timeout_seconds="${2:-220}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Independent confirmation arm.  It repeats the successful B configuration
# without tuning after inspecting the discovery batch.
export F1_VT_ARSP_BLEND=8.0
export F1_VT_ARSP_TRANS=13.0
export ALLOW_DEVELOPMENT_PX4_PARAMETER_SWEEP=1
export F1_VALID_TARGET=5
export F1_MAX_ATTEMPTS=10

mkdir -p "${output_root}"
OUTPUT_ROOT="${output_root}" python - <<'PY'
import json
import os
from pathlib import Path

root = Path(os.environ['OUTPUT_ROOT']).expanduser().resolve()
payload = {
    'experiment': 'f1_transition_control_authority_confirmation',
    'arm': 'C_independent_confirmation',
    'discovery_arm': (
        'data/evaluation/f1_gap_va10_lambda_0p6_trans13_v11_5x'
    ),
    'reference_arm': (
        'data/evaluation/f1_gap_va10_lambda_0p6_v10_10x'
    ),
    'va_target_mps': 10.0,
    'lambda_target': 0.6,
    'vt_airspeed_blend_mps': 8.0,
    'vt_transition_airspeed_mps': 13.0,
    'valid_repeat_target': 5,
    'maximum_attempts': 10,
    'require_mode': 'f1',
    'predeclared_decision': {
        'confirmation_passes_required': 5,
        'combined_passes_required': 9,
        'combined_valid_runs': 10,
        'hard_physical_aborts_allowed': 0,
    },
    'fixed_controller_settings': {
        'VT_UNLD_ALT_P': 2.0,
        'VT_UNLD_VZ_D': 2.0,
        'pusher_airspeed_ff': 0.25,
        'pusher_airspeed_kp': 0.08,
        'pusher_airspeed_ki': 0.03,
        'pusher_throttle_max': 0.45,
        'transition_timeout_s': 20.0,
    },
}
manifest = root / 'experiment_manifest.json'
if manifest.exists():
    existing = json.loads(manifest.read_text(encoding='utf-8'))
    if existing != payload:
        raise SystemExit(f'refusing incompatible resume: {manifest}')
else:
    manifest.write_text(
        json.dumps(payload, indent=2, allow_nan=False) + '\n',
        encoding='utf-8',
    )
PY

set +e
"${project_root}/scripts/run_phase0_75_va_hold_repeats.sh" \
    "${output_root}" 10.0 0.6 "${timeout_seconds}" f1
repeat_status=$?
set -e

if [[ -e "${output_root}/repeat_summary.json" ]]; then
    PYTHONPATH="${project_root}/src${PYTHONPATH:+:${PYTHONPATH}}" \
        python -m ca_lsc_td3.evaluation.f1_transition_gap \
        "${output_root}"
fi

exit "${repeat_status}"
