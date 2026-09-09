#!/usr/bin/env bash
set -euo pipefail

# DEVELOPMENT ARCHIVE ONLY.  This changes a low-level PX4 parameter and is
# excluded from formal F1, training and paper comparisons.

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "usage: $0 OUTPUT_ROOT [TIMEOUT_SECONDS]" >&2
    exit 2
fi

output_root="$1"
timeout_seconds="${2:-220}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "${output_root}"

# Identification experiment only: lambda, outer-loop gains and the airspeed
# target remain fixed while the stock airspeed handover threshold changes.
# The schema-12 recorder captures the actual PX4 pitch weights and both
# normalized virtual pitch-controller demands.  No threshold is selected or
# frozen by this script.
OUTPUT_ROOT="${output_root}" python - <<'PY'
import json
import os
from pathlib import Path

root = Path(os.environ['OUTPUT_ROOT']).expanduser().resolve()
payload = {
    'experiment': 'f1_control_authority_identification',
    'purpose': (
        'identify unloading-pitch-authority coupling; not final F1 envelope'
    ),
    'va_target_mps': 10.0,
    'lambda_target': 0.6,
    'vt_airspeed_blend_mps': 8.0,
    'vt_transition_airspeed_mps_arms': [10.0, 13.0, 15.0],
    'valid_repeats_per_arm': 5,
    'controlled_variables': [
        'VT_UNLD_ALT_P=2.0',
        'VT_UNLD_VZ_D=2.0',
        'pusher PI and throttle bounds',
        'schema-v9 dwell/window/occupancy protocol',
    ],
    'required_telemetry_schema': 12,
    'decision_rule': (
        'diagnose trend using actual MC/FW pitch weights, normalized virtual '
        'pitch demands, elevator joint-limit command fraction, altitude and q; '
        'do not choose a final threshold from pass count alone'
    ),
}
path = root / 'experiment_manifest.json'
if path.exists():
    existing = json.loads(path.read_text(encoding='utf-8'))
    if existing != payload:
        raise SystemExit(f'refusing incompatible resume: {path}')
else:
    path.write_text(
        json.dumps(payload, indent=2, allow_nan=False) + '\n',
        encoding='utf-8',
    )
PY

overall_status=0
for trans in 10 13 15; do
    arm_dir="${output_root}/trans_${trans}"
    export F1_VT_ARSP_BLEND=8.0
    export F1_VT_ARSP_TRANS="${trans}.0"
    export ALLOW_DEVELOPMENT_PX4_PARAMETER_SWEEP=1
    export F1_VALID_TARGET=5
    export F1_MAX_ATTEMPTS=10
    set +e
    "${project_root}/scripts/run_phase0_75_va_hold_repeats.sh" \
        "${arm_dir}" 10.0 0.6 "${timeout_seconds}" f1
    arm_status=$?
    set -e
    if (( arm_status != 0 )); then
        overall_status=1
    fi
done

OUTPUT_ROOT="${output_root}" python - <<'PY'
import json
import math
import os
from pathlib import Path

root = Path(os.environ['OUTPUT_ROOT']).expanduser().resolve()
arms = []
for trans in (10, 13, 15):
    arm_root = root / f'trans_{trans}'
    repeat_path = arm_root / 'repeat_summary.json'
    if not repeat_path.exists():
        arms.append({
            'vt_transition_airspeed_mps': float(trans),
            'status': 'missing_repeat_summary',
        })
        continue
    repeat = json.loads(repeat_path.read_text(encoding='utf-8'))
    valid = [
        item for item in repeat.get('runs', [])
        if item.get('f1_attempt_quality') == 'valid'
    ]
    summaries = []
    for item in valid:
        path = arm_root / f"run_{item['run']}" / 'summary.json'
        if path.exists():
            summaries.append(json.loads(path.read_text(encoding='utf-8')))

    def mean(key):
        values = [s.get(key) for s in summaries]
        values = [float(v) for v in values if isinstance(v, (int, float)) and math.isfinite(v)]
        return sum(values) / len(values) if values else None

    arms.append({
        'vt_transition_airspeed_mps': float(trans),
        'valid_runs': len(valid),
        'passes': sum(bool(item.get('pass')) for item in valid),
        'telemetry_schema_versions': sorted({
            s.get('telemetry_schema_version') for s in summaries
        }),
        'mean_mc_pitch_weight_actual': mean(
            'va_hold_mean_mc_pitch_weight_actual'
        ),
        'mean_fw_pitch_weight_actual': mean(
            'va_hold_mean_fw_pitch_weight_actual'
        ),
        'mean_mc_pitch_weight_proxy_rmse': mean(
            'va_hold_mc_pitch_weight_proxy_rmse'
        ),
        'mean_fw_pitch_demand_normalized': mean(
            'va_hold_mean_fw_pitch_torque_demand_normalized'
        ),
        'mean_mc_pitch_demand_normalized': mean(
            'va_hold_mean_mc_pitch_torque_demand_normalized'
        ),
        'mean_elevator_joint_limit_command_fraction': mean(
            'va_hold_elevator_joint_limit_command_fraction'
        ),
        'mean_altitude_rmse_m': mean('va_hold_altitude_rmse_m'),
        'mean_pitch_rate_rmse_rad_s': mean('va_hold_pitch_rate_rmse_rad_s'),
    })

payload = {
    'experiment': 'f1_control_authority_identification',
    'arms': arms,
    'interpretation_guard': (
        'VehicleTorqueSetpoint values are normalized; they are not N m and '
        'cannot yet be substituted into paper eta_C.'
    ),
}
(root / 'authority_sweep_summary.json').write_text(
    json.dumps(payload, indent=2, allow_nan=False) + '\n',
    encoding='utf-8',
)
PY

exit "${overall_status}"
