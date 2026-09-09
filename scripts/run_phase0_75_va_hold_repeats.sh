#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 || $# -gt 5 ]]; then
    echo "usage: $0 OUTPUT_ROOT VA_TARGET_MPS LAMBDA_TARGET [TIMEOUT_SECONDS] [REQUIRE_MODE]" >&2
    echo "REQUIRE_MODE: va_hold (Phase-0.75 default) | f1" >&2
    exit 2
fi

output_root="$1"
va_target="$2"
lambda_target="$3"
timeout_seconds="${4:-220}"
require_mode="${5:-va_hold}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "${output_root}"

if [[ "${require_mode}" != "va_hold" && "${require_mode}" != "f1" ]]; then
    echo "REQUIRE_MODE must be va_hold or f1" >&2
    exit 2
fi

valid_target=5
max_attempts=5
if [[ "${require_mode}" == "f1" ]]; then
    valid_target="${F1_VALID_TARGET:-5}"
    max_attempts="${F1_MAX_ATTEMPTS:-10}"
fi
if ! [[ "${valid_target}" =~ ^[0-9]+$ ]] \
    || (( valid_target < 1 )); then
    echo "F1_VALID_TARGET must be a positive integer" >&2
    exit 2
fi
if ! [[ "${max_attempts}" =~ ^[0-9]+$ ]] \
    || (( max_attempts < valid_target )); then
    echo "F1_MAX_ATTEMPTS must be an integer >= ${valid_target}" >&2
    exit 2
fi

for run in $(seq 1 "${max_attempts}"); do
    run_dir="${output_root}/run_${run}"
    if [[ -e "${run_dir}/telemetry.csv" && ! -e "${run_dir}/summary.json" ]]; then
        echo "refusing to overwrite incomplete ${run_dir}/telemetry.csv" >&2
        exit 2
    fi
done

status_file="${output_root}/repeat_status.tsv"
: > "${status_file}"
failures=0
valid_runs=0
valid_passes=0
protocol_invalid_runs=0
attempts=0

for run in $(seq 1 "${max_attempts}"); do
    if [[ "${require_mode}" == "f1" ]] && (( valid_runs >= valid_target )); then
        break
    fi
    run_dir="${output_root}/run_${run}"
    attempts=$((attempts + 1))
    echo "=== Phase-0.75 Va-hold attempt ${run}/${max_attempts}: ${run_dir} ==="
    if [[ -e "${run_dir}/summary.json" ]]; then
        echo "resume: keeping completed ${run_dir}"
    else
        set +e
        "${project_root}/scripts/run_va_hold_characterization.sh" \
            "${va_target}" "${lambda_target}" "${run_dir}" \
            "${timeout_seconds}" "${require_mode}"
        set -e
    fi

    if [[ ! -e "${run_dir}/summary.json" ]]; then
        run_status="MISSING_SUMMARY"
        protocol_invalid_runs=$((protocol_invalid_runs + 1))
        RUN_DIR="${run_dir}" VA_TARGET="${va_target}" \
        LAMBDA_TARGET="${lambda_target}" python - <<'PY'
import json
import os
from pathlib import Path

run_dir = Path(os.environ['RUN_DIR'])
run_dir.mkdir(parents=True, exist_ok=True)
placeholder = {
    'source_csv': str((run_dir / 'telemetry.csv').resolve()),
    'telemetry_schema_version': None,
    'va_target_config': float(os.environ['VA_TARGET']),
    'lambda_target_config': float(os.environ['LAMBDA_TARGET']),
    'f1_measurement_pass': False,
    'phase_0_75_va_hold_pass': False,
    'va_hold_measurement_complete': False,
    'primary_failure_cause': 'missing_summary',
    'secondary_failure_cause': 'attempt_process_failure',
    'va_hold_abort_reason': 'none',
    'f1_attempt_quality': 'protocol_invalid',
    'f1_attempt_quality_reason': 'missing_summary',
}
(run_dir / 'summary.json').write_text(
    json.dumps(placeholder, indent=2, allow_nan=False) + '\n',
    encoding='utf-8',
)
PY
        if [[ "${require_mode}" != "f1" ]]; then
            failures=$((failures + 1))
        fi
    else
        IFS=$'\t' read -r attempt_quality attempt_pass quality_reason < <(
            RUN_SUMMARY="${run_dir}/summary.json" \
            REQUIRED_KEY="$({
                [[ "${require_mode}" == "f1" ]] \
                    && echo f1_measurement_pass \
                    || echo phase_0_75_va_hold_pass
            })" REQUIRE_MODE="${require_mode}" python - <<'PY'
import json
import os

summary = json.load(open(os.environ['RUN_SUMMARY'], encoding='utf-8'))
quality = (
    summary.get('f1_attempt_quality', 'valid')
    if os.environ['REQUIRE_MODE'] == 'f1' else 'valid'
)
reason = summary.get('f1_attempt_quality_reason', 'none')
passed = bool(summary.get(os.environ['REQUIRED_KEY']))
print(f"{quality}\t{int(passed)}\t{reason}")
PY
        )
        if [[ "${attempt_quality}" == "valid" ]]; then
            valid_runs=$((valid_runs + 1))
            if [[ "${attempt_pass}" == "1" ]]; then
                valid_passes=$((valid_passes + 1))
                run_status="VALID_PASS"
            else
                failures=$((failures + 1))
                run_status="VALID_PHYSICAL_FAIL"
            fi
        else
            protocol_invalid_runs=$((protocol_invalid_runs + 1))
            run_status="PROTOCOL_INVALID:${quality_reason}"
        fi
    fi
    echo -e "run_${run}\t${run_status}" | tee -a "${status_file}"
done

REPEAT_OUTPUT_ROOT="${output_root}" REPEAT_REQUIRE_MODE="${require_mode}" \
REPEAT_MAX_ATTEMPTS="${max_attempts}" REPEAT_VALID_TARGET="${valid_target}" \
python - <<'PY'
import json
import math
import os
from pathlib import Path

root = Path(os.environ['REPEAT_OUTPUT_ROOT'])
require_mode = os.environ['REPEAT_REQUIRE_MODE']
required_key = {
    'va_hold': 'phase_0_75_va_hold_pass',
    'f1': 'f1_measurement_pass',
}[require_mode]
runs = []
attempt_dirs = sorted(
    (path for path in root.glob('run_*') if path.is_dir()),
    key=lambda path: int(path.name.split('_', 1)[1]),
)
for attempt_dir in attempt_dirs:
    index = int(attempt_dir.name.split('_', 1)[1])
    path = attempt_dir / 'summary.json'
    item = {'run': index, 'summary_present': path.exists()}
    if path.exists():
        summary = json.loads(path.read_text(encoding='utf-8'))
        item.update({
            'pass': bool(summary.get(required_key)),
            'f1_attempt_quality': summary.get(
                'f1_attempt_quality', 'valid'
            ),
            'f1_attempt_quality_reason': summary.get(
                'f1_attempt_quality_reason', 'none'
            ),
            'primary_failure_cause': summary.get('primary_failure_cause'),
            'secondary_failure_cause': summary.get('secondary_failure_cause'),
            'va_target_config': summary.get('va_target_config'),
            'lambda_target_config': summary.get('lambda_target_config'),
            'condition_id': summary.get('condition_id'),
            'wind_model_config': summary.get('wind_model_config'),
            'airspeed_source_config': summary.get('airspeed_source_config'),
            'mass_scale_config': summary.get('mass_scale_config'),
            'nominal_model_mass_kg_config': summary.get(
                'nominal_model_mass_kg_config'
            ),
            'actual_model_mass_kg_config': summary.get(
                'actual_model_mass_kg_config'
            ),
            'payload_mass_kg_config': summary.get('payload_mass_kg_config'),
            'vt_unload_altitude_pitch_kp_config': summary.get(
                'vt_unload_altitude_pitch_kp_config'
            ),
            'vt_unload_vertical_speed_pitch_kd_config': summary.get(
                'vt_unload_vertical_speed_pitch_kd_config'
            ),
            'vt_unload_pitch_min_deg_config': summary.get(
                'vt_unload_pitch_min_deg_config'
            ),
            'vt_unload_pitch_max_deg_config': summary.get(
                'vt_unload_pitch_max_deg_config'
            ),
            'vt_airspeed_blend_mps_config': summary.get(
                'vt_airspeed_blend_mps_config'
            ),
            'vt_transition_airspeed_mps_config': summary.get(
                'vt_transition_airspeed_mps_config'
            ),
            'va_hold_measurement_complete': summary.get(
                'va_hold_measurement_complete'
            ),
            'va_hold_measurement_protocol_valid': summary.get(
                'va_hold_measurement_protocol_valid'
            ),
            'va_hold_measurement_interrupted': summary.get(
                'va_hold_measurement_interrupted'
            ),
            'va_hold_abort_reason': summary.get('va_hold_abort_reason'),
            'va_hold_pre_measurement_altitude_transient': summary.get(
                'va_hold_pre_measurement_altitude_transient'
            ),
            'va_low_speed_vertical_transient_samples': summary.get(
                'va_low_speed_vertical_transient_samples'
            ),
            'va_vertical_abort_enabled_fraction': summary.get(
                'va_vertical_abort_enabled_fraction'
            ),
            'phase_0_75_pusher_airspeed_regulation_pass': summary.get(
                'phase_0_75_pusher_airspeed_regulation_pass'
            ),
            'phase_0_75_pusher_airspeed_regulation_failure': summary.get(
                'phase_0_75_pusher_airspeed_regulation_failure'
            ),
            'va_hold_measurement_duration_s': summary.get(
                'va_hold_measurement_duration_s'
            ),
            'va_hold_measurement_segment_count': summary.get(
                'va_hold_measurement_segment_count'
            ),
            'va_hold_mean_airspeed_mps': summary.get(
                'va_hold_mean_airspeed_mps'
            ),
            'va_hold_max_abs_airspeed_error_mps': summary.get(
                'va_hold_max_abs_airspeed_error_mps'
            ),
            'va_hold_mean_lambda_exec': summary.get(
                'va_hold_mean_lambda_exec'
            ),
            'va_hold_max_abs_lambda_error': summary.get(
                'va_hold_max_abs_lambda_error'
            ),
            'va_hold_transition_pusher_external_active_fraction': summary.get(
                'va_hold_transition_pusher_external_active_fraction'
            ),
            'va_hold_measurement_pusher_external_active_fraction': summary.get(
                'va_hold_measurement_pusher_external_active_fraction'
            ),
            'va_hold_mean_pusher_throttle_command': summary.get(
                'va_hold_mean_pusher_throttle_command'
            ),
            'va_hold_mean_pusher_throttle_status': summary.get(
                'va_hold_mean_pusher_throttle_status'
            ),
            'va_hold_mean_airspeed_filtered_mps': summary.get(
                'va_hold_mean_airspeed_filtered_mps'
            ),
            'va_hold_mean_pusher_airspeed_error_filtered_mps': summary.get(
                'va_hold_mean_pusher_airspeed_error_filtered_mps'
            ),
            'va_hold_max_abs_pusher_airspeed_error_filtered_mps': summary.get(
                'va_hold_max_abs_pusher_airspeed_error_filtered_mps'
            ),
            'va_hold_mean_pusher_pi_integral_mps_s': summary.get(
                'va_hold_mean_pusher_pi_integral_mps_s'
            ),
            'va_hold_max_abs_pusher_pi_integral_mps_s': summary.get(
                'va_hold_max_abs_pusher_pi_integral_mps_s'
            ),
            'va_hold_mean_pusher_throttle_unsaturated': summary.get(
                'va_hold_mean_pusher_throttle_unsaturated'
            ),
            'va_hold_mean_down_velocity_command_mps': summary.get(
                'va_hold_mean_down_velocity_command_mps'
            ),
            'va_hold_max_abs_down_velocity_command_mps': summary.get(
                'va_hold_max_abs_down_velocity_command_mps'
            ),
            'va_hold_mean_lift_thrust_n': summary.get(
                'va_hold_mean_lift_thrust_n'
            ),
            'va_hold_mean_lift_power_w': summary.get(
                'va_hold_mean_lift_power_w'
            ),
            'va_hold_mean_pusher_power_w': summary.get(
                'va_hold_mean_pusher_power_w'
            ),
            'va_hold_mean_total_power_w': summary.get(
                'va_hold_mean_total_power_w'
            ),
            'va_hold_lift_energy_proxy_j': summary.get(
                'va_hold_lift_energy_proxy_j'
            ),
            'va_hold_pusher_energy_proxy_j': summary.get(
                'va_hold_pusher_energy_proxy_j'
            ),
            'va_hold_total_energy_proxy_j': summary.get(
                'va_hold_total_energy_proxy_j'
            ),
            'va_hold_altitude_rmse_m': summary.get(
                'va_hold_altitude_rmse_m'
            ),
            'va_hold_max_abs_altitude_error_m': summary.get(
                'va_hold_max_abs_altitude_error_m'
            ),
            'va_hold_max_abs_vertical_speed_mps': summary.get(
                'va_hold_max_abs_vertical_speed_mps'
            ),
            'va_hold_mean_alpha_rad': summary.get('va_hold_mean_alpha_rad'),
            'va_hold_max_abs_alpha_rad': summary.get(
                'va_hold_max_abs_alpha_rad'
            ),
            'va_hold_pitch_rate_rmse_rad_s': summary.get(
                'va_hold_pitch_rate_rmse_rad_s'
            ),
            'va_hold_pitch_rate_p95_rad_s': summary.get(
                'va_hold_pitch_rate_p95_rad_s'
            ),
            'va_hold_mean_pitch_setpoint_rad': summary.get(
                'va_hold_mean_pitch_setpoint_rad'
            ),
            'va_hold_pitch_tracking_rmse_rad': summary.get(
                'va_hold_pitch_tracking_rmse_rad'
            ),
            'va_hold_mean_mc_pitch_weight_proxy': summary.get(
                'va_hold_mean_mc_pitch_weight_proxy'
            ),
            'va_hold_min_mc_pitch_weight_proxy': summary.get(
                'va_hold_min_mc_pitch_weight_proxy'
            ),
            'va_hold_max_mc_pitch_weight_proxy': summary.get(
                'va_hold_max_mc_pitch_weight_proxy'
            ),
            'va_hold_lift_rotor_spike_samples': summary.get(
                'va_hold_lift_rotor_spike_samples'
            ),
            'va_hold_lift_rotor_saturation_fraction': summary.get(
                'va_hold_lift_rotor_saturation_fraction'
            ),
            'va_hold_lift_collective_saturation_fraction': summary.get(
                'va_hold_lift_collective_saturation_fraction'
            ),
            'va_hold_servo_saturation_fraction': summary.get(
                'va_hold_servo_saturation_fraction'
            ),
            'va_hold_pusher_throttle_upper_saturation_fraction': summary.get(
                'va_hold_pusher_throttle_upper_saturation_fraction'
            ),
            'va_hold_pusher_rotor_saturation_fraction': summary.get(
                'va_hold_pusher_rotor_saturation_fraction'
            ),
        })
    else:
        item.update({
            'pass': False,
            'primary_failure_cause': 'missing_summary',
        })
    runs.append(item)

def finite_values(key):
    values = []
    physical_runs = (
        [item for item in runs
         if item.get('f1_attempt_quality') == 'valid']
        if require_mode == 'f1' else runs
    )
    for item in physical_runs:
        value = item.get(key)
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            if math.isfinite(value):
                values.append(float(value))
    return values

stats = {}
for key in (
    'va_hold_measurement_duration_s',
    'va_hold_measurement_segment_count',
    'va_hold_measurement_protocol_valid',
    'va_hold_measurement_interrupted',
    'mass_scale_config',
    'nominal_model_mass_kg_config',
    'actual_model_mass_kg_config',
    'payload_mass_kg_config',
    'va_hold_mean_airspeed_mps',
    'va_hold_max_abs_airspeed_error_mps',
    'va_hold_mean_lambda_exec',
    'va_hold_max_abs_lambda_error',
    'va_low_speed_vertical_transient_samples',
    'va_vertical_abort_enabled_fraction',
    'va_hold_transition_pusher_external_active_fraction',
    'va_hold_measurement_pusher_external_active_fraction',
    'va_hold_mean_pusher_throttle_command',
    'va_hold_mean_pusher_throttle_status',
    'va_hold_mean_airspeed_filtered_mps',
    'va_hold_mean_pusher_airspeed_error_filtered_mps',
    'va_hold_max_abs_pusher_airspeed_error_filtered_mps',
    'va_hold_mean_pusher_pi_integral_mps_s',
    'va_hold_max_abs_pusher_pi_integral_mps_s',
    'va_hold_mean_pusher_throttle_unsaturated',
    'va_hold_mean_down_velocity_command_mps',
    'va_hold_max_abs_down_velocity_command_mps',
    'va_hold_mean_lift_thrust_n',
    'va_hold_mean_lift_power_w',
    'va_hold_mean_pusher_power_w',
    'va_hold_mean_total_power_w',
    'va_hold_lift_energy_proxy_j',
    'va_hold_pusher_energy_proxy_j',
    'va_hold_total_energy_proxy_j',
    'va_hold_altitude_rmse_m',
    'va_hold_max_abs_altitude_error_m',
    'va_hold_max_abs_vertical_speed_mps',
    'va_hold_mean_alpha_rad',
    'va_hold_max_abs_alpha_rad',
    'va_hold_pitch_rate_rmse_rad_s',
    'va_hold_pitch_rate_p95_rad_s',
    'va_hold_mean_pitch_setpoint_rad',
    'va_hold_pitch_tracking_rmse_rad',
    'va_hold_mean_mc_pitch_weight_proxy',
    'va_hold_min_mc_pitch_weight_proxy',
    'va_hold_max_mc_pitch_weight_proxy',
    'va_hold_lift_rotor_spike_samples',
    'va_hold_lift_rotor_saturation_fraction',
    'va_hold_lift_collective_saturation_fraction',
    'va_hold_servo_saturation_fraction',
    'va_hold_pusher_throttle_upper_saturation_fraction',
    'va_hold_pusher_rotor_saturation_fraction',
):
    values = finite_values(key)
    if not values:
        continue
    mean = sum(values) / len(values)
    variance = sum((value - mean) ** 2 for value in values) / len(values)
    stats[key + '_mean'] = mean
    stats[key + '_std'] = math.sqrt(variance)
    stats[key + '_min'] = min(values)
    stats[key + '_max'] = max(values)

valid_runs = [
    item for item in runs
    if item.get('summary_present')
    and (
        require_mode != 'f1'
        or item.get('f1_attempt_quality') == 'valid'
    )
]
protocol_invalid_runs = [
    item for item in runs
    if require_mode == 'f1'
    and item.get('f1_attempt_quality') != 'valid'
]
valid_target = int(os.environ['REPEAT_VALID_TARGET'])
max_attempts = int(os.environ['REPEAT_MAX_ATTEMPTS'])
aggregate = {
    'mode': 'f1_va_lambda' if require_mode == 'f1' else 'phase0_75_va_hold',
    'required_key': required_key,
    'passes': sum(item['pass'] for item in valid_runs),
    'total': len(runs),
    'attempt_count': len(runs),
    'valid_runs': len(valid_runs),
    'valid_passes': sum(item['pass'] for item in valid_runs),
    'protocol_invalid_runs': len(protocol_invalid_runs),
    'valid_target': valid_target,
    'maximum_attempts': max_attempts if require_mode == 'f1' else 5,
    'attempts_exhausted': bool(
        require_mode == 'f1'
        and len(valid_runs) < valid_target
        and len(runs) >= max_attempts
    ),
    'runs': runs,
    'stats': stats,
}
(root / 'repeat_summary.json').write_text(
    json.dumps(aggregate, indent=2, allow_nan=False) + '\n',
    encoding='utf-8',
)
print(json.dumps(aggregate, indent=2, allow_nan=False))
PY

if [[ "${require_mode}" == "f1" ]] && (( valid_runs < valid_target )); then
    echo "protocol unstable: obtained ${valid_runs}/${valid_target} valid runs " \
        "in ${attempts}/${max_attempts} attempts" >&2
    exit 1
fi
if [[ ${failures} -ne 0 ]]; then
    exit 1
fi
