#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 || $# -gt 5 ]]; then
    echo "usage: $0 VA_TARGET_MPS LAMBDA_TARGET OUTPUT_DIR [TIMEOUT_SECONDS] [REQUIRE_MODE]" >&2
    echo "REQUIRE_MODE: va_hold (Phase-0.75 default) | f1 | f2b" >&2
    exit 2
fi

va_target="$1"
lambda_target="$2"
output_dir="$3"
timeout_seconds="${4:-220}"
require_mode="${5:-va_hold}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Formal experiments use one frozen low-level controller.  In particular,
# none of these PX4 parameters may depend on the requested Va grid point.
source "${project_root}/config/nominal_low_level.env"
vt_blend_airspeed="${NOMINAL_VT_ARSP_BLEND}"
vt_transition_airspeed="${NOMINAL_VT_ARSP_TRANS}"
fw_airspeed_min="${NOMINAL_FW_AIRSPD_MIN}"
fw_airspeed_trim="${NOMINAL_FW_AIRSPD_TRIM}"
fw_airspeed_max="${NOMINAL_FW_AIRSPD_MAX}"

# Historical transition-gap scripts may explicitly opt into a development
# A/B override.  Formal F1/training runners never set this flag.
if [[ "${ALLOW_DEVELOPMENT_PX4_PARAMETER_SWEEP:-0}" == "1" ]]; then
    vt_blend_airspeed="${F1_VT_ARSP_BLEND:-${vt_blend_airspeed}}"
    vt_transition_airspeed="${F1_VT_ARSP_TRANS:-${vt_transition_airspeed}}"
fi

if [[ "${require_mode}" != "va_hold" && "${require_mode}" != "f1" \
    && "${require_mode}" != "f2b" ]]; then
    echo "REQUIRE_MODE must be va_hold, f1, or f2b" >&2
    exit 2
fi

# F1 measures later than the engineering Phase-0.75 check: Va and lambda
# must remain jointly settled for two seconds before the fixed three-second
# scientific window starts.
lambda_target_dwell="1.0"
if [[ "${require_mode}" == "f1" || "${require_mode}" == "f2b" ]]; then
    lambda_target_dwell="2.0"
fi

if ! python - "${va_target}" "${lambda_target}" <<'PY'
import math
import sys

va = float(sys.argv[1])
lam = float(sys.argv[2])
if not math.isfinite(va) or va <= 0.0:
    raise SystemExit(1)
if not math.isfinite(lam) or not 0.0 <= lam <= 1.0:
    raise SystemExit(1)
PY
then
    echo "VA_TARGET_MPS must be positive and LAMBDA_TARGET must be in [0, 1]" >&2
    exit 2
fi

if [[ -e "${output_dir}/telemetry.csv" ]]; then
    echo "refusing to overwrite ${output_dir}/telemetry.csv" >&2
    exit 2
fi

mkdir -p "${output_dir}/px4_workdir"

world_arg=()
model_name="${CA_LSC_MODEL_NAME:-standard_vtol}"
extra_gz_resource_path="${CA_LSC_EXTRA_GZ_RESOURCE_PATH:-}"
condition_id="${CA_LSC_CONDITION_ID:-nominal}"
wind_model="${CA_LSC_WIND_MODEL:-none}"
wind_cmd_e_enu="${CA_LSC_WIND_E_ENU:-0.0}"
wind_cmd_n_enu="${CA_LSC_WIND_N_ENU:-0.0}"
wind_cmd_u_enu="${CA_LSC_WIND_U_ENU:-0.0}"
airspeed_source="${CA_LSC_AIRSPEED_SOURCE:-native}"
mass_scale="${CA_LSC_MASS_SCALE:-1.0}"
nominal_model_mass_kg="${CA_LSC_NOMINAL_MODEL_MASS_KG:-nan}"
actual_model_mass_kg="${CA_LSC_ACTUAL_MODEL_MASS_KG:-nan}"
payload_mass_kg="${CA_LSC_PAYLOAD_MASS_KG:-0.0}"
wing_force_gt_enabled="${CA_LSC_WING_FORCE_GT_ENABLED:-false}"
wing_force_left_gz_topic="${CA_LSC_WING_FORCE_LEFT_GZ_TOPIC:-/ca_lsc/f3/wing_left/lift}"
wing_force_right_gz_topic="${CA_LSC_WING_FORCE_RIGHT_GZ_TOPIC:-/ca_lsc/f3/wing_right/lift}"
elevator_joint_position_gz_topic="${CA_LSC_ELEVATOR_JOINT_GZ_TOPIC:-/ca_lsc/f3/elevator/joint_position}"
elevator_wrench_gz_topic="${CA_LSC_ELEVATOR_WRENCH_GZ_TOPIC:-}"
elevator_effective_angle_gz_topic="${CA_LSC_ELEVATOR_EFFECTIVE_ANGLE_GZ_TOPIC:-}"
elevator_dither_scale_gz_topic="${CA_LSC_ELEVATOR_DITHER_SCALE_GZ_TOPIC:-}"
eta_l_estimated_mass_kg="${CA_LSC_ESTIMATED_MASS_KG:-${actual_model_mass_kg}}"
f3b2_test_mode="${CA_LSC_F3B2_TEST_MODE:-}"
f3b2_shadow_demand_offset="${CA_LSC_F3B2_SHADOW_DEMAND_OFFSET_NM:-0.0}"
f3b2_shadow_demand_for_eta_c="${CA_LSC_F3B2_SHADOW_DEMAND_FOR_ETA_C_NM:-nan}"
f3b2_elevator_bias="${CA_LSC_F3B2_ELEVATOR_BIAS_RAD:-0.0}"
f3b2_stage_index="${CA_LSC_F3B2_STAGE_INDEX:--1.0}"
f3b2_gt_dmoment_ddelta_per_q="${CA_LSC_F3B2_GT_DMOMENT_DDELTA_PER_Q_M3:-nan}"
elevator_id_dither_enabled="${CA_LSC_ELEVATOR_ID_DITHER_ENABLED:-false}"
gust_amplitude="${CA_LSC_GUST_AMPLITUDE_MPS:-0.0}"
gust_delay="${CA_LSC_GUST_DELAY_S:-0.5}"
gust_duration="${CA_LSC_GUST_DURATION_S:-2.0}"
gust_recovery="${CA_LSC_GUST_RECOVERY_S:-2.0}"
schedule_mode="va_hold_target"
if [[ "${require_mode}" == "f2b" ]]; then
    schedule_mode="gust_target"
fi

optional_launch_args=()
add_optional_launch_arg() {
    local name="$1"
    local value="$2"
    if [[ -n "${value}" ]]; then
        optional_launch_args+=("${name}:=${value}")
    fi
}
add_optional_launch_arg "extra_gz_resource_path" "${extra_gz_resource_path}"
add_optional_launch_arg "elevator_wrench_gz_topic" "${elevator_wrench_gz_topic}"
add_optional_launch_arg "elevator_effective_angle_gz_topic" "${elevator_effective_angle_gz_topic}"
add_optional_launch_arg "elevator_dither_scale_gz_topic" "${elevator_dither_scale_gz_topic}"
add_optional_launch_arg "f3b2_test_mode" "${f3b2_test_mode}"

if [[ -n "${CA_LSC_WORLD:-}" ]]; then
    world_arg+=(world:="${CA_LSC_WORLD}")
fi
if [[ -n "${CA_LSC_CONDITION_JSON:-}" ]]; then
    if [[ ! -s "${CA_LSC_CONDITION_JSON}" ]]; then
        echo "CA_LSC_CONDITION_JSON does not exist: ${CA_LSC_CONDITION_JSON}" >&2
        exit 2
    fi
    cp "${CA_LSC_CONDITION_JSON}" "${output_dir}/condition.json"
fi

read -r va_velocity_min va_velocity_max max_horizontal_speed \
    fw_min_forward_speed <<EOF
$(python - "${va_target}" "${vt_blend_airspeed}" \
    "${vt_transition_airspeed}" <<'PY'
import math
import sys

va = float(sys.argv[1])
blend = float(sys.argv[2])
transition = float(sys.argv[3])
velocity_min = max(3.0, va - 2.0)
velocity_max = max(velocity_min + 1.0, va + 3.0)
max_horizontal_speed = velocity_max
fw_min_forward_speed = max(3.0, min(15.0, va))

if not all(math.isfinite(value) for value in (blend, transition)):
    raise SystemExit('VTOL blend/transition airspeeds must be finite')
if not 0.0 <= blend <= transition <= 30.0:
    raise SystemExit(
        'require 0 <= F1_VT_ARSP_BLEND <= F1_VT_ARSP_TRANS <= 30'
    )

print(
    f"{velocity_min:.3f} {velocity_max:.3f} "
    f"{max_horizontal_speed:.3f} {fw_min_forward_speed:.3f}"
)
PY
)
EOF

echo "Frozen low-level PX4 configuration: VT_ARSP_BLEND=${vt_blend_airspeed}, VT_ARSP_TRANS=${vt_transition_airspeed}, FW_AIRSPD_MIN/TRIM/MAX=${fw_airspeed_min}/${fw_airspeed_trim}/${fw_airspeed_max}"

set +u
source /opt/ros/humble/setup.bash
source "${project_root}/ros2_ws/install_ca/setup.bash"
set -u
export PYTHONNOUSERSITE=1

set +e
timeout --signal=INT --kill-after=15s "${timeout_seconds}s" \
    ros2 launch ca_lsc_transition nominal_transition.launch.py \
    "${world_arg[@]}" \
    px4_dir:="${project_root}/PX4-Autopilot" \
    px4_build_dir:="${project_root}/PX4-Autopilot/build_ca_make" \
    model_name:="${model_name}" \
    "${optional_launch_args[@]}" \
    condition_id:="${condition_id}" \
    wind_model:="${wind_model}" \
    wind_cmd_e_enu:="${wind_cmd_e_enu}" \
    wind_cmd_n_enu:="${wind_cmd_n_enu}" \
    wind_cmd_u_enu:="${wind_cmd_u_enu}" \
    airspeed_source:="${airspeed_source}" \
    mass_scale:="${mass_scale}" \
    nominal_model_mass_kg:="${nominal_model_mass_kg}" \
    actual_model_mass_kg:="${actual_model_mass_kg}" \
    payload_mass_kg:="${payload_mass_kg}" \
    wing_force_gt_enabled:="${wing_force_gt_enabled}" \
    wing_force_left_gz_topic:="${wing_force_left_gz_topic}" \
    wing_force_right_gz_topic:="${wing_force_right_gz_topic}" \
    elevator_joint_position_gz_topic:="${elevator_joint_position_gz_topic}" \
    eta_l_estimated_mass_kg:="${eta_l_estimated_mass_kg}" \
    f3b2_shadow_demand_offset:="${f3b2_shadow_demand_offset}" \
    f3b2_shadow_demand_for_eta_c:="${f3b2_shadow_demand_for_eta_c}" \
    f3b2_elevator_bias:="${f3b2_elevator_bias}" \
    f3b2_stage_index:="${f3b2_stage_index}" \
    f3b2_gt_dmoment_ddelta_per_q_m3:="${f3b2_gt_dmoment_ddelta_per_q}" \
    elevator_id_dither_enabled:="${elevator_id_dither_enabled}" \
    max_horizontal_speed:="${max_horizontal_speed}" \
    fw_min_forward_speed:="${fw_min_forward_speed}" \
    fw_airspeed_min:="${fw_airspeed_min}" \
    fw_airspeed_trim:="${fw_airspeed_trim}" \
    fw_airspeed_max:="${fw_airspeed_max}" \
    vt_blend_airspeed:="${vt_blend_airspeed}" \
    vt_transition_airspeed:="${vt_transition_airspeed}" \
    lambda_attitude_blend_start:=0.1 \
    lambda_attitude_blend_full:=0.9 \
    schedule_mode:="${schedule_mode}" \
    gust_amplitude:="${gust_amplitude}" \
    gust_delay:="${gust_delay}" \
    gust_duration:="${gust_duration}" \
    gust_recovery:="${gust_recovery}" \
    va_target:="${va_target}" \
    va_target_tolerance:=0.3 \
    va_target_dwell:=1.0 \
    va_measurement_s:=3.0 \
    va_measurement_min_band_fraction:=0.9 \
    va_measurement_max_abs_airspeed_error:=0.6 \
    va_velocity_kp:=0.8 \
    va_velocity_min:="${va_velocity_min}" \
    va_velocity_max:="${va_velocity_max}" \
    va_altitude_kp:=0.35 \
    va_altitude_vertical_speed_kd:=0.5 \
    va_altitude_velocity_limit:=1.5 \
    max_vertical_speed:=1.5 \
    vt_unload_altitude_pitch_kp:=2.0 \
    vt_unload_vertical_speed_pitch_kd:=2.0 \
    vt_unload_pitch_min:=-12.0 \
    vt_unload_pitch_max:=10.0 \
    pusher_airspeed_ff:=0.25 \
    pusher_airspeed_kp:=0.08 \
    pusher_airspeed_ki:=0.03 \
    pusher_throttle_min:=0.0 \
    pusher_throttle_max:=0.45 \
    pusher_handover_below_target:=1.0 \
    pusher_throttle_slew_up:=0.5 \
    pusher_throttle_slew_down:=1.5 \
    va_filter_alpha:=0.35 \
    va_runaway_margin:=3.0 \
    va_runaway_dwell:=0.5 \
    va_altitude_abort_error:=5.0 \
    va_vertical_speed_abort:=2.0 \
    va_vertical_abort_min_airspeed:=6.0 \
    lambda_target:="${lambda_target}" \
    lambda_target_tolerance:=0.03 \
    lambda_target_dwell:="${lambda_target_dwell}" \
    vt_unload_test_enable:=1 \
    output_csv:="${output_dir}/telemetry.csv" \
    px4_workdir:="${output_dir}/px4_workdir" \
    2>&1 | tee "${output_dir}/console.log"
launch_status=${PIPESTATUS[0]}
set -e

if [[ ! -s "${output_dir}/telemetry.csv" ]]; then
    echo "telemetry file was not produced" >&2
    exit 1
fi

set +e
if [[ "${require_mode}" == "f2b" ]]; then
    export PYTHONPATH="${project_root}/src${PYTHONPATH:+:${PYTHONPATH}}"
    python -m ca_lsc_td3.evaluation.f2_gust run \
        "${output_dir}/telemetry.csv" \
        --output "${output_dir}/summary.json" \
        | tee "${output_dir}/summary.stdout.json"
    evaluation_status=${PIPESTATUS[0]}
else
    ros2 run ca_lsc_transition evaluate_transition \
        "${output_dir}/telemetry.csv" \
        --output "${output_dir}/summary.json" \
        --require "${require_mode}" \
        | tee "${output_dir}/summary.stdout.json"
    evaluation_status=${PIPESTATUS[0]}
fi
set -e

if [[ ${launch_status} -ne 0 ]]; then
    exit "${launch_status}"
fi
if [[ ${evaluation_status} -ne 0 ]]; then
    exit "${evaluation_status}"
fi
