#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 3 ]]; then
    cat >&2 <<'EOF'
usage: scripts/run_physics_prior_smoke.sh OUTPUT_ROOT [REPEATS=3] [TIMEOUT_SECONDS=220]

Runs the RL-free Physics Prior Only nominal smoke:
  eta_L, eta_C -> lambda_phy -> hard shield/rate limiter -> lambda_exec

This is the last baseline before A0/A1/A2 TD3 training.  It uses a generated
nominal no-wind instrumented Gazebo model so eta_C is available online.
EOF
    exit 2
fi

output_root="$1"
repeats="${2:-3}"
timeout_seconds="${3:-220}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
condition_root="${output_root}/condition"

if ! python - "${repeats}" "${timeout_seconds}" <<'PY'
import sys
repeats = int(sys.argv[1])
timeout = float(sys.argv[2])
if repeats <= 0 or timeout <= 0:
    raise SystemExit(1)
PY
then
    echo "REPEATS and TIMEOUT_SECONDS must be positive" >&2
    exit 2
fi

mkdir -p "${output_root}"
if [[ ! -s "${condition_root}/condition.json" ]]; then
    "${project_root}/scripts/generate_wind_qualification_world.py" \
        "${condition_root}" \
        --wind-enu 0 0 0 \
        --model-name standard_vtol_physics_prior_smoke \
        --instrument-main-wings \
        --instrument-elevator-moment
fi

readarray -t values < <(python - "${condition_root}/condition.json" <<'PY'
import json
import sys

item = json.load(open(sys.argv[1], encoding='utf-8'))
for key in (
    'world', 'model_name', 'model_resource_path',
    'nominal_model_mass_kg', 'actual_model_mass_kg',
    'wing_left_lift_gz_topic', 'wing_right_lift_gz_topic',
    'elevator_joint_position_gz_topic',
    'elevator_wrench_gz_topic',
    'elevator_effective_angle_gz_topic',
    'elevator_dither_scale_gz_topic',
):
    print(item.get(key, ''))
PY
)

source "${project_root}/config/nominal_low_level.env"
export PYTHONNOUSERSITE=1
export PYTHONPATH="${project_root}/src${PYTHONPATH:+:${PYTHONPATH}}"

set +u
source /opt/ros/humble/setup.bash
source "${project_root}/ros2_ws/install_ca/setup.bash"
set -u

for run_index in $(seq 1 "${repeats}"); do
    run_root="${output_root}/run_${run_index}"
    if [[ -e "${run_root}/telemetry.csv" ]]; then
        echo "refusing to overwrite ${run_root}/telemetry.csv" >&2
        exit 2
    fi
    mkdir -p "${run_root}/px4_workdir"
    echo "Physics Prior Only smoke run ${run_index}/${repeats}"

    set +e
    timeout --signal=INT --kill-after=15s "${timeout_seconds}s" \
        ros2 launch ca_lsc_transition nominal_transition.launch.py \
        world:="${values[0]}" \
        px4_dir:="${project_root}/PX4-Autopilot" \
        px4_build_dir:="${project_root}/PX4-Autopilot/build_ca_make" \
        model_name:="${values[1]}" \
        extra_gz_resource_path:="${values[2]}" \
        condition_id:="physics_prior_smoke_run_${run_index}" \
        wind_model:=steady_gazebo_world_wind \
        wind_cmd_e_enu:=0.0 \
        wind_cmd_n_enu:=0.0 \
        wind_cmd_u_enu:=0.0 \
        airspeed_source:=relative_wind \
        mass_scale:=1.0 \
        nominal_model_mass_kg:="${values[3]}" \
        actual_model_mass_kg:="${values[4]}" \
        payload_mass_kg:=0.0 \
        wing_force_gt_enabled:=true \
        wing_force_left_gz_topic:="${values[5]}" \
        wing_force_right_gz_topic:="${values[6]}" \
        elevator_joint_position_gz_topic:="${values[7]}" \
        elevator_wrench_gz_topic:="${values[8]}" \
        elevator_effective_angle_gz_topic:="${values[9]}" \
        elevator_dither_scale_gz_topic:="${values[10]}" \
        eta_l_estimated_mass_kg:="${values[4]}" \
        elevator_id_dither_enabled:=false \
        fw_airspeed_min:="${NOMINAL_FW_AIRSPD_MIN}" \
        fw_airspeed_trim:="${NOMINAL_FW_AIRSPD_TRIM}" \
        fw_airspeed_max:="${NOMINAL_FW_AIRSPD_MAX}" \
        vt_blend_airspeed:="${NOMINAL_VT_ARSP_BLEND}" \
        vt_transition_airspeed:="${NOMINAL_VT_ARSP_TRANS}" \
        lambda_attitude_blend_start:=0.1 \
        lambda_attitude_blend_full:=0.9 \
        schedule_mode:=physics_prior_only \
        va_target:="${CA_LSC_PHY_PUSHER_VA_TARGET:-15.0}" \
        va_target_tolerance:=0.5 \
        vt_unload_test_enable:=1 \
        hold_mc_s:=5.0 \
        hold_fw_s:=8.0 \
        forward_distance_m:=500.0 \
        max_horizontal_speed:=18.0 \
        max_vertical_speed:=1.5 \
        fw_min_forward_speed:=15.0 \
        transition_stability_dwell:=2.0 \
        transition_altitude_tolerance:=1.0 \
        transition_vertical_speed_tolerance:=0.2 \
        transition_groundspeed_tolerance:=0.2 \
        physics_prior_eta_l_lower:="${CA_LSC_PHY_ETA_L_LOWER:-0.15}" \
        physics_prior_eta_l_upper:="${CA_LSC_PHY_ETA_L_UPPER:-0.85}" \
        physics_prior_eta_c_lower:="${CA_LSC_PHY_ETA_C_LOWER:-0.20}" \
        physics_prior_eta_c_upper:="${CA_LSC_PHY_ETA_C_UPPER:-0.80}" \
        physics_prior_hard_eta_l_lower:="${CA_LSC_PHY_HARD_ETA_L_LOWER:-0.05}" \
        physics_prior_hard_eta_l_upper:="${CA_LSC_PHY_HARD_ETA_L_UPPER:-0.35}" \
        physics_prior_hard_eta_c_lower:="${CA_LSC_PHY_HARD_ETA_C_LOWER:-0.05}" \
        physics_prior_hard_eta_c_upper:="${CA_LSC_PHY_HARD_ETA_C_UPPER:-0.35}" \
        physics_prior_altitude_drop_soft:="${CA_LSC_PHY_ALT_DROP_SOFT:-1.0}" \
        physics_prior_altitude_drop_hard:="${CA_LSC_PHY_ALT_DROP_HARD:-4.0}" \
        physics_prior_descent_soft:="${CA_LSC_PHY_DESCENT_SOFT:-0.5}" \
        physics_prior_descent_hard:="${CA_LSC_PHY_DESCENT_HARD:-1.5}" \
        physics_prior_alpha_soft:="${CA_LSC_PHY_ALPHA_SOFT:-0.28}" \
        physics_prior_alpha_hard:="${CA_LSC_PHY_ALPHA_HARD:-0.42}" \
        physics_prior_unloading_rate:="${CA_LSC_PHY_UNLOAD_RATE:-0.25}" \
        physics_prior_recovery_rate:="${CA_LSC_PHY_RECOVERY_RATE:-1.0}" \
        physics_prior_emergency_recovery_rate:="${CA_LSC_PHY_EMERGENCY_RECOVERY_RATE:-2.5}" \
        physics_prior_release_lambda:="${CA_LSC_PHY_RELEASE_LAMBDA:-0.95}" \
        physics_prior_release_dwell:="${CA_LSC_PHY_RELEASE_DWELL:-2.0}" \
        output_csv:="${run_root}/telemetry.csv" \
        px4_workdir:="${run_root}/px4_workdir" \
        2>&1 | tee "${run_root}/console.log"
    launch_status=${PIPESTATUS[0]}
    set -e

    if [[ ! -s "${run_root}/telemetry.csv" ]]; then
        echo "telemetry file was not produced for run ${run_index}" >&2
        exit 1
    fi
    python -m ca_lsc_td3.evaluation.physics_prior_smoke \
        "${run_root}/telemetry.csv" \
        --output "${run_root}/physics_prior_summary.json" \
        | tee "${run_root}/physics_prior_summary.stdout.json"

    if [[ ${launch_status} -ne 0 ]]; then
        exit "${launch_status}"
    fi
done

python -m ca_lsc_td3.evaluation.physics_prior_smoke \
    "${output_root}" \
    --output "${output_root}/physics_prior_repeat_summary.json" \
    | tee "${output_root}/physics_prior_repeat_summary.stdout.json"

echo "Physics Prior Only smoke complete: ${output_root}"
