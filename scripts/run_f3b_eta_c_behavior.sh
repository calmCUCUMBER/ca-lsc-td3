#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    cat >&2 <<'EOF'
usage: scripts/run_f3b_eta_c_behavior.sh OUTPUT_ROOT [TIMEOUT_SECONDS]

Runs the F3-B2 eta_C behavior smoke matrix with the frozen schema-13
transition allocator.  The diagnostic perturbations are read-only:
they change only the shadow eta_C calculation and telemetry labels, never PX4
MC/FW blending or actuator commands.

Stages:
  q_gate:           Va/lambda = 6/0.3, 8/0.3, 10/0.3
  demand:           Va/lambda = 14/0.6, shadow offsets 0.25/0.75/1.25 Nm
  remaining_travel: Va/lambda = 10/0.3, bias 0/0.05/0.10 rad, offset 0.75 Nm
EOF
    exit 2
fi

output_root="$1"
timeout_seconds="${2:-220}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
condition_root="${output_root}/condition"
gt_derivative="${CA_LSC_F3B2_GT_DMOMENT_DDELTA_PER_Q_M3:--0.05799089829280411}"
run_modes=" ${CA_LSC_F3B2_MODES:-q_gate demand remaining_travel} "
remaining_stages="${CA_LSC_F3B2_REMAINING_STAGES:-0:0.00 1:0.05 2:0.10}"
expect_full_matrix=0
if [[ "${run_modes}" == " q_gate demand remaining_travel " \
    && "${remaining_stages}" == "0:0.00 1:0.05 2:0.10" ]]; then
    expect_full_matrix=1
fi
analyzer_extra_args=()
if [[ "${expect_full_matrix}" -eq 0 ]]; then
    analyzer_extra_args+=(--allow-incomplete-behavior)
fi

slug() {
    python - "$1" <<'PY'
import sys
text = f'{float(sys.argv[1]):.3f}'.rstrip('0').rstrip('.')
print(text.replace('-', 'm').replace('.', 'p'))
PY
}

mkdir -p "${output_root}"
if [[ ! -s "${condition_root}/condition.json" ]]; then
    "${project_root}/scripts/generate_wind_qualification_world.py" \
        "${condition_root}" \
        --wind-enu 0 0 0 \
        --model-name standard_vtol_f3b2_eta_c_behavior \
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

export CA_LSC_WORLD="${values[0]}"
export CA_LSC_MODEL_NAME="${values[1]}"
export CA_LSC_EXTRA_GZ_RESOURCE_PATH="${values[2]}"
export CA_LSC_NOMINAL_MODEL_MASS_KG="${values[3]}"
export CA_LSC_ACTUAL_MODEL_MASS_KG="${values[4]}"
export CA_LSC_ESTIMATED_MASS_KG="${values[4]}"
export CA_LSC_WING_FORCE_LEFT_GZ_TOPIC="${values[5]}"
export CA_LSC_WING_FORCE_RIGHT_GZ_TOPIC="${values[6]}"
export CA_LSC_ELEVATOR_JOINT_GZ_TOPIC="${values[7]}"
export CA_LSC_ELEVATOR_WRENCH_GZ_TOPIC="${values[8]}"
export CA_LSC_ELEVATOR_EFFECTIVE_ANGLE_GZ_TOPIC="${values[9]}"
export CA_LSC_ELEVATOR_DITHER_SCALE_GZ_TOPIC="${values[10]}"
export CA_LSC_WING_FORCE_GT_ENABLED=true
export CA_LSC_MASS_SCALE=1.0
export CA_LSC_PAYLOAD_MASS_KG=0.0
export CA_LSC_CONDITION_JSON="${condition_root}/condition.json"
export CA_LSC_CONDITION_ID=f3b2_eta_c_behavior
export CA_LSC_WIND_MODEL=steady_gazebo_world_wind
export CA_LSC_WIND_E_ENU=0.0
export CA_LSC_WIND_N_ENU=0.0
export CA_LSC_WIND_U_ENU=0.0
export CA_LSC_AIRSPEED_SOURCE=relative_wind
export CA_LSC_F3B2_GT_DMOMENT_DDELTA_PER_Q_M3="${gt_derivative}"
export PYTHONNOUSERSITE=1
export PYTHONPATH="${project_root}/src${PYTHONPATH:+:${PYTHONPATH}}"

run_case() {
    local mode="$1"
    local stage="$2"
    local va="$3"
    local lambda="$4"
    local demand_offset="$5"
    local elevator_bias="$6"
    local demand_for_eta_c="$7"
    local label
    label="${mode}_stage${stage}_va$(slug "${va}")_lam$(slug "${lambda}")"
    if [[ "${mode}" == "demand" ]]; then
        label="${label}_demand$(slug "${demand_offset}")"
    elif [[ "${mode}" == "remaining_travel" ]]; then
        label="${label}_bias$(slug "${elevator_bias}")"
    fi
    local run_root="${output_root}/${label}"
    echo "=== F3-B2 ${mode} stage ${stage}: Va=${va}, lambda=${lambda}, demand_offset=${demand_offset}, elevator_bias=${elevator_bias} ==="
    CA_LSC_F3B2_TEST_MODE="${mode}" \
    CA_LSC_F3B2_STAGE_INDEX="${stage}" \
    CA_LSC_F3B2_SHADOW_DEMAND_OFFSET_NM="${demand_offset}" \
    CA_LSC_F3B2_SHADOW_DEMAND_FOR_ETA_C_NM="${demand_for_eta_c}" \
    CA_LSC_F3B2_ELEVATOR_BIAS_RAD="${elevator_bias}" \
        "${project_root}/scripts/run_va_hold_characterization.sh" \
        "${va}" "${lambda}" "${run_root}" "${timeout_seconds}" f1
}

status=0
if [[ "${run_modes}" == *" q_gate "* ]]; then
    for item in "0:6:0.3" "1:8:0.3" "2:10:0.3"; do
        IFS=: read -r stage va lambda <<<"${item}"
        set +e
        run_case q_gate "${stage}" "${va}" "${lambda}" 0.0 0.0 nan
        run_status=$?
        set -e
        (( run_status == 0 )) || status=1
    done
fi

if [[ "${run_modes}" == *" demand "* ]]; then
    for item in "0:0.25" "1:0.75" "2:1.25"; do
        IFS=: read -r stage offset <<<"${item}"
        set +e
        run_case demand "${stage}" 14 0.6 "${offset}" 0.0 nan
        run_status=$?
        set -e
        (( run_status == 0 )) || status=1
    done
fi

if [[ "${run_modes}" == *" remaining_travel "* ]]; then
    for item in ${remaining_stages}; do
        IFS=: read -r stage bias <<<"${item}"
        if [[ -z "${stage}" || -z "${bias}" ]]; then
            echo "invalid CA_LSC_F3B2_REMAINING_STAGES item '${item}', expected STAGE:BIAS" >&2
            exit 2
        fi
        set +e
        run_case remaining_travel "${stage}" 10 0.3 0.0 "${bias}" 0.7
        run_status=$?
        set -e
        (( run_status == 0 )) || status=1
    done
fi

python -m ca_lsc_td3.evaluation.f3b_eta_c_behavior \
    "${output_root}" --output-dir "${output_root}/analysis" \
    "${analyzer_extra_args[@]}" \
    | tee "${output_root}/analysis.stdout.json"

python - "${output_root}/analysis/f3b2_eta_c_behavior_validation.json" \
    "${expect_full_matrix}" <<'PY'
import json
import sys

item = json.load(open(sys.argv[1], encoding='utf-8'))
expect_full_matrix = bool(int(sys.argv[2]))
print(
    'F3-B2 eta_C behavior smoke: '
    f"pass={item['f3b2_eta_c_behavior_smoke_pass']}, "
    f"offline_pass={item['f3b2_eta_c_offline_presmoke_pass']}, "
    f"samples={item['sample_count']}, "
    f"RMSE={item['eta_c_gt_rmse']}, "
    f"Spearman={item['eta_c_gt_spearman']}"
)
if expect_full_matrix and item['f3b2_eta_c_behavior_smoke_pass'] is not True:
    raise SystemExit('F3-B2 eta_C behavior smoke failed')
PY

exit "${status}"
