#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    cat >&2 <<'EOF'
usage: scripts/run_f3b_eta_c_smoke.sh OUTPUT_ROOT [TIMEOUT_SECONDS]

Runs one F3-B eta_C instrumentation smoke:
  Va=12 m/s, lambda=0.6, m=m0, wind=0

The smoke verifies that PX4 virtual-FW pitch demand, Gazebo elevator angle,
directional remaining elevator authority, qbar gate, and eta_C are recorded
as read-only telemetry.  It does not let eta_C modify PX4 blending.
EOF
    exit 2
fi

output_root="$1"
timeout_seconds="${2:-220}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
condition_root="${output_root}/condition"
run_root="${output_root}/run_1"

mkdir -p "${output_root}"
if [[ ! -s "${condition_root}/condition.json" ]]; then
    "${project_root}/scripts/generate_wind_qualification_world.py" \
        "${condition_root}" \
        --wind-enu 0 0 0 \
        --model-name standard_vtol_f3b_eta_c_smoke \
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
export CA_LSC_WING_FORCE_GT_ENABLED=true
export CA_LSC_MASS_SCALE=1.0
export CA_LSC_PAYLOAD_MASS_KG=0.0
export CA_LSC_CONDITION_ID=f3b_eta_c_smoke
export CA_LSC_CONDITION_JSON="${condition_root}/condition.json"
export CA_LSC_WIND_MODEL=steady_gazebo_world_wind
export CA_LSC_WIND_E_ENU=0.0
export CA_LSC_WIND_N_ENU=0.0
export CA_LSC_WIND_U_ENU=0.0
export CA_LSC_AIRSPEED_SOURCE=relative_wind
export PYTHONNOUSERSITE=1
export PYTHONPATH="${project_root}/src${PYTHONPATH:+:${PYTHONPATH}}"

set +e
"${project_root}/scripts/run_va_hold_characterization.sh" \
    12.0 0.6 "${run_root}" "${timeout_seconds}" f1
flight_status=$?
set -e

python -m ca_lsc_td3.evaluation.f3b_control_authority smoke \
    "${run_root}" --output-dir "${output_root}/analysis" \
    | tee "${output_root}/analysis.stdout.json"

python - "${output_root}/analysis/f3b_eta_c_smoke.json" <<'PY'
import json
import sys

item = json.load(open(sys.argv[1], encoding='utf-8'))
print(
    'F3-B eta_C smoke: '
    f"pass={item['f3b_eta_c_instrumentation_smoke_pass']}, "
    f"schema={item['schema_versions']}, "
    f"valid={item['eta_c_valid_sample_count']}, "
    f"measurement_valid={item['eta_c_valid_measurement_sample_count']}, "
    f"low_q_gate_max={item['low_q_gate_max']}, "
    f"measurement_q_gate_mean={item['measurement_q_gate_mean']}, "
    f"eta_c_measurement_mean={item['eta_c_measurement_mean']}"
)
if not item['f3b_eta_c_instrumentation_smoke_pass']:
    raise SystemExit('F3-B eta_C instrumentation smoke failed')
PY

if (( flight_status != 0 )); then
    echo "note: the F1 flight criterion did not pass; F3-B diagnostic data were still recorded" >&2
fi
echo "F3-B eta_C instrumentation smoke complete: ${output_root}"
