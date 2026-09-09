#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    cat >&2 <<'EOF'
usage: scripts/run_f3a_eta_l_smoke.sh OUTPUT_ROOT [TIMEOUT_SECONDS]

Runs one schema-18 F3-A instrumentation smoke at Va=12 m/s, lambda=0.6.
The generated model replaces only the two main-wing LiftDrag force producers
with an equation-equivalent plugin that publishes the exact lift applied to
Gazebo physics.  It also publishes the read-only elevator joint position.
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
        --model-name standard_vtol_f3a_smoke \
        --instrument-main-wings
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
):
    print(item[key])
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
export CA_LSC_WING_FORCE_GT_ENABLED=true
export CA_LSC_MASS_SCALE=1.0
export CA_LSC_PAYLOAD_MASS_KG=0.0
export CA_LSC_CONDITION_ID=f3a_eta_l_smoke
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

python -m ca_lsc_td3.evaluation.f3_capabilities validate-eta-l \
    "${run_root}" --output-dir "${output_root}/analysis" \
    | tee "${output_root}/analysis.stdout.json"

python - "${output_root}/analysis/f3a_eta_l_validation.json" <<'PY'
import json
import sys

item = json.load(open(sys.argv[1], encoding='utf-8'))
samples = int(item['ground_truth_sample_count'])
coverage = float(item['ground_truth_coverage_fraction'])
print(f'F3-A instrumentation smoke: samples={samples}, coverage={coverage:.3f}')
if samples < 50 or coverage < 0.80:
    raise SystemExit('F3-A instrumentation smoke failed')
PY

if (( flight_status != 0 )); then
    echo "note: the F1 flight criterion did not pass; F3 diagnostic data were still recorded" >&2
fi
echo "F3-A instrumentation smoke complete: ${output_root}"
