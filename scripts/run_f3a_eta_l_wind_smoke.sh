#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    cat >&2 <<'EOF'
usage: scripts/run_f3a_eta_l_wind_smoke.sh OUTPUT_ROOT [TIMEOUT_SECONDS]

Runs one schema-18 F3-A wind instrumentation smoke at Va=10 m/s,
lambda=0.5, with a steady 4 m/s longitudinal headwind.

This is an instrumentation qualification, not a strict eta_L calibration
pass/fail experiment: it checks that wind-relative airspeed, body-flow fields,
beta gating, and Gazebo wing-force ground truth are recorded together.
EOF
    exit 2
fi

output_root="$1"
timeout_seconds="${2:-220}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
condition_root="${output_root}/condition"
run_root="${output_root}/run_1"
expected_wind_e_enu="-4.0"
expected_wind_n_enu="0.0"
expected_wind_u_enu="0.0"

mkdir -p "${output_root}"
if [[ ! -s "${condition_root}/condition.json" ]]; then
    "${project_root}/scripts/generate_wind_qualification_world.py" \
        "${condition_root}" \
        --wind-enu "${expected_wind_e_enu}" "${expected_wind_n_enu}" "${expected_wind_u_enu}" \
        --model-name standard_vtol_f3a_headwind4_smoke \
        --instrument-main-wings
else
    python - "${condition_root}/condition.json" \
        "${expected_wind_e_enu}" "${expected_wind_n_enu}" "${expected_wind_u_enu}" <<'PY'
import json
import math
import sys

condition_path = sys.argv[1]
expected = tuple(float(value) for value in sys.argv[2:5])
item = json.load(open(condition_path, encoding='utf-8'))
actual = tuple(float(value) for value in item.get('wind_velocity_mps', []))
if len(actual) != 3 or any(
    not math.isclose(a, e, abs_tol=1.0e-6)
    for a, e in zip(actual, expected)
):
    raise SystemExit(
        f'{condition_path} has wind_velocity_mps={actual}; '
        f'this smoke requires longitudinal headwind {expected}. '
        'Use a fresh output directory or regenerate the condition.'
    )
PY
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
export CA_LSC_CONDITION_ID=f3a_eta_l_headwind4_smoke
export CA_LSC_CONDITION_JSON="${condition_root}/condition.json"
export CA_LSC_WIND_MODEL=steady_gazebo_world_wind
export CA_LSC_WIND_E_ENU="${expected_wind_e_enu}"
export CA_LSC_WIND_N_ENU="${expected_wind_n_enu}"
export CA_LSC_WIND_U_ENU="${expected_wind_u_enu}"
export CA_LSC_AIRSPEED_SOURCE=relative_wind
export PYTHONNOUSERSITE=1
export PYTHONPATH="${project_root}/src${PYTHONPATH:+:${PYTHONPATH}}"

set +e
"${project_root}/scripts/run_va_hold_characterization.sh" \
    10.0 0.5 "${run_root}" "${timeout_seconds}" f1
flight_status=$?
set -e

python -m ca_lsc_td3.evaluation.f3_capabilities validate-eta-l \
    "${run_root}" --output-dir "${output_root}/analysis" \
    | tee "${output_root}/analysis.stdout.json"

python - "${output_root}/analysis/f3a_eta_l_validation.json" <<'PY'
import json
import math
import sys

item = json.load(open(sys.argv[1], encoding='utf-8'))
run = item['runs'][0]
samples = int(item['ground_truth_sample_count'])
coverage = float(item['ground_truth_coverage_fraction'])
protocol_valid = bool(run.get('protocol_valid'))
schema = None
summary = {}
summary_path = run['source_csv'].replace('/telemetry.csv', '/summary.json')
try:
    with open(summary_path, encoding='utf-8') as stream:
        summary = json.load(stream)
        schema = summary.get('telemetry_schema_version')
except OSError:
    schema = 'unknown'
print(
    'F3-A headwind smoke: '
    f'schema={schema}, samples={samples}, coverage={coverage:.3f}, '
    f'protocol_valid={protocol_valid}, '
    f"wind_parallel_available={run.get('mean_wind_heading_parallel_available_mps')}, "
    f"wind_cross_available={run.get('mean_wind_heading_cross_available_mps')}, "
    f"wind_parallel_force_valid={run.get('mean_wind_heading_parallel_force_valid_mps')}, "
    f"wind_cross_force_valid={run.get('mean_wind_heading_cross_force_valid_mps')}, "
    f"u_available={run.get('mean_vrel_body_u_available_mps')}, "
    f"u_force_valid={run.get('mean_vrel_body_u_force_valid_mps')}, "
    f"abs_beta_available={run.get('mean_abs_beta_available_rad')}, "
    f"abs_beta_force_valid={run.get('mean_abs_beta_force_valid_rad')}"
)
try:
    if float(schema) < 18.0:
        raise SystemExit('F3-A wind instrumentation smoke requires schema >= 18')
except (TypeError, ValueError):
    raise SystemExit('F3-A wind instrumentation smoke could not verify schema')
if not protocol_valid:
    raise SystemExit('F3-A wind instrumentation smoke did not establish target condition')
if samples < 50 or coverage < 0.80:
    raise SystemExit('F3-A wind instrumentation smoke failed')
PY

if (( flight_status != 0 )); then
    echo "note: the F1 flight criterion did not pass; F3 diagnostic data were still recorded" >&2
fi
echo "F3-A headwind instrumentation smoke complete: ${output_root}"
