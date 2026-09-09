#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 4 ]]; then
    cat >&2 <<'EOF'
usage: scripts/run_f2a_payload_smoke.sh OUTPUT_ROOT [TIMEOUT_SECONDS] [POINTS] [MASS_SCALES]

examples:
  scripts/run_f2a_payload_smoke.sh data/evaluation/f2a_payload_smoke_v1
  scripts/run_f2a_payload_smoke.sh data/evaluation/f2a_payload_smoke_v1 220 "6:0.5 8:0.7" "1.0 1.2"

Default points are the minimal F2-A boundary smoke set:
  6:0.5 8:0.7 10:0.9 12:0.6 14:0.9

Each point is run through the fixed F1 measurement protocol with 5 valid
independent restarts and at most 10 attempts.  The aircraft mass perturbation
is implemented as a real fixed payload link in the generated Gazebo model.
EOF
    exit 2
fi

output_root="$1"
timeout_seconds="${2:-220}"
points="${3:-6:0.5 8:0.7 10:0.9 12:0.6 14:0.9}"
mass_scales="${4:-1.0 1.1 1.2}"

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "${output_root}"

slug() {
    python - "$1" <<'PY'
import sys
value = float(sys.argv[1])
text = f'{value:.3f}'.rstrip('0').rstrip('.')
print(text.replace('-', 'm').replace('.', 'p'))
PY
}

status_file="${output_root}/f2a_payload_point_status.tsv"
: > "${status_file}"
failures=0

for mass_scale in ${mass_scales}; do
    mass_slug="$(slug "${mass_scale}")"
    condition_id="f2a_payload_m${mass_slug}"
    condition_root="${output_root}/conditions/${condition_id}"
    model_name="standard_vtol_${condition_id}"

    if [[ ! -s "${condition_root}/condition.json" ]]; then
        "${project_root}/scripts/generate_wind_qualification_world.py" \
            "${condition_root}" \
            --wind-enu 0 0 0 \
            --mass-scale "${mass_scale}" \
            --model-name "${model_name}"
    else
        echo "resume: keeping existing condition ${condition_root}/condition.json"
    fi

    read -r world model resource nominal_mass actual_mass payload_mass < <(
        CONDITION_JSON="${condition_root}/condition.json" python - <<'PY'
import json
import os

condition = json.load(open(os.environ['CONDITION_JSON'], encoding='utf-8'))
print(
    condition['world'],
    condition['model_name'],
    condition['model_resource_path'],
    condition.get('nominal_model_mass_kg', float('nan')),
    condition.get('actual_model_mass_kg', float('nan')),
    condition.get('payload_mass_kg', 0.0),
)
PY
    )

    condition_output_root="${output_root}/${condition_id}"
    mkdir -p "${condition_output_root}"
    cp "${condition_root}/condition.json" "${condition_output_root}/condition.json"

    for point in ${points}; do
        va_target="${point%%:*}"
        lambda_target="${point##*:}"
        va_slug="$(slug "${va_target}")"
        lambda_slug="$(slug "${lambda_target}")"
        run_root="${condition_output_root}/va_${va_slug}_lambda_${lambda_slug}"

        echo "=== F2-A payload grid ${condition_id}: Va=${va_target}, lambda=${lambda_target} ==="
        if CA_LSC_WORLD="${world}" \
            CA_LSC_MODEL_NAME="${model}" \
            CA_LSC_EXTRA_GZ_RESOURCE_PATH="${resource}" \
            CA_LSC_CONDITION_ID="${condition_id}" \
            CA_LSC_CONDITION_JSON="${condition_root}/condition.json" \
            CA_LSC_WIND_MODEL="steady_gazebo_world_wind" \
            CA_LSC_WIND_E_ENU="0.0" \
            CA_LSC_WIND_N_ENU="0.0" \
            CA_LSC_WIND_U_ENU="0.0" \
            CA_LSC_AIRSPEED_SOURCE="relative_wind" \
            CA_LSC_MASS_SCALE="${mass_scale}" \
            CA_LSC_NOMINAL_MODEL_MASS_KG="${nominal_mass}" \
            CA_LSC_ACTUAL_MODEL_MASS_KG="${actual_mass}" \
            CA_LSC_PAYLOAD_MASS_KG="${payload_mass}" \
            "${project_root}/scripts/run_phase0_75_va_hold_repeats.sh" \
                "${run_root}" "${va_target}" "${lambda_target}" \
                "${timeout_seconds}" f1; then
            echo -e "${condition_id}\t${va_target}\t${lambda_target}\tPASS" \
                | tee -a "${status_file}"
        else
            failures=$((failures + 1))
            echo -e "${condition_id}\t${va_target}\t${lambda_target}\tBOUNDARY_OR_FAIL" \
                | tee -a "${status_file}"
        fi
    done

    export PYTHONNOUSERSITE=1
    export PYTHONPATH="${project_root}/src${PYTHONPATH:+:${PYTHONPATH}}"
    python -m ca_lsc_td3.evaluation.f1_grid "${condition_output_root}" \
        --calibration "${project_root}/data/calibration/model_based/airframe_calibration.json" \
        --output-json "${condition_output_root}/f2a_payload_summary.json" \
        --output-csv "${condition_output_root}/f2a_payload_points.csv" \
        --output-boundary-csv "${condition_output_root}/f2a_payload_boundaries.csv" \
        --output-quality-csv "${condition_output_root}/f2a_payload_data_quality_cases.csv" \
        | tee "${condition_output_root}/f2a_payload_summary.stdout.json"
done

echo "F2-A payload grid complete. Boundary/fail point count during execution: ${failures}"
