#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    cat >&2 <<'EOF'
usage: scripts/run_f3a_eta_l_validation.sh OUTPUT_ROOT [TIMEOUT_SECONDS]

Formal F3-A physical validation matrix:
  Va/lambda: 6/0.3, 8/0.5, 10/0.5, 12/0.6, 14/0.8, 18/0.8
  mass scale: 1.0, 1.2
  steady longitudinal wind: -4 m/s ENU-X headwind
  repeats: 3 data-quality-valid independent restarts per cell
  supplemental probes: wind0 18/0.8 at m0, wind0 6/0.3 at 1.2m0,
  and 12/1.0 at 8 m/s gust

This is 12 headwind cells plus 3 supplemental cells (45 valid runs).  It
validates eta_L model fidelity over airspeed, payload, angle-of-attack
trajectories, longitudinal headwind, gust exposure/recovery, and known
boundary probes.  It does not alter the frozen lambda-to-PX4 transition
allocator.
EOF
    exit 2
fi

output_root="$1"
timeout_seconds="${2:-220}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
points="6:0.3 8:0.5 10:0.5 12:0.6 14:0.8 18:0.8"
mkdir -p "${output_root}"
: > "${output_root}/f3a_cell_status.tsv"
cell_failures=0

slug() {
    python - "$1" <<'PY'
import sys
text = f'{float(sys.argv[1]):.3f}'.rstrip('0').rstrip('.')
print(text.replace('-', 'm').replace('.', 'p'))
PY
}

for mass_scale in 1.0 1.2; do
        mass_slug="$(slug "${mass_scale}")"
        condition_id="f3a_m${mass_slug}_headwind4"
        condition_root="${output_root}/conditions/${condition_id}"
        model_name="standard_vtol_${condition_id}"
        if [[ ! -s "${condition_root}/condition.json" ]]; then
            "${project_root}/scripts/generate_wind_qualification_world.py" \
                "${condition_root}" \
                --wind-enu -4 0 0 \
                --mass-scale "${mass_scale}" \
                --model-name "${model_name}" \
                --instrument-main-wings
        fi
        readarray -t values < <(python - "${condition_root}/condition.json" <<'PY'
import json
import sys
item = json.load(open(sys.argv[1], encoding='utf-8'))
for key in (
    'world', 'model_name', 'model_resource_path',
    'nominal_model_mass_kg', 'actual_model_mass_kg', 'payload_mass_kg',
    'wing_left_lift_gz_topic', 'wing_right_lift_gz_topic',
    'elevator_joint_position_gz_topic',
):
    print(item[key])
PY
        )

        for point in ${points}; do
            va="${point%%:*}"
            lam="${point##*:}"
            point_root="${output_root}/${condition_id}/va$(slug "${va}")_lam$(slug "${lam}")"
            echo "=== F3-A ${condition_id}: Va=${va}, lambda=${lam} ==="
            set +e
            CA_LSC_WORLD="${values[0]}" \
            CA_LSC_MODEL_NAME="${values[1]}" \
            CA_LSC_EXTRA_GZ_RESOURCE_PATH="${values[2]}" \
            CA_LSC_NOMINAL_MODEL_MASS_KG="${values[3]}" \
            CA_LSC_ACTUAL_MODEL_MASS_KG="${values[4]}" \
            CA_LSC_ESTIMATED_MASS_KG="${values[4]}" \
            CA_LSC_PAYLOAD_MASS_KG="${values[5]}" \
            CA_LSC_WING_FORCE_LEFT_GZ_TOPIC="${values[6]}" \
            CA_LSC_WING_FORCE_RIGHT_GZ_TOPIC="${values[7]}" \
            CA_LSC_ELEVATOR_JOINT_GZ_TOPIC="${values[8]}" \
            CA_LSC_WING_FORCE_GT_ENABLED=true \
            CA_LSC_MASS_SCALE="${mass_scale}" \
            CA_LSC_CONDITION_ID="${condition_id}" \
            CA_LSC_CONDITION_JSON="${condition_root}/condition.json" \
            CA_LSC_WIND_MODEL=steady_gazebo_world_wind \
            CA_LSC_WIND_E_ENU=-4.0 \
            CA_LSC_WIND_N_ENU=0.0 \
            CA_LSC_WIND_U_ENU=0.0 \
            CA_LSC_AIRSPEED_SOURCE=relative_wind \
            F1_VALID_TARGET=3 F1_MAX_ATTEMPTS=5 \
                "${project_root}/scripts/run_phase0_75_va_hold_repeats.sh" \
                "${point_root}" "${va}" "${lam}" \
                "${timeout_seconds}" f1
            status=$?
            set -e
            # A physical F1 failure is useful F3 evidence.  This runner only
            # requires the requested number of data-quality-valid attempts.
            if [[ -s "${point_root}/repeat_summary.json" ]]; then
                valid_runs="$(python - "${point_root}/repeat_summary.json" <<'PY'
import json
import sys
item = json.load(open(sys.argv[1], encoding='utf-8'))
print(int(item.get('valid_runs', 0)))
PY
                )"
                if (( valid_runs >= 3 )); then
                    status=0
                fi
            fi
            printf '%s\t%s\t%s\t%s\t%s\n' \
                "${condition_id}" "${va}" "${lam}" "${status}" \
                "${point_root}" | tee -a "${output_root}/f3a_cell_status.tsv"
            (( status == 0 )) || cell_failures=$((cell_failures + 1))
        done
done

for supplemental in "1.0:0.0:18:0.8" "1.2:0.0:6:0.3"; do
    IFS=: read -r mass_scale wind_e va lam <<<"${supplemental}"
    mass_slug="$(slug "${mass_scale}")"
    condition_id="f3a_m${mass_slug}_wind0"
    condition_root="${output_root}/conditions/${condition_id}"
    model_name="standard_vtol_${condition_id}"
    if [[ ! -s "${condition_root}/condition.json" ]]; then
        "${project_root}/scripts/generate_wind_qualification_world.py" \
            "${condition_root}" \
            --wind-enu "${wind_e}" 0 0 \
            --mass-scale "${mass_scale}" \
            --model-name "${model_name}" \
            --instrument-main-wings
    fi
    readarray -t values < <(python - "${condition_root}/condition.json" <<'PY'
import json
import sys
item = json.load(open(sys.argv[1], encoding='utf-8'))
for key in (
    'world', 'model_name', 'model_resource_path',
    'nominal_model_mass_kg', 'actual_model_mass_kg', 'payload_mass_kg',
    'wing_left_lift_gz_topic', 'wing_right_lift_gz_topic',
    'elevator_joint_position_gz_topic',
):
    print(item[key])
PY
    )
    point_root="${output_root}/${condition_id}/va$(slug "${va}")_lam$(slug "${lam}")"
    echo "=== F3-A supplemental ${condition_id}: Va=${va}, lambda=${lam} ==="
    set +e
    CA_LSC_WORLD="${values[0]}" \
    CA_LSC_MODEL_NAME="${values[1]}" \
    CA_LSC_EXTRA_GZ_RESOURCE_PATH="${values[2]}" \
    CA_LSC_NOMINAL_MODEL_MASS_KG="${values[3]}" \
    CA_LSC_ACTUAL_MODEL_MASS_KG="${values[4]}" \
    CA_LSC_ESTIMATED_MASS_KG="${values[4]}" \
    CA_LSC_PAYLOAD_MASS_KG="${values[5]}" \
    CA_LSC_WING_FORCE_LEFT_GZ_TOPIC="${values[6]}" \
    CA_LSC_WING_FORCE_RIGHT_GZ_TOPIC="${values[7]}" \
    CA_LSC_ELEVATOR_JOINT_GZ_TOPIC="${values[8]}" \
    CA_LSC_WING_FORCE_GT_ENABLED=true \
    CA_LSC_MASS_SCALE="${mass_scale}" \
    CA_LSC_CONDITION_ID="${condition_id}" \
    CA_LSC_CONDITION_JSON="${condition_root}/condition.json" \
    CA_LSC_WIND_MODEL=steady_gazebo_world_wind \
    CA_LSC_WIND_E_ENU="${wind_e}" \
    CA_LSC_WIND_N_ENU=0.0 \
    CA_LSC_WIND_U_ENU=0.0 \
    CA_LSC_AIRSPEED_SOURCE=relative_wind \
    F1_VALID_TARGET=3 F1_MAX_ATTEMPTS=5 \
        "${project_root}/scripts/run_phase0_75_va_hold_repeats.sh" \
        "${point_root}" "${va}" "${lam}" \
        "${timeout_seconds}" f1
    status=$?
    set -e
    if [[ -s "${point_root}/repeat_summary.json" ]]; then
        valid_runs="$(python - "${point_root}/repeat_summary.json" <<'PY'
import json
import sys
item = json.load(open(sys.argv[1], encoding='utf-8'))
print(int(item.get('valid_runs', 0)))
PY
        )"
        if (( valid_runs >= 3 )); then
            status=0
        fi
    fi
    printf '%s\t%s\t%s\t%s\t%s\n' \
        "${condition_id}" "${va}" "${lam}" "${status}" \
        "${point_root}" | tee -a "${output_root}/f3a_cell_status.tsv"
    (( status == 0 )) || cell_failures=$((cell_failures + 1))
done

# Preserve capability samples during dynamic disturbance and before hard
# abort. These are validation probes, not an extension or rerun of F2-B.
gust_condition_id="f3a_dynamic_gust"
gust_condition_root="${output_root}/conditions/${gust_condition_id}"
if [[ ! -s "${gust_condition_root}/condition.json" ]]; then
    "${project_root}/scripts/generate_wind_qualification_world.py" \
        "${gust_condition_root}" --wind-enu 0 0 0 --dynamic-wind \
        --model-name standard_vtol_f3a_dynamic \
        --instrument-main-wings
fi
readarray -t gust_values < <(python - "${gust_condition_root}/condition.json" <<'PY'
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

for probe in 12:1.0:8; do
    IFS=: read -r va lam amplitude <<<"${probe}"
    probe_root="${output_root}/${gust_condition_id}/va$(slug "${va}")_lam$(slug "${lam}")_gust${amplitude}"
    echo "=== F3-A dynamic probe: Va=${va}, lambda=${lam}, gust=${amplitude} ==="
    set +e
    CA_LSC_WORLD="${gust_values[0]}" \
    CA_LSC_MODEL_NAME="${gust_values[1]}" \
    CA_LSC_EXTRA_GZ_RESOURCE_PATH="${gust_values[2]}" \
    CA_LSC_NOMINAL_MODEL_MASS_KG="${gust_values[3]}" \
    CA_LSC_ACTUAL_MODEL_MASS_KG="${gust_values[4]}" \
    CA_LSC_ESTIMATED_MASS_KG="${gust_values[4]}" \
    CA_LSC_WING_FORCE_LEFT_GZ_TOPIC="${gust_values[5]}" \
    CA_LSC_WING_FORCE_RIGHT_GZ_TOPIC="${gust_values[6]}" \
    CA_LSC_ELEVATOR_JOINT_GZ_TOPIC="${gust_values[7]}" \
    CA_LSC_WING_FORCE_GT_ENABLED=true \
    CA_LSC_MASS_SCALE=1.0 CA_LSC_PAYLOAD_MASS_KG=0.0 \
    CA_LSC_CONDITION_ID="${gust_condition_id}_${amplitude}mps" \
    CA_LSC_CONDITION_JSON="${gust_condition_root}/condition.json" \
    CA_LSC_WIND_MODEL=deterministic_1cos_tailwind_gazebo \
    CA_LSC_AIRSPEED_SOURCE=relative_wind \
    CA_LSC_GUST_AMPLITUDE_MPS="${amplitude}" \
    CA_LSC_GUST_DELAY_S=0.5 CA_LSC_GUST_DURATION_S=2.0 \
    CA_LSC_GUST_RECOVERY_S=2.0 \
    F2B_VALID_TARGET=3 F2B_MAX_ATTEMPTS=5 \
        "${project_root}/scripts/run_f2b_gust_repeats.sh" \
        "${probe_root}" "${va}" "${lam}" "${amplitude}" \
        "${timeout_seconds}"
    status=$?
    set -e
    printf '%s\t%s\t%s\t%s\t%s\n' \
        "${gust_condition_id}_${amplitude}mps" "${va}" "${lam}" \
        "${status}" "${probe_root}" \
        | tee -a "${output_root}/f3a_cell_status.tsv"
    (( status == 0 )) || cell_failures=$((cell_failures + 1))
done

export PYTHONNOUSERSITE=1
export PYTHONPATH="${project_root}/src${PYTHONPATH:+:${PYTHONPATH}}"
python -m ca_lsc_td3.evaluation.f3_capabilities validate-eta-l \
    "${output_root}" --output-dir "${output_root}/analysis" \
    | tee "${output_root}/analysis.stdout.json"

echo "F3-A validation complete; cells lacking required valid repeats: ${cell_failures}"
echo "Physical eta_L verdict: ${output_root}/analysis/f3a_eta_l_validation.json"
python - "${output_root}/analysis/f3a_eta_l_validation.json" "${cell_failures}" <<'PY'
import json
import sys
item = json.load(open(sys.argv[1], encoding='utf-8'))
cell_failures = int(sys.argv[2])
if cell_failures:
    raise SystemExit(f'{cell_failures} F3-A cells lack required valid repeats')
if not item.get('eta_l_physical_validation_pass'):
    raise SystemExit('F3-A physical-validation thresholds were not satisfied')
PY
