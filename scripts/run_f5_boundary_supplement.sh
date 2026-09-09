#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    cat >&2 <<'EOF'
usage: scripts/run_f5_boundary_supplement.sh OUTPUT_ROOT [TIMEOUT_SECONDS]

Collects a small current-schema F5 boundary dataset (10 cells x 5 valid
restarts).  It does not reopen F1/F2/F3 or modify the frozen PX4 allocator.

Steady payload pairs:
  (Va,lambda) = (6,.5), (8,.7), (10,.8), (12,1.0)
  mass scale  = 1.0, 1.2
Dynamic pair:
  (Va,lambda) = (14,.9), gust amplitude = 0, 8 m/s
EOF
    exit 2
fi

output_root="$1"
timeout_seconds="${2:-220}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "${output_root}"

slug() {
    python - "$1" <<'PY'
import sys
text = f'{float(sys.argv[1]):.3f}'.rstrip('0').rstrip('.')
print(text.replace('-', 'm').replace('.', 'p'))
PY
}

valid_repeat_count() {
    python - "$1" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1]) / 'repeat_summary.json'
if not path.exists():
    print(0)
    raise SystemExit
item = json.loads(path.read_text(encoding='utf-8'))
value = item.get('valid_runs', item.get('valid_repeat_count', 0))
try:
    print(int(value))
except (TypeError, ValueError):
    print(0)
PY
}

export PYTHONNOUSERSITE=1
export PYTHONPATH="${project_root}/src${PYTHONPATH:+:${PYTHONPATH}}"
: > "${output_root}/f5_supplement_status.tsv"
failures=0

steady_points="6:0.5 8:0.7 10:0.8 12:1.0"
for mass_scale in 1.0 1.2; do
    mass_slug="$(slug "${mass_scale}")"
    condition_id="f5_boundary_m${mass_slug}_wind0"
    condition_root="${output_root}/conditions/${condition_id}"
    model_name="standard_vtol_${condition_id}"
    if [[ ! -s "${condition_root}/condition.json" ]]; then
        "${project_root}/scripts/generate_wind_qualification_world.py" \
            "${condition_root}" --wind-enu 0 0 0 \
            --mass-scale "${mass_scale}" --model-name "${model_name}" \
            --instrument-main-wings --instrument-elevator-moment
    fi
    readarray -t values < <(python - "${condition_root}/condition.json" <<'PY'
import json, sys
item = json.load(open(sys.argv[1], encoding='utf-8'))
for key in (
    'world', 'model_name', 'model_resource_path',
    'nominal_model_mass_kg', 'actual_model_mass_kg', 'payload_mass_kg',
    'wing_left_lift_gz_topic', 'wing_right_lift_gz_topic',
    'elevator_joint_position_gz_topic', 'elevator_wrench_gz_topic',
    'elevator_effective_angle_gz_topic', 'elevator_dither_scale_gz_topic',
):
    print(item.get(key, ''))
PY
    )
    for index in 6 7 8 9 10 11; do
        if [[ -z "${values[$index]:-}" ]]; then
            echo "F5 condition lacks required instrumentation topic: ${condition_root}/condition.json" >&2
            echo "Use a fresh output root so the instrumented model is regenerated." >&2
            exit 2
        fi
    done
    for point in ${steady_points}; do
        va="${point%%:*}"
        lam="${point##*:}"
        point_root="${output_root}/${condition_id}/va$(slug "${va}")_lam$(slug "${lam}")"
        echo "=== F5 boundary: mass=${mass_scale}, Va=${va}, lambda=${lam} ==="
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
        CA_LSC_ELEVATOR_WRENCH_GZ_TOPIC="${values[9]}" \
        CA_LSC_ELEVATOR_EFFECTIVE_ANGLE_GZ_TOPIC="${values[10]}" \
        CA_LSC_ELEVATOR_DITHER_SCALE_GZ_TOPIC="${values[11]}" \
        CA_LSC_ELEVATOR_ID_DITHER_ENABLED=false \
        CA_LSC_WING_FORCE_GT_ENABLED=true \
        CA_LSC_MASS_SCALE="${mass_scale}" \
        CA_LSC_CONDITION_ID="${condition_id}" \
        CA_LSC_CONDITION_JSON="${condition_root}/condition.json" \
        CA_LSC_WIND_MODEL=steady_gazebo_world_wind \
        CA_LSC_WIND_E_ENU=0.0 CA_LSC_WIND_N_ENU=0.0 CA_LSC_WIND_U_ENU=0.0 \
        CA_LSC_AIRSPEED_SOURCE=relative_wind \
        F1_VALID_TARGET=5 F1_MAX_ATTEMPTS=8 \
            "${project_root}/scripts/run_phase0_75_va_hold_repeats.sh" \
            "${point_root}" "${va}" "${lam}" "${timeout_seconds}" f1
        status=$?
        set -e
        valid_count="$(valid_repeat_count "${point_root}")"
        printf 'steady\t%s\t%s\t%s\t%s\t%s\t%s\n' \
            "${mass_scale}" "${va}" "${lam}" "${status}" "${valid_count}" "${point_root}" \
            | tee -a "${output_root}/f5_supplement_status.tsv"
        (( valid_count >= 5 )) || failures=$((failures + 1))
    done
done

gust_condition_id="f5_boundary_dynamic_wind"
gust_root="${output_root}/conditions/${gust_condition_id}"
if [[ ! -s "${gust_root}/condition.json" ]]; then
    "${project_root}/scripts/generate_wind_qualification_world.py" \
        "${gust_root}" --wind-enu 0 0 0 --dynamic-wind \
        --model-name standard_vtol_f5_dynamic \
        --instrument-main-wings --instrument-elevator-moment
fi
readarray -t gust_values < <(python - "${gust_root}/condition.json" <<'PY'
import json, sys
item = json.load(open(sys.argv[1], encoding='utf-8'))
for key in (
    'world', 'model_name', 'model_resource_path',
    'nominal_model_mass_kg', 'actual_model_mass_kg',
    'wing_left_lift_gz_topic', 'wing_right_lift_gz_topic',
    'elevator_joint_position_gz_topic', 'elevator_wrench_gz_topic',
    'elevator_effective_angle_gz_topic', 'elevator_dither_scale_gz_topic',
):
    print(item.get(key, ''))
PY
)
for index in 5 6 7 8 9 10; do
    if [[ -z "${gust_values[$index]:-}" ]]; then
        echo "F5 gust condition lacks required instrumentation topic: ${gust_root}/condition.json" >&2
        echo "Use a fresh output root so the instrumented model is regenerated." >&2
        exit 2
    fi
done

for amplitude in 0 8; do
    point_root="${output_root}/gust_${amplitude}/va14_lam0p9"
    echo "=== F5 boundary: gust=${amplitude}, Va=14, lambda=0.9 ==="
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
    CA_LSC_ELEVATOR_WRENCH_GZ_TOPIC="${gust_values[8]}" \
    CA_LSC_ELEVATOR_EFFECTIVE_ANGLE_GZ_TOPIC="${gust_values[9]}" \
    CA_LSC_ELEVATOR_DITHER_SCALE_GZ_TOPIC="${gust_values[10]}" \
    CA_LSC_ELEVATOR_ID_DITHER_ENABLED=false \
    CA_LSC_WING_FORCE_GT_ENABLED=true \
    CA_LSC_MASS_SCALE=1.0 CA_LSC_PAYLOAD_MASS_KG=0.0 \
    CA_LSC_CONDITION_ID="${gust_condition_id}_${amplitude}mps" \
    CA_LSC_CONDITION_JSON="${gust_root}/condition.json" \
    CA_LSC_WIND_MODEL=deterministic_1cos_tailwind_gazebo \
    CA_LSC_AIRSPEED_SOURCE=relative_wind \
    CA_LSC_GUST_AMPLITUDE_MPS="${amplitude}" \
    CA_LSC_GUST_DELAY_S=0.5 CA_LSC_GUST_DURATION_S=2.0 \
    CA_LSC_GUST_RECOVERY_S=2.0 \
    F2B_VALID_TARGET=5 F2B_MAX_ATTEMPTS=8 \
        "${project_root}/scripts/run_f2b_gust_repeats.sh" \
        "${point_root}" 14 0.9 "${amplitude}" "${timeout_seconds}"
    status=$?
    set -e
    valid_count="$(valid_repeat_count "${point_root}")"
    printf 'gust\t1.0\t14\t0.9\t%s\t%s\t%s\n' "${status}" "${valid_count}" "${point_root}" \
        | tee -a "${output_root}/f5_supplement_status.tsv"
    (( valid_count >= 5 )) || failures=$((failures + 1))
done

python "${project_root}/scripts/audit_f5_dataset.py" \
    "${project_root}/data/evaluation/f3a_eta_l_validation_v2" \
    "${output_root}" \
    --output-dir "${output_root}/f5_audit"

if (( failures > 0 )); then
    echo "F5 supplement finished with ${failures} cells lacking five protocol-valid telemetry runs" >&2
    exit 1
fi

echo "F5 boundary supplement complete: ${output_root}"
echo "Audit: ${output_root}/f5_audit/f5_data_audit.json"
