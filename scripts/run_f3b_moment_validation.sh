#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 3 ]]; then
    cat >&2 <<'EOF'
usage: scripts/run_f3b_moment_validation.sh OUTPUT_ROOT [TIMEOUT_SECONDS] [REPEATS]

Runs a schema-21 F3-B1 elevator-moment GT validation matrix.

Defaults are intentionally small and use stable calibration points:
  (Va, lambda) = (6,0.3), (10,0.6), (14,0.6), (18,0.6)
  repeats = 1
  elevator identification dither = 0.015 rad at 0.75 Hz

Override with:
  CA_LSC_F3B1_POINTS="6:0.3 10:0.6 14:0.6 18:0.6"
  CA_LSC_F3B1_DITHER_AMPLITUDE_RAD="0.015"
  CA_LSC_F3B1_DITHER_FREQUENCY_HZ="0.75"

For diagnostic cartesian sweeps only:
  CA_LSC_F3B1_VA_LIST="6 10 14 18"
  CA_LSC_F3B1_LAMBDA_LIST="0.3 0.6 0.8"
EOF
    exit 2
fi

output_root="$1"
timeout_seconds="${2:-220}"
repeats="${3:-1}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
condition_root="${output_root}/condition"
point_list="${CA_LSC_F3B1_POINTS:-6:0.3 10:0.6 14:0.6 18:0.6}"
va_list="${CA_LSC_F3B1_VA_LIST:-}"
lambda_list="${CA_LSC_F3B1_LAMBDA_LIST:-}"
dither_amplitude="${CA_LSC_F3B1_DITHER_AMPLITUDE_RAD:-0.015}"
dither_frequency="${CA_LSC_F3B1_DITHER_FREQUENCY_HZ:-0.75}"
dither_phase="${CA_LSC_F3B1_DITHER_PHASE_RAD:-0.0}"

if ! python - "${repeats}" <<'PY'
import sys
value = int(sys.argv[1])
if value < 1:
    raise SystemExit(1)
PY
then
    echo "REPEATS must be a positive integer" >&2
    exit 2
fi

mkdir -p "${output_root}"
if [[ ! -s "${condition_root}/condition.json" ]]; then
    "${project_root}/scripts/generate_wind_qualification_world.py" \
        "${condition_root}" \
        --wind-enu 0 0 0 \
        --model-name standard_vtol_f3b1_moment_validation \
        --instrument-main-wings \
        --instrument-elevator-moment \
        --elevator-id-dither-amplitude-rad "${dither_amplitude}" \
        --elevator-id-dither-frequency-hz "${dither_frequency}" \
        --elevator-id-dither-phase-rad "${dither_phase}"
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
    'elevator_id_dither_amplitude_rad',
    'elevator_id_dither_frequency_hz',
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
export CA_LSC_ELEVATOR_ID_DITHER_ENABLED=false
if [[ -n "${dither_amplitude}" && "${dither_amplitude}" != "0" ]]; then
    export CA_LSC_ELEVATOR_ID_DITHER_ENABLED=true
fi
export CA_LSC_WING_FORCE_GT_ENABLED=true
export CA_LSC_MASS_SCALE=1.0
export CA_LSC_PAYLOAD_MASS_KG=0.0
export CA_LSC_CONDITION_JSON="${condition_root}/condition.json"
export CA_LSC_CONDITION_ID=f3b1_elevator_moment_validation
export CA_LSC_WIND_MODEL=steady_gazebo_world_wind
export CA_LSC_WIND_E_ENU=0.0
export CA_LSC_WIND_N_ENU=0.0
export CA_LSC_WIND_U_ENU=0.0
export CA_LSC_AIRSPEED_SOURCE=relative_wind
export PYTHONNOUSERSITE=1
export PYTHONPATH="${project_root}/src${PYTHONPATH:+:${PYTHONPATH}}"

if [[ -n "${dither_amplitude}" && "${dither_amplitude}" != "0" \
    && -z "${CA_LSC_ELEVATOR_EFFECTIVE_ANGLE_GZ_TOPIC}" ]]; then
    echo "condition does not expose elevator_effective_angle_gz_topic; use a new OUTPUT_ROOT or regenerate condition" >&2
    exit 2
fi
if [[ -n "${dither_amplitude}" && "${dither_amplitude}" != "0" \
    && -z "${CA_LSC_ELEVATOR_DITHER_SCALE_GZ_TOPIC}" ]]; then
    echo "condition does not expose elevator_dither_scale_gz_topic; use a new OUTPUT_ROOT or regenerate condition" >&2
    exit 2
fi

status=0
for repeat in $(seq 1 "${repeats}"); do
    if [[ -n "${va_list}" || -n "${lambda_list}" ]]; then
        if [[ -z "${va_list}" || -z "${lambda_list}" ]]; then
            echo "CA_LSC_F3B1_VA_LIST and CA_LSC_F3B1_LAMBDA_LIST must be set together" >&2
            exit 2
        fi
        run_points=()
        for va in ${va_list}; do
            for lambda in ${lambda_list}; do
                run_points+=("${va}:${lambda}")
            done
        done
    else
        read -r -a run_points <<< "${point_list}"
    fi
    for point in "${run_points[@]}"; do
        if [[ "${point}" != *:* ]]; then
            echo "invalid F3-B1 point '${point}', expected VA:LAMBDA" >&2
            exit 2
        fi
        va="${point%%:*}"
        lambda="${point##*:}"
            label="$(python - "${va}" "${lambda}" "${repeat}" <<'PY'
import sys
va = str(sys.argv[1]).replace('.', 'p').replace('-', 'm')
lam = str(sys.argv[2]).replace('.', 'p').replace('-', 'm')
repeat = sys.argv[3]
print(f"va{va}_lam{lam}_run_{repeat}")
PY
)"
            run_root="${output_root}/${label}"
            echo "F3-B1 run: Va=${va}, lambda=${lambda}, repeat=${repeat}"
            set +e
            "${project_root}/scripts/run_va_hold_characterization.sh" \
                "${va}" "${lambda}" "${run_root}" "${timeout_seconds}" f1
            run_status=$?
            set -e
            if (( run_status != 0 )); then
                echo "warning: run failed with status ${run_status}: ${run_root}" >&2
                status=1
            fi
    done
done

read -r -a required_va <<< "$(python - "${point_list}" "${va_list}" <<'PY'
import sys
points = sys.argv[1].split()
va_list = sys.argv[2].split()
values = []
if va_list:
    values = va_list
else:
    values = [point.split(':', 1)[0] for point in points if ':' in point]
seen = []
for value in values:
    if value not in seen:
        seen.append(value)
print(' '.join(seen))
PY
)"

python -m ca_lsc_td3.evaluation.f3b_moment_validation \
    "${output_root}" --output-dir "${output_root}/analysis" \
    --minimum-valid-cells "${#required_va[@]}" \
    --required-va "${required_va[@]}" \
    | tee "${output_root}/analysis.stdout.json"

python - "${output_root}/analysis/f3b1_elevator_moment_validation.json" <<'PY'
import json
import sys

item = json.load(open(sys.argv[1], encoding='utf-8'))
print(
    'F3-B1 elevator moment validation: '
    f"pass={item['f3b1_elevator_moment_validation_pass']}, "
    f"samples={item['sample_count']}, "
    f"slope_fit={item['fit_derivative_per_q_m3']}, "
    f"slope_config={item['configured_derivative_per_q_m3']}, "
    f"R2={item['fit_r_squared']}, "
    f"RMSE={item['fit_rmse_nm']}"
)
if not item['f3b1_elevator_moment_validation_pass']:
    raise SystemExit('F3-B1 elevator moment validation failed')
PY

exit "${status}"
