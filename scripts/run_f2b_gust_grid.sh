#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    cat >&2 <<'EOF'
usage: scripts/run_f2b_gust_grid.sh OUTPUT_ROOT [TIMEOUT_SECONDS]

Runs the frozen F2-B 9-cell partial Va-lambda grid for deterministic
longitudinal tailwind gust amplitudes 0, 4, and 8 m/s.  Five protocol-valid
independent restarts are required per cell (135 valid runs total).
EOF
    exit 2
fi

output_root="$1"
timeout_seconds="${2:-220}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
world_root="${output_root}/dynamic_wind_world"
mkdir -p "${output_root}"

if [[ ! -s "${world_root}/condition.json" ]]; then
    "${project_root}/scripts/generate_wind_qualification_world.py" \
        "${world_root}" --wind-enu 0 0 0 --dynamic-wind \
        --model-name standard_vtol_gust
fi

readarray -t condition_values < <(python - "${world_root}/condition.json" <<'PY'
import json, sys
item = json.load(open(sys.argv[1], encoding='utf-8'))
for key in ('world', 'model_name', 'model_resource_path',
            'nominal_model_mass_kg', 'actual_model_mass_kg'):
    print(item[key])
PY
)
export CA_LSC_WORLD="${condition_values[0]}"
export CA_LSC_MODEL_NAME="${condition_values[1]}"
export CA_LSC_EXTRA_GZ_RESOURCE_PATH="${condition_values[2]}"
export CA_LSC_NOMINAL_MODEL_MASS_KG="${condition_values[3]}"
export CA_LSC_ACTUAL_MODEL_MASS_KG="${condition_values[4]}"
export CA_LSC_MASS_SCALE=1.0
export CA_LSC_PAYLOAD_MASS_KG=0.0
export CA_LSC_CONDITION_JSON="${world_root}/condition.json"
export CA_LSC_WIND_MODEL=deterministic_1cos_tailwind_gazebo
export CA_LSC_AIRSPEED_SOURCE=relative_wind
export CA_LSC_GUST_DELAY_S=0.5
export CA_LSC_GUST_DURATION_S=2.0
export CA_LSC_GUST_RECOVERY_S=2.0
export PYTHONNOUSERSITE=1
export PYTHONPATH="${project_root}/src${PYTHONPATH:+:${PYTHONPATH}}"

points="10:0.5 10:0.6 10:0.7 12:0.6 12:0.8 12:1.0 14:0.8 14:0.9 14:1.0"
: > "${output_root}/f2b_point_status.tsv"
failures=0
for amplitude in 0 4 8; do
    export CA_LSC_GUST_AMPLITUDE_MPS="${amplitude}"
    export CA_LSC_CONDITION_ID="f2b_gust_${amplitude}mps"
    gust_slug="${amplitude}"
    for point in ${points}; do
        va="${point%%:*}"
        lam="${point##*:}"
        lam_slug="${lam//./p}"
        point_dir="${output_root}/gust_${gust_slug}/va${va}_lam${lam_slug}"
        set +e
        "${project_root}/scripts/run_f2b_gust_repeats.sh" \
            "${point_dir}" "${va}" "${lam}" "${amplitude}" \
            "${timeout_seconds}"
        status=$?
        set -e
        printf '%s\t%s\t%s\t%s\t%s\n' \
            "${amplitude}" "${va}" "${lam}" "${status}" "${point_dir}" \
            | tee -a "${output_root}/f2b_point_status.tsv"
        (( status == 0 )) || failures=$((failures + 1))
    done
done

python -m ca_lsc_td3.evaluation.f2_gust aggregate "${output_root}" \
    --output-dir "${output_root}/combined" \
    | tee "${output_root}/combined_summary.stdout.json"

if (( failures > 0 )); then
    echo "F2-B finished with ${failures} cells lacking five valid repeats" >&2
    exit 1
fi

