#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 5 ]]; then
    echo "usage: $0 OUTPUT_DIR [VA_TARGET] [LAMBDA_TARGET] [GUST_AMPLITUDE] [TIMEOUT_SECONDS]" >&2
    exit 2
fi

output_dir="$1"
va_target="${2:-10.0}"
lambda_target="${3:-0.6}"
amplitude="${4:-4.0}"
timeout_seconds="${5:-220}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
world_root="${output_dir}/dynamic_wind_world"
mkdir -p "${output_dir}"

"${project_root}/scripts/generate_wind_qualification_world.py" \
    "${world_root}" --wind-enu 0 0 0 --dynamic-wind \
    --model-name standard_vtol_gust

eval "$(python - "${world_root}/condition.json" <<'PY'
import json, shlex, sys
item = json.load(open(sys.argv[1], encoding='utf-8'))
mapping = {
    'CA_LSC_WORLD': item['world'],
    'CA_LSC_MODEL_NAME': item['model_name'],
    'CA_LSC_EXTRA_GZ_RESOURCE_PATH': item['model_resource_path'],
    'CA_LSC_NOMINAL_MODEL_MASS_KG': item['nominal_model_mass_kg'],
    'CA_LSC_ACTUAL_MODEL_MASS_KG': item['actual_model_mass_kg'],
}
for key, value in mapping.items():
    print(f'export {key}={shlex.quote(str(value))}')
PY
)"
export CA_LSC_MASS_SCALE=1.0
export CA_LSC_PAYLOAD_MASS_KG=0.0
export CA_LSC_CONDITION_JSON="${world_root}/condition.json"
export CA_LSC_CONDITION_ID="f2b_gust_smoke_${amplitude}mps"
export CA_LSC_WIND_MODEL=deterministic_1cos_tailwind_gazebo
export CA_LSC_AIRSPEED_SOURCE=relative_wind
export CA_LSC_GUST_AMPLITUDE_MPS="${amplitude}"
export CA_LSC_GUST_DELAY_S=0.5
export CA_LSC_GUST_DURATION_S=2.0
export CA_LSC_GUST_RECOVERY_S=2.0

"${project_root}/scripts/run_va_hold_characterization.sh" \
    "${va_target}" "${lambda_target}" "${output_dir}/run_1" \
    "${timeout_seconds}" f2b

