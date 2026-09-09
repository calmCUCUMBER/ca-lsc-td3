#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
    echo "usage: $0 MODE OUTPUT_ROOT [LAMBDA_TARGET]" >&2
    echo "MODE: native | manual | characterization" >&2
    exit 2
fi

mode="$1"
output_root="$2"
lambda_target="${3:-}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "${output_root}"

case "${mode}" in
    native|manual)
        if [[ -n "${lambda_target}" ]]; then
            echo "LAMBDA_TARGET is only valid in characterization mode" >&2
            exit 2
        fi
        ;;
    characterization)
        if [[ -z "${lambda_target}" ]]; then
            echo "characterization mode requires LAMBDA_TARGET" >&2
            exit 2
        fi
        ;;
    *)
        echo "invalid mode: ${mode}" >&2
        exit 2
        ;;
esac

for run in 1 2 3 4 5; do
    run_dir="${output_root}/run_${run}"
    if [[ -e "${run_dir}/telemetry.csv" ]]; then
        echo "refusing to overwrite ${run_dir}/telemetry.csv" >&2
        exit 2
    fi
done

status_file="${output_root}/repeat_status.tsv"
: > "${status_file}"
failures=0

for run in 1 2 3 4 5; do
    run_dir="${output_root}/run_${run}"
    echo "=== independent restart ${run}/5: ${run_dir} ==="
    run_status="FAIL"
    case "${mode}" in
        native)
            if "${project_root}/scripts/run_transition_test.sh" \
                none "${run_dir}" 180; then
                run_status="PASS"
            fi
            ;;
        manual)
            if "${project_root}/scripts/run_transition_test.sh" \
                manual_airspeed "${run_dir}" 180; then
                run_status="PASS"
            fi
            ;;
        characterization)
            if "${project_root}/scripts/run_lambda_characterization.sh" \
                "${lambda_target}" "${run_dir}" 180; then
                run_status="PASS"
            fi
            ;;
    esac
    if [[ "${run_status}" != "PASS" ]]; then
        failures=$((failures + 1))
    fi
    echo -e "run_${run}\t${run_status}" | tee -a "${status_file}"
done

REPEAT_OUTPUT_ROOT="${output_root}" REPEAT_MODE="${mode}" python - <<'PY'
import json
import os
from pathlib import Path

root = Path(os.environ['REPEAT_OUTPUT_ROOT'])
mode = os.environ['REPEAT_MODE']
required_key = {
    'native': 'test_1_native_transition_pass',
    'manual': 'test_4_manual_lambda_pass',
    'characterization': 'phase_0_5_lambda_characterization_pass',
}[mode]
runs = []
for index in range(1, 6):
    path = root / f'run_{index}' / 'summary.json'
    item = {'run': index, 'summary_present': path.exists()}
    if path.exists():
        summary = json.loads(path.read_text(encoding='utf-8'))
        item.update({
            'pass': bool(summary.get(required_key)),
            'primary_failure_cause': summary.get('primary_failure_cause'),
            'pretransition_stability_dwell_s': summary.get(
                'pretransition_stability_dwell_s'
            ),
            'pretransition_hold_duration_s': summary.get(
                'pretransition_hold_duration_s'
            ),
            'pretransition_hold_mean_altitude_m': summary.get(
                'pretransition_hold_mean_altitude_m'
            ),
            'pretransition_hold_mean_trajectory_sp_altitude_m': summary.get(
                'pretransition_hold_mean_trajectory_sp_altitude_m'
            ),
            'pretransition_height_gate_fraction': summary.get(
                'pretransition_height_gate_fraction'
            ),
            'max_lambda_exec': summary.get('max_lambda_exec'),
        })
    else:
        item.update({
            'pass': False,
            'primary_failure_cause': 'missing_summary',
        })
    runs.append(item)
aggregate = {
    'mode': mode,
    'required_key': required_key,
    'passes': sum(item['pass'] for item in runs),
    'total': len(runs),
    'runs': runs,
}
(root / 'repeat_summary.json').write_text(
    json.dumps(aggregate, indent=2, allow_nan=False) + '\n',
    encoding='utf-8',
)
print(json.dumps(aggregate, indent=2, allow_nan=False))
PY

if [[ "${failures}" -ne 0 ]]; then
    exit 1
fi
