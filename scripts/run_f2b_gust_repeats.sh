#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 4 || $# -gt 5 ]]; then
    echo "usage: $0 OUTPUT_ROOT VA_TARGET LAMBDA_TARGET GUST_AMPLITUDE [TIMEOUT_SECONDS]" >&2
    exit 2
fi

output_root="$1"
va_target="$2"
lambda_target="$3"
gust_amplitude="$4"
timeout_seconds="${5:-220}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${project_root}/src${PYTHONPATH:+:${PYTHONPATH}}"
valid_target="${F2B_VALID_TARGET:-5}"
max_attempts="${F2B_MAX_ATTEMPTS:-10}"
mkdir -p "${output_root}"

valid=0
attempts=0
: > "${output_root}/repeat_status.tsv"
for run in $(seq 1 "${max_attempts}"); do
    (( valid >= valid_target )) && break
    attempts=$((attempts + 1))
    run_dir="${output_root}/run_${run}"
    echo "=== F2-B attempt ${run}/${max_attempts}: Va=${va_target}, lambda=${lambda_target}, Ag=${gust_amplitude} ==="
    if [[ ! -s "${run_dir}/summary.json" ]]; then
        set +e
        "${project_root}/scripts/run_va_hold_characterization.sh" \
            "${va_target}" "${lambda_target}" "${run_dir}" \
            "${timeout_seconds}" f2b
        set -e
    fi
    quality="missing_summary"
    outcome="unresolved"
    if [[ -s "${run_dir}/summary.json" ]]; then
        read -r quality outcome < <(
            python - "${run_dir}/summary.json" <<'PY'
import json, sys
item = json.load(open(sys.argv[1], encoding='utf-8'))
print(item.get('f2b_attempt_quality', 'protocol_invalid'),
      item.get('f2b_run_outcome', 'unresolved'))
PY
        )
    fi
    if [[ "${quality}" == "valid" ]]; then
        valid=$((valid + 1))
        status="VALID_${outcome}"
    else
        status="PROTOCOL_INVALID:${quality}"
    fi
    printf 'run_%s\t%s\n' "${run}" "${status}" | tee -a "${output_root}/repeat_status.tsv"
done

python - "${output_root}" "${va_target}" "${lambda_target}" \
    "${gust_amplitude}" "${valid_target}" <<'PY'
import json, sys
from pathlib import Path
from ca_lsc_td3.evaluation.f2_gust import _json_safe, classify_point

root = Path(sys.argv[1])
runs = []
for path in sorted(root.glob('run_*/summary.json')):
    item = json.loads(path.read_text(encoding='utf-8'))
    if item.get('experiment') == 'F2B_deterministic_gust_run':
        runs.append(item)
result = classify_point(runs)
result.update({
    'experiment': 'F2B_deterministic_gust_repeats',
    'va_target_mps': float(sys.argv[2]),
    'lambda_target': float(sys.argv[3]),
    'gust_amplitude_mps': float(sys.argv[4]),
    'required_valid_repeats': int(sys.argv[5]),
})
result = _json_safe(result)
(root / 'repeat_summary.json').write_text(
    json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8'
)
print(json.dumps(result, indent=2, allow_nan=False))
PY

if (( valid < valid_target )); then
    echo "F2-B point has only ${valid}/${valid_target} protocol-valid repeats" >&2
    exit 1
fi
