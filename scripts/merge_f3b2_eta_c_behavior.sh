#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 ]]; then
    cat >&2 <<'EOF'
usage: scripts/merge_f3b2_eta_c_behavior.sh BASE_V1_ROOT REMAINING_ROOT... OUTPUT_ROOT

Builds a non-destructive merged F3-B2 behavior-smoke directory:
  - q_gate and demand runs are linked from BASE_V1_ROOT
  - remaining_travel runs are linked from REMAINING_ROOT...
    Later remaining roots replace earlier roots for the same stage. This lets
    a stage-2 replacement run override an earlier protocol-invalid stage-2 run
    while preserving valid stage-0/stage-1 runs.
Then it reruns the F3-B2 eta_C behavior analyzer on OUTPUT_ROOT.
EOF
    exit 2
fi

base_root="$(realpath "$1")"
output_root="${!#}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
remaining_roots=()
for arg in "${@:2:$(($# - 2))}"; do
    remaining_roots+=("$(realpath "${arg}")")
done

if [[ -e "${output_root}" ]]; then
    echo "refusing to overwrite existing ${output_root}" >&2
    exit 2
fi
mkdir -p "${output_root}"

link_run() {
    local source="$1"
    local destination="${output_root}/$(basename "${source}")"
    mkdir -p "${destination}"
    local linked=0
    for name in telemetry.csv summary.json console.log summary.stdout.json condition.json; do
        if [[ -e "${source}/${name}" ]]; then
            ln -s "$(realpath "${source}/${name}")" "${destination}/${name}"
            linked=1
        fi
    done
    if [[ "${linked}" -eq 0 ]]; then
        echo "warning: no known run files linked from ${source}" >&2
    fi
}

for source in "${base_root}"/q_gate_* "${base_root}"/demand_*; do
    [[ -d "${source}" ]] || continue
    link_run "${source}"
done

declare -A remaining_by_stage=()
for remaining_root in "${remaining_roots[@]}"; do
    for source in "${remaining_root}"/remaining_travel_stage*; do
        [[ -d "${source}" ]] || continue
        name="$(basename "${source}")"
        stage="${name#remaining_travel_stage}"
        stage="${stage%%_*}"
        if [[ -z "${stage}" || "${stage}" == "${name}" ]]; then
            echo "warning: cannot infer remaining-travel stage from ${source}" >&2
            continue
        fi
        remaining_by_stage["${stage}"]="${source}"
    done
done

for stage in 0 1 2; do
    source="${remaining_by_stage[${stage}]:-}"
    if [[ -z "${source}" ]]; then
        echo "missing remaining_travel stage ${stage} in remaining roots" >&2
        exit 2
    fi
    link_run "${source}"
done

telemetry_count="$(find "${output_root}" -mindepth 2 -maxdepth 2 -name telemetry.csv -print | wc -l)"
if [[ "${telemetry_count}" -ne 9 ]]; then
    echo "expected 9 linked telemetry.csv files, got ${telemetry_count}" >&2
    exit 2
fi
find "${output_root}" -mindepth 2 -maxdepth 2 -name telemetry.csv -print | sort

cd "${project_root}"
export PYTHONNOUSERSITE=1
export PYTHONPATH="${project_root}/src${PYTHONPATH:+:${PYTHONPATH}}"
python -m ca_lsc_td3.evaluation.f3b_eta_c_behavior \
    "${output_root}" --output-dir "${output_root}/analysis" \
    | tee "${output_root}/analysis.stdout.json"
