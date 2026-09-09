#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
    echo "usage: $0 F1_SUMMARY_DIR_OR_JSON [VA:LAMBDA ...]" >&2
    exit 2
fi

summary_json="$1"
shift
if [[ -d "${summary_json}" ]]; then
    summary_json="${summary_json}/f1_grid_summary.json"
fi
if [[ ! -f "${summary_json}" ]]; then
    echo "summary JSON not found: ${summary_json}" >&2
    exit 2
fi
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1
export PYTHONPATH="${project_root}/src${PYTHONPATH:+:${PYTHONPATH}}"

arguments=()
for point in "$@"; do
    arguments+=(--point "${point}")
done

python -m ca_lsc_td3.evaluation.f1_diagnostics \
    "${summary_json}" "${arguments[@]}"
