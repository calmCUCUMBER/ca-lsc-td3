#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "usage: $0 F1_OUTPUT_ROOT" >&2
    exit 2
fi

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
root="$1"
export PYTHONNOUSERSITE=1
export PYTHONPATH="${project_root}/src:${project_root}/ros2_ws/src/ca_lsc_transition${PYTHONPATH:+:${PYTHONPATH}}"

python - "${root}" <<'PY'
import json
import math
import sys
from pathlib import Path

from ca_lsc_transition.evaluate_run import evaluate


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    return value


root = Path(sys.argv[1]).expanduser().resolve()
csv_files = sorted(root.glob('va_*_lambda_*/run_*/telemetry.csv'))
if not csv_files:
    raise SystemExit(f'no F1 telemetry.csv files found under {root}')
for csv_path in csv_files:
    result = evaluate(csv_path)
    payload = json.dumps(json_safe(result), indent=2, allow_nan=False) + '\n'
    (csv_path.parent / 'summary.json').write_text(payload, encoding='utf-8')
print(f'reevaluated {len(csv_files)} run summaries under {root}')

# Keep each point-level repeat summary consistent with the newly evaluated
# F1-specific gate.  Preserve the detailed per-run fields already archived by
# the flight runner, changing only the mode/key/pass evidence and adding the
# explicit F1 result for auditability.
for point in sorted(root.glob('va_*_lambda_*')):
    repeat_path = point / 'repeat_summary.json'
    if not repeat_path.exists():
        continue
    repeat = json.loads(repeat_path.read_text(encoding='utf-8'))
    archived_runs = {
        int(item.get('run')): item
        for item in repeat.get('runs', [])
        if isinstance(item, dict) and isinstance(item.get('run'), int)
    }
    updated_runs = []
    for index in range(1, 6):
        summary_path = point / f'run_{index}' / 'summary.json'
        item = dict(archived_runs.get(index, {'run': index}))
        item['summary_present'] = summary_path.exists()
        if summary_path.exists():
            summary = json.loads(summary_path.read_text(encoding='utf-8'))
            passed = bool(summary.get('f1_measurement_pass'))
            item['pass'] = passed
            item['f1_measurement_pass'] = passed
            item['phase_0_75_va_hold_pass'] = bool(
                summary.get('phase_0_75_va_hold_pass')
            )
            item['primary_failure_cause'] = summary.get(
                'primary_failure_cause'
            )
            item['secondary_failure_cause'] = summary.get(
                'secondary_failure_cause'
            )
        else:
            item['pass'] = False
            item['f1_measurement_pass'] = False
            item['primary_failure_cause'] = 'missing_summary'
        updated_runs.append(item)
    repeat['mode'] = 'f1_va_lambda'
    repeat['required_key'] = 'f1_measurement_pass'
    repeat['passes'] = sum(bool(item['pass']) for item in updated_runs)
    repeat['total'] = len(updated_runs)
    repeat['runs'] = updated_runs
    repeat_path.write_text(
        json.dumps(json_safe(repeat), indent=2, allow_nan=False) + '\n',
        encoding='utf-8',
    )
print('updated F1 repeat summaries')
PY

python -m ca_lsc_td3.evaluation.f1_grid "${root}" \
    --calibration "${project_root}/data/calibration/model_based/airframe_calibration.json" \
    --output-json "${root}/f1_grid_summary.json" \
    --output-csv "${root}/f1_grid_points.csv"
