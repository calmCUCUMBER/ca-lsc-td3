#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 3 ]]; then
    cat >&2 <<'EOF'
usage: scripts/run_f1_full_grid_v8.sh OUTPUT_ROOT [TIMEOUT_SECONDS] [SEED]

Runs the formal schema-v8 F1 grid:
  Va:     6 8 10 12 14 16 18 20 m/s
  lambda: 0.0 0.1 0.2 ... 1.0

This is 88 cells x 5 independent restarts = 440 runs.  Schema v8 uses one
uninterrupted fixed-condition measurement window; leaving the Va/lambda band
after measurement begins is recorded as non-convergent and is never spliced
with a later segment.  The runner is resumable and will not overwrite an
existing cell with a repeat_summary.json.
EOF
    exit 2
fi

output_root="$1"
timeout_seconds="${2:-220}"
seed="${3:-20260901}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

"${project_root}/scripts/run_f1_va_lambda_grid.sh" \
    "${output_root}" "${timeout_seconds}" \
    "6 8 10 12 14 16 18 20" \
    "0.0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0" \
    "${seed}"

SUMMARY_JSON="${output_root}/f1_grid_summary.json" python - <<'PY'
import json
import os
from pathlib import Path

summary_path = Path(os.environ['SUMMARY_JSON'])
payload = json.loads(summary_path.read_text(encoding='utf-8'))
errors = []
if payload.get('point_count') != 88:
    errors.append(f"expected 88 points, got {payload.get('point_count')}")
if payload.get('telemetry_schema_versions') != [8.0]:
    errors.append(
        'formal F1 requires telemetry schema [8.0], got '
        f"{payload.get('telemetry_schema_versions')}"
    )
for point in payload.get('points', []):
    segments = point.get('va_hold_measurement_segment_count_max')
    if isinstance(segments, (int, float)) and segments > 1.0:
        errors.append(
            f"fragmented measurement at Va={point.get('va_target_mps')}, "
            f"lambda={point.get('lambda_target')}"
        )
if errors:
    raise SystemExit('\n'.join(errors))
print('Formal F1 schema-v8 audit passed: 88 cells, single-window protocol.')
PY
