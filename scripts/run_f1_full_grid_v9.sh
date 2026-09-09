#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 3 ]]; then
    cat >&2 <<'EOF'
usage: scripts/run_f1_full_grid_v9.sh OUTPUT_ROOT [TIMEOUT_SECONDS] [SEED]

Runs the formal protocol-v9 F1 grid with transition-allocation telemetry
schema 13:
  Va:     6 8 10 12 14 16 18 20 m/s
  lambda: 0.0 0.1 0.2 ... 1.0

Each cell targets five data-quality-valid repeats in at most ten audited
attempts.  Schema v9 starts measurement only after a two-second joint
Va/lambda dwell, then records one fixed continuous three-second window.
Soft-band quality is decided after the window using 90% occupancy and a
0.6 m/s maximum airspeed-error guard; hard safety aborts remain immediate.
EOF
    exit 2
fi

output_root="$1"
timeout_seconds="${2:-220}"
seed="${3:-20260902}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Formal data collection is allowed only against the exact schema-13
# transition allocator and protocol that passed the nine-point architecture
# verification.  Fail before creating any new run directories if a critical
# PX4/ROS/config file has drifted.
python "${project_root}/scripts/check_transition_allocation_freeze.py" \
    --manifest "${project_root}/config/transition_allocation_freeze.json"

F1_MAX_ATTEMPTS=10 "${project_root}/scripts/run_f1_va_lambda_grid.sh" \
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
if payload.get('telemetry_schema_versions') != [13.0]:
    errors.append(
        'formal F1 protocol-v9 requires transition-allocation telemetry '
        'schema [13.0], got '
        f"{payload.get('telemetry_schema_versions')}"
    )
if not payload.get('protocol_consistent'):
    errors.append('formal F1 protocol configurations are inconsistent')
for point in payload.get('points', []):
    label = (
        f"Va={point.get('va_target_mps')}, "
        f"lambda={point.get('lambda_target')}"
    )
    if point.get('valid_f1_total', 0) < 5:
        errors.append(f"{label}: fewer than five valid repeats")
    if point.get('attempt_count', 0) > 10:
        errors.append(f"{label}: exceeded ten-attempt audit cap")
    segments = point.get('va_hold_measurement_segment_count_max')
    if isinstance(segments, (int, float)) and segments > 1.0:
        errors.append(f"{label}: fragmented measurement")
if errors:
    raise SystemExit('\n'.join(errors))
print(
    'Formal F1 protocol-v9 / telemetry-schema-13 audit passed: 88 cells, '
    'five valid repeats per cell, fixed-window protocol.'
)
PY
