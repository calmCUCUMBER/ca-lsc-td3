#!/usr/bin/env python3
"""Summarize W0/W1/W2 wind qualification telemetry.

The script intentionally separates two questions:

1. Does the aircraft show a dose-response to configured Gazebo wind?
2. Does the logged `airspeed_mps` match ground-minus-wind relative speed?
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path
from statistics import mean


def _finite(value: object) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def _json_safe(value: object) -> object:
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value


def _load_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def _telemetry_rank(path: Path) -> int:
    if path.name == 'telemetry.csv':
        return 0
    match = re.fullmatch(r'telemetry_v(\d+)\.csv', path.name)
    return int(match.group(1)) if match else -1


def _select_telemetry(case_dir: Path) -> Path:
    candidates = [
        path for path in case_dir.glob('telemetry*.csv')
        if _telemetry_rank(path) >= 0
    ]
    if not candidates:
        raise FileNotFoundError(f'missing telemetry*.csv in {case_dir}')
    return max(candidates, key=lambda path: (_telemetry_rank(path), path.name))


def _clean_command_state(value: str) -> str:
    return str(value or '').split('|', 1)[0]


def _values(rows: list[dict[str, str]], key: str) -> list[float]:
    values = [_finite(row.get(key)) for row in rows]
    return [value for value in values if math.isfinite(value)]


def _mean(rows: list[dict[str, str]], key: str) -> float:
    values = _values(rows, key)
    return mean(values) if values else math.nan


def _wind_from_condition(condition: dict[str, object]) -> tuple[float, float, float]:
    vector = condition.get('wind_velocity_mps', [0.0, 0.0, 0.0])
    if not isinstance(vector, list) or len(vector) != 3:
        return math.nan, math.nan, math.nan
    return _finite(vector[0]), _finite(vector[1]), _finite(vector[2])


def _relative_speed(row: dict[str, str], wind_e: float, wind_n: float, wind_u: float) -> float:
    if 'vrel_norm_mps' in row:
        logged = _finite(row.get('vrel_norm_mps'))
        if math.isfinite(logged):
            return logged
    vn = _finite(row.get('vn_mps'))
    ve = _finite(row.get('ve_mps'))
    vu = _finite(row.get('vz_up_mps'))
    if not all(math.isfinite(value) for value in (vn, ve, vu, wind_e, wind_n, wind_u)):
        return math.nan
    return math.sqrt((ve - wind_e) ** 2 + (vn - wind_n) ** 2 + (vu - wind_u) ** 2)


def _phase_summary(
    rows: list[dict[str, str]],
    state: str,
    wind_e: float,
    wind_n: float,
    wind_u: float,
) -> dict[str, object]:
    phase_rows = [
        row for row in rows
        if _clean_command_state(row.get('command_state', '')) == state
    ]
    rel_values = [
        _relative_speed(row, wind_e, wind_n, wind_u) for row in phase_rows
    ]
    rel_values = [value for value in rel_values if math.isfinite(value)]
    airspeed_values = _values(phase_rows, 'airspeed_mps')
    errors = [
        airspeed - rel
        for row in phase_rows
        for airspeed, rel in [(
            _finite(row.get('airspeed_mps')),
            _relative_speed(row, wind_e, wind_n, wind_u),
        )]
        if math.isfinite(airspeed) and math.isfinite(rel)
    ]
    return {
        'rows': len(phase_rows),
        'mean_groundspeed_mps': _mean(phase_rows, 'groundspeed_mps'),
        'mean_airspeed_mps': mean(airspeed_values) if airspeed_values else math.nan,
        'mean_vrel_norm_mps': mean(rel_values) if rel_values else math.nan,
        'mean_airspeed_minus_vrel_mps': mean(errors) if errors else math.nan,
        'mean_pitch_deg': math.degrees(_mean(phase_rows, 'pitch_rad')),
        'mean_lift_thrust_n': _mean(phase_rows, 'lift_thrust_n'),
        'mean_pusher_throttle': _mean(phase_rows, 'pusher_throttle_status'),
        'mean_altitude_relative_m': _mean(phase_rows, 'altitude_relative_m'),
    }


def _summarize_case(case_dir: Path) -> dict[str, object]:
    condition_path = case_dir / 'condition.json'
    telemetry_path = _select_telemetry(case_dir)
    if not condition_path.exists():
        raise FileNotFoundError(f'missing condition.json: {condition_path}')
    condition = json.loads(condition_path.read_text(encoding='utf-8'))
    rows = _load_rows(telemetry_path)
    wind_e, wind_n, wind_u = _wind_from_condition(condition)
    schema_versions = sorted({
        _finite(row.get('schema_version')) for row in rows
        if math.isfinite(_finite(row.get('schema_version')))
    })
    condition_ids = sorted({
        row.get('condition_id', '') for row in rows
        if row.get('condition_id', '')
    })
    return {
        'case_dir': str(case_dir.resolve()),
        'telemetry_csv': str(telemetry_path.resolve()),
        'condition_id_from_json': case_dir.name,
        'condition_id_from_telemetry': condition_ids,
        'wind_velocity_enu_mps': [wind_e, wind_n, wind_u],
        'schema_versions': schema_versions,
        'telemetry_rows': len(rows),
        'has_schema14_wind_columns': 'vrel_norm_mps' in (rows[0] if rows else {}),
        'hold_mc': _phase_summary(rows, 'HOLD_MC', wind_e, wind_n, wind_u),
        'hold_fw': _phase_summary(rows, 'HOLD_FW', wind_e, wind_n, wind_u),
    }


def _write_csv(cases: list[dict[str, object]], path: Path) -> None:
    fields = [
        'case',
        'telemetry_csv',
        'wind_e_enu_mps',
        'wind_n_enu_mps',
        'wind_u_enu_mps',
        'schema_versions',
        'has_schema14_wind_columns',
        'hold_mc_rows',
        'hold_mc_mean_groundspeed_mps',
        'hold_mc_mean_airspeed_mps',
        'hold_mc_mean_vrel_norm_mps',
        'hold_mc_mean_airspeed_minus_vrel_mps',
        'hold_mc_mean_pitch_deg',
        'hold_mc_mean_lift_thrust_n',
        'hold_fw_rows',
        'hold_fw_mean_groundspeed_mps',
        'hold_fw_mean_airspeed_mps',
        'hold_fw_mean_vrel_norm_mps',
        'hold_fw_mean_airspeed_minus_vrel_mps',
        'hold_fw_mean_pitch_deg',
        'hold_fw_mean_lift_thrust_n',
        'hold_fw_mean_pusher_throttle',
        'hold_fw_mean_altitude_relative_m',
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        for case in cases:
            wind = case['wind_velocity_enu_mps']
            hold_mc = case['hold_mc']
            hold_fw = case['hold_fw']
            writer.writerow({
                'case': Path(str(case['case_dir'])).name,
                'telemetry_csv': case['telemetry_csv'],
                'wind_e_enu_mps': wind[0],
                'wind_n_enu_mps': wind[1],
                'wind_u_enu_mps': wind[2],
                'schema_versions': case['schema_versions'],
                'has_schema14_wind_columns': case['has_schema14_wind_columns'],
                **{f'hold_mc_{key}': value for key, value in hold_mc.items()},
                **{f'hold_fw_{key}': value for key, value in hold_fw.items()},
            })


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'root',
        type=Path,
        help='Directory containing WQ case directories with condition.json and telemetry.csv.',
    )
    parser.add_argument('--output-json', type=Path)
    parser.add_argument('--output-csv', type=Path)
    args = parser.parse_args()

    case_dirs = sorted(
        item for item in args.root.iterdir()
        if item.is_dir() and (item / 'condition.json').exists()
    )
    if not case_dirs:
        raise SystemExit(f'no wind qualification cases found under {args.root}')
    cases = [_summarize_case(case_dir) for case_dir in case_dirs]
    payload = {
        'root': str(args.root.resolve()),
        'case_count': len(cases),
        'interpretation': (
            'physical wind smoke can be assessed from dose-response metrics; '
            'wind-relative airspeed is supported only if mean airspeed-minus-'
            'vrel is near zero in wind cases'
        ),
        'cases': cases,
    }
    output_json = args.output_json or args.root / 'wind_qualification_summary.json'
    output_csv = args.output_csv or args.root / 'wind_qualification_summary.csv'
    payload = _json_safe(payload)
    output_json.write_text(
        json.dumps(payload, indent=2, allow_nan=False) + '\n',
        encoding='utf-8',
    )
    _write_csv(cases, output_csv)
    print(json.dumps(payload, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
