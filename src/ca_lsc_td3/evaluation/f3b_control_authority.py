"""F3-B eta_C instrumentation smoke checks."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import fmean
from typing import Iterable


def _finite(value: object) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def _mean(values: Iterable[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return fmean(finite) if finite else math.nan


def _fraction(flags: Iterable[bool]) -> float:
    values = list(flags)
    return sum(values) / len(values) if values else math.nan


def _json_safe(value: object) -> object:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _read_rows(root: Path) -> list[dict[str, str]]:
    paths = sorted(root.rglob('telemetry.csv'))
    if not paths:
        raise ValueError(f'no telemetry.csv files under {root}')
    rows: list[dict[str, str]] = []
    for path in paths:
        with path.open(newline='', encoding='utf-8') as stream:
            for row in csv.DictReader(stream):
                row['_source_csv'] = str(path.resolve())
                rows.append(row)
    return rows


def _read_summaries(root: Path) -> list[dict[str, object]]:
    summaries: list[dict[str, object]] = []
    for path in sorted(root.rglob('summary.json')):
        if path.name != 'summary.json':
            continue
        try:
            item = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError):
            continue
        item['_source_summary'] = str(path.resolve())
        summaries.append(item)
    return summaries


def _measurement_row(row: dict[str, str]) -> bool:
    return (
        _finite(row.get('va_hold_measurement_active')) > 0.5
        and _finite(row.get('va_hold_measurement_complete')) <= 0.5
    )


def _phase(row: dict[str, str]) -> str:
    if _measurement_row(row):
        return 'measurement'
    if _finite(row.get('va_hold_measurement_complete')) > 0.5:
        return 'post_measurement_release'
    if row.get('schedule_mode') == 'gust_target':
        return row.get('gust_phase') or row.get('va_hold_phase') or 'unknown'
    return row.get('va_hold_phase') or 'unknown'


def _eta_c_valid(row: dict[str, str]) -> bool:
    return _finite(row.get('eta_c_valid')) > 0.5


def _value(row: dict[str, str], primary: str, fallback: str = '') -> float:
    value = _finite(row.get(primary))
    if math.isfinite(value) or not fallback:
        return value
    return _finite(row.get(fallback))


def _summary_bool(summary: dict[str, object], key: str, default: bool) -> bool:
    value = summary.get(key, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'pass'}
    return default


def _summary_float(summary: dict[str, object], key: str) -> float:
    return _finite(summary.get(key))


def _f3b_target_condition_result(
    summary: dict[str, object],
    *,
    maximum_mean_airspeed_error_mps: float,
    maximum_std_airspeed_mps: float,
    maximum_abs_airspeed_error_mps: float,
) -> dict[str, object]:
    """Return the F3-B instrumentation target-condition gate for one run.

    F3-B is an eta_C instrumentation qualification.  It must not inherit the
    stricter F1 "90% inside ±0.3 m/s" performance occupancy gate; the smoke
    only needs a clean fixed measurement window at the requested condition.
    """
    mean_error = abs(_summary_float(summary, 'va_hold_mean_airspeed_error_mps'))
    if not math.isfinite(mean_error):
        mean_airspeed = _summary_float(summary, 'va_hold_mean_airspeed_mps')
        target_airspeed = _summary_float(summary, 'va_target_config')
        if not math.isfinite(target_airspeed):
            target_airspeed = _summary_float(summary, 'va_target_mps')
        if math.isfinite(mean_airspeed) and math.isfinite(target_airspeed):
            mean_error = abs(mean_airspeed - target_airspeed)

    std_airspeed = _summary_float(summary, 'va_hold_std_airspeed_mps')
    max_abs_error = _summary_float(summary, 'va_hold_max_abs_airspeed_error_mps')
    checks = {
        'measurement_complete': _summary_bool(
            summary, 'va_hold_measurement_complete', False),
        'measurement_protocol_valid': _summary_bool(
            summary, 'va_hold_measurement_protocol_valid', True),
        'lambda_target_reached': _summary_bool(
            summary, 'lambda_target_reached', True),
        'no_primary_failure': str(
            summary.get('primary_failure_cause', 'none')).lower() == 'none',
        'no_va_hold_abort': str(
            summary.get('va_hold_abort_reason', 'none')).lower() == 'none',
        'no_failsafe': not _summary_bool(summary, 'any_failsafe', False),
        'mean_airspeed_error': (
            math.isfinite(mean_error)
            and mean_error <= maximum_mean_airspeed_error_mps
        ),
        'std_airspeed': (
            math.isfinite(std_airspeed)
            and std_airspeed <= maximum_std_airspeed_mps
        ),
        'max_abs_airspeed_error': (
            math.isfinite(max_abs_error)
            and max_abs_error <= maximum_abs_airspeed_error_mps
        ),
    }
    return {
        'source_summary': summary.get('_source_summary', ''),
        'pass': all(checks.values()),
        'checks': checks,
        'mean_airspeed_error_mps': mean_error,
        'std_airspeed_mps': std_airspeed,
        'max_abs_airspeed_error_mps': max_abs_error,
    }


def validate_eta_c_smoke(
    root: Path,
    *,
    minimum_valid_samples: int = 50,
    minimum_measurement_valid_fraction: float = 0.80,
    minimum_positive_available_moment_fraction: float = 0.95,
    maximum_low_q_gate: float = 0.05,
    minimum_measurement_q_gate: float = 0.90,
    maximum_mean_airspeed_error_mps: float = 0.30,
    maximum_std_airspeed_mps: float = 0.30,
    maximum_abs_airspeed_error_mps: float = 0.60,
) -> dict[str, object]:
    rows = _read_rows(root)
    summaries = _read_summaries(root)
    schema_versions = sorted({
        _finite(row.get('schema_version')) for row in rows
        if math.isfinite(_finite(row.get('schema_version')))
    })
    latest_schema = max(schema_versions) if schema_versions else math.nan
    measurement_rows = [row for row in rows if _measurement_row(row)]
    valid_rows = [row for row in rows if _eta_c_valid(row)]
    valid_measurement_rows = [
        row for row in measurement_rows if _eta_c_valid(row)
    ]

    qbar = [_value(row, 'qbar_selected_pa', 'eta_c_dynamic_pressure_pa') for row in valid_rows]
    lower_gate = _mean(
        _finite(row.get('eta_c_pressure_gate_lower_pa_config'))
        for row in rows
    )
    upper_gate = _mean(
        _finite(row.get('eta_c_pressure_gate_upper_pa_config'))
        for row in rows
    )
    low_q_rows = [
        row for row in valid_rows
        if _value(row, 'qbar_selected_pa', 'eta_c_dynamic_pressure_pa')
        <= lower_gate
    ]
    measurement_q_gate = _mean(
        _value(row, 'eta_c_q_gate', 'eta_c_pressure_gate')
        for row in valid_measurement_rows
    )
    low_q_gate_max = max((
        _value(row, 'eta_c_q_gate', 'eta_c_pressure_gate')
        for row in low_q_rows
        if math.isfinite(_value(row, 'eta_c_q_gate', 'eta_c_pressure_gate'))
    ), default=math.nan)

    requested = [
        _value(row, 'shadow_fw_pitch_moment_req_nm',
               'shadow_pitch_moment_increment_nm')
        for row in valid_rows
    ]
    available = [
        _value(row, 'moment_available_est_nm',
               'eta_c_available_increment_moment_nm')
        for row in valid_rows
    ]
    remaining = [
        _value(row, 'elevator_remaining_directional_rad',
               'eta_c_remaining_elevator_angle_rad')
        for row in valid_rows
    ]
    eta_c = [_finite(row.get('eta_c')) for row in valid_rows]
    eta_c_raw = [
        _value(row, 'eta_c_raw', 'eta_c_moment_margin')
        for row in valid_rows
    ]
    elevator_actual = [
        _value(row, 'elevator_actual_angle_rad',
               'elevator_actual_angle_gt_rad')
        for row in valid_rows
    ]
    elevator_pitch_moment_gt_rows = [
        row for row in rows
        if _finite(row.get('elevator_aero_wrench_gt_valid')) > 0.5
        and math.isfinite(_finite(row.get('elevator_pitch_moment_gt_nm')))
    ]
    elevator_pitch_moment_gt_measurement_rows = [
        row for row in measurement_rows
        if _finite(row.get('elevator_aero_wrench_gt_valid')) > 0.5
        and math.isfinite(_finite(row.get('elevator_pitch_moment_gt_nm')))
    ]
    elevator_pitch_moment_gt = [
        _finite(row.get('elevator_pitch_moment_gt_nm'))
        for row in elevator_pitch_moment_gt_rows
    ]
    elevator_pitch_moment_gt_measurement = [
        _finite(row.get('elevator_pitch_moment_gt_nm'))
        for row in elevator_pitch_moment_gt_measurement_rows
    ]
    modifies_blending_values = [
        _finite(row.get('eta_c_modifies_px4_blending')) for row in rows
    ]

    phase_counts: dict[str, int] = {}
    for row in valid_rows:
        phase = _phase(row)
        phase_counts[phase] = phase_counts.get(phase, 0) + 1

    target_condition_results = [
        _f3b_target_condition_result(
            summary,
            maximum_mean_airspeed_error_mps=maximum_mean_airspeed_error_mps,
            maximum_std_airspeed_mps=maximum_std_airspeed_mps,
            maximum_abs_airspeed_error_mps=maximum_abs_airspeed_error_mps,
        )
        for summary in summaries
    ]
    target_condition_established = (
        bool(target_condition_results)
        and all(bool(item['pass']) for item in target_condition_results)
    )
    protocol_valid = all(
        summary.get('f1_attempt_quality', 'valid') == 'valid'
        for summary in summaries
        if 'f1_attempt_quality' in summary
    )

    checks = {
        'schema_ge_19': math.isfinite(latest_schema) and latest_schema >= 19.0,
        'target_condition_established': target_condition_established,
        'protocol_valid': protocol_valid,
        'minimum_eta_c_valid_samples': len(valid_rows) >= minimum_valid_samples,
        'measurement_eta_c_valid_fraction': (
            len(valid_measurement_rows) / len(measurement_rows)
            if measurement_rows else 0.0
        ) >= minimum_measurement_valid_fraction,
        'shadow_moment_request_finite': all(
            math.isfinite(value) for value in requested
        ) and bool(requested),
        'elevator_actual_finite': all(
            math.isfinite(value) for value in elevator_actual
        ) and bool(elevator_actual),
        'available_moment_positive': _fraction(
            value > 0.0 for value in available if math.isfinite(value)
        ) >= minimum_positive_available_moment_fraction,
        'directional_remaining_nonnegative': all(
            value >= 0.0 for value in remaining if math.isfinite(value)
        ) and bool(remaining),
        'low_q_gate_near_zero': (
            len(low_q_rows) > 0
            and math.isfinite(low_q_gate_max)
            and low_q_gate_max <= maximum_low_q_gate
        ),
        'measurement_q_gate_high': (
            math.isfinite(measurement_q_gate)
            and measurement_q_gate >= minimum_measurement_q_gate
        ),
        'eta_c_read_only': all(
            value == 0.0 for value in modifies_blending_values
            if math.isfinite(value)
        ) and bool(modifies_blending_values),
    }
    return {
        'experiment': 'F3B_eta_C_instrumentation_smoke',
        'root': str(root.resolve()),
        'schema_versions': schema_versions,
        'telemetry_sample_count': len(rows),
        'measurement_sample_count': len(measurement_rows),
        'eta_c_valid_sample_count': len(valid_rows),
        'eta_c_valid_measurement_sample_count': len(valid_measurement_rows),
        'measurement_eta_c_valid_fraction': (
            len(valid_measurement_rows) / len(measurement_rows)
            if measurement_rows else math.nan
        ),
        'low_q_sample_count': len(low_q_rows),
        'low_q_gate_max': low_q_gate_max,
        'measurement_q_gate_mean': measurement_q_gate,
        'qbar_selected_mean_pa': _mean(qbar),
        'qbar_selected_measurement_mean_pa': _mean(
            _value(row, 'qbar_selected_pa', 'eta_c_dynamic_pressure_pa')
            for row in valid_measurement_rows
        ),
        'f3b_target_condition_thresholds': {
            'maximum_mean_airspeed_error_mps': maximum_mean_airspeed_error_mps,
            'maximum_std_airspeed_mps': maximum_std_airspeed_mps,
            'maximum_abs_airspeed_error_mps': maximum_abs_airspeed_error_mps,
        },
        'f3b_target_condition_results': target_condition_results,
        'shadow_fw_pitch_moment_req_mean_nm': _mean(requested),
        'shadow_fw_pitch_moment_req_abs_mean_nm': _mean(
            abs(value) for value in requested
        ),
        'moment_available_est_mean_nm': _mean(available),
        'elevator_remaining_directional_mean_rad': _mean(remaining),
        'elevator_actual_angle_mean_rad': _mean(elevator_actual),
        'elevator_pitch_moment_gt_valid_sample_count': len(
            elevator_pitch_moment_gt_rows),
        'elevator_pitch_moment_gt_valid_measurement_sample_count': len(
            elevator_pitch_moment_gt_measurement_rows),
        'elevator_pitch_moment_gt_mean_nm': _mean(elevator_pitch_moment_gt),
        'elevator_pitch_moment_gt_measurement_mean_nm': _mean(
            elevator_pitch_moment_gt_measurement),
        'eta_c_raw_mean': _mean(eta_c_raw),
        'eta_c_mean': _mean(eta_c),
        'eta_c_measurement_mean': _mean(
            _finite(row.get('eta_c')) for row in valid_measurement_rows
        ),
        'phase_sample_counts': dict(sorted(phase_counts.items())),
        'checks': checks,
        'f3b_eta_c_instrumentation_smoke_pass': all(checks.values()),
    }


def write_eta_c_smoke(payload: dict[str, object], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    safe = _json_safe(payload)
    (output_dir / 'f3b_eta_c_smoke.json').write_text(
        json.dumps(safe, indent=2, allow_nan=False) + '\n',
        encoding='utf-8',
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest='command', required=True)
    smoke = subparsers.add_parser('smoke')
    smoke.add_argument('root', type=Path)
    smoke.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    if args.command == 'smoke':
        payload = validate_eta_c_smoke(args.root)
        write_eta_c_smoke(payload, args.output_dir)
        print(json.dumps(_json_safe(payload), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
