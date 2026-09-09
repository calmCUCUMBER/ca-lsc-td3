"""Build and audit the leakage-safe F5 predictive-utility dataset.

F5 asks whether the physically validated capability features ``eta_L`` and
``eta_C`` add predictive information beyond airspeed for the *executed*
transition allocation.  Labels are therefore computed only from future
physical/safety telemetry; capability features never enter the label.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


REQUIRED_COLUMNS = (
    'time_s',
    'selected_airspeed_mps',
    'lambda_exec',
    'eta_l',
    'eta_c',
    'alpha_est_rad',
    'altitude_error_m',
    'vz_up_mps',
    'failsafe',
)

HARD_ABORT_REASONS = {
    'va_hold_altitude_runaway',
    'va_hold_vertical_speed_runaway',
    'airspeed_hold_runaway',
    'px4_failsafe',
    'altitude_runaway',
    'aoa_limit',
    'descent_rate_limit',
    'pitch_rate_limit',
}

ELIGIBLE_PHASES = {
    'airspeed_settle',
    'lambda_settle',
    'measurement',
    'gust_baseline',
    'gust_exposure',
    'gust_recovery',
    'hard_abort_prefix',
}

DATASET_FIELDS = (
    'run_id', 'run_group', 'condition_id', 'condition_group',
    'source_experiment', 'source_csv', 'schema_version', 'time_s',
    'va_mps', 'eta_l', 'eta_c', 'lambda_exec', 'label_safe_2s',
    'label_reason', 'mass_scale', 'actual_model_mass_kg',
    'gust_amplitude_mps', 'wind_model', 'alpha_rad',
    'altitude_error_m', 'vz_up_mps', 'phase',
)


@dataclass(frozen=True)
class F5LabelConfig:
    """Frozen F5 label parameters, inherited from the flight protocol."""

    horizon_s: float = 2.0
    sample_period_s: float = 0.1
    maximum_gap_s: float = 0.5
    minimum_channel_coverage: float = 0.9
    alpha_hard_limit_rad: float = 0.3391428111
    altitude_error_hard_limit_m: float = 5.0
    vertical_speed_hard_limit_mps: float = 2.0

    def validate(self) -> None:
        values = (
            self.horizon_s,
            self.sample_period_s,
            self.maximum_gap_s,
            self.alpha_hard_limit_rad,
            self.altitude_error_hard_limit_m,
            self.vertical_speed_hard_limit_mps,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError('F5 label limits and durations must be positive')
        if not 0.0 < self.minimum_channel_coverage <= 1.0:
            raise ValueError('minimum_channel_coverage must be in (0, 1]')


def _finite(value: object) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return math.nan
    return number if math.isfinite(number) else math.nan


def _flag(row: dict[str, str], key: str) -> bool:
    return _finite(row.get(key)) > 0.5


def _text(row: dict[str, str], key: str, fallback: str = '') -> str:
    value = str(row.get(key, '') or '').strip()
    return value or fallback


def _json_safe(value: object) -> object:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _effective_phase(row: dict[str, str]) -> str:
    if _flag(row, 'va_hold_measurement_complete'):
        return 'post_measurement_release'
    if _flag(row, 'va_hold_measurement_active'):
        return 'measurement'
    schedule = _text(row, 'schedule_mode')
    gust = _text(row, 'gust_phase')
    if schedule == 'gust_target' and gust not in {'', 'inactive'}:
        return gust
    return _text(row, 'va_hold_phase', 'unknown')


def _feature_valid(row: dict[str, str]) -> bool:
    return (
        _flag(row, 'eta_l_valid')
        and _flag(row, 'eta_c_valid')
        and all(math.isfinite(_finite(row.get(key))) for key in (
            'selected_airspeed_mps', 'lambda_exec', 'eta_l', 'eta_c',
        ))
    )


def _hard_violation_reasons(
    row: dict[str, str], config: F5LabelConfig,
) -> list[str]:
    reasons: list[str] = []
    if _flag(row, 'failsafe'):
        reasons.append('failsafe')
    alpha = _finite(row.get('alpha_est_rad'))
    if _flag(row, 'alpha_valid') and math.isfinite(alpha):
        if abs(alpha) > config.alpha_hard_limit_rad:
            reasons.append('aoa_hard_limit')
    altitude_error = _finite(row.get('altitude_error_m'))
    if (
        math.isfinite(altitude_error)
        and abs(altitude_error) > config.altitude_error_hard_limit_m
    ):
        reasons.append('altitude_error_hard_limit')
    vertical_speed = _finite(row.get('vz_up_mps'))
    if (
        math.isfinite(vertical_speed)
        and abs(vertical_speed) > config.vertical_speed_hard_limit_mps
    ):
        reasons.append('vertical_speed_hard_limit')
    abort_reason = _text(row, 'va_hold_abort_reason', 'none')
    if abort_reason in HARD_ABORT_REASONS:
        reasons.append(abort_reason)
    return sorted(set(reasons))


def discover_telemetry(inputs: Iterable[Path]) -> list[tuple[Path, Path]]:
    """Return unique ``(csv, owning_input_root)`` pairs.

    File-level symlinks used by the F3-B2 merged archive resolve to their
    source path, so passing both an original and merged root cannot silently
    duplicate a trajectory.
    """
    discovered: dict[Path, tuple[Path, Path]] = {}
    for raw_root in inputs:
        root = raw_root.expanduser().resolve()
        if root.is_file() and root.name == 'telemetry.csv':
            paths = [root]
            owner = root.parent
        elif root.is_dir():
            paths = sorted(root.rglob('telemetry.csv'))
            owner = root
        else:
            continue
        for path in paths:
            resolved = path.resolve()
            discovered.setdefault(resolved, (resolved, owner))
    return list(discovered.values())


def _read_csv(path: Path) -> tuple[list[dict[str, str]], list[str]]:
    with path.open(newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        return list(reader), list(reader.fieldnames or [])


def _read_summary(path: Path) -> dict[str, object]:
    summary = path.parent / 'summary.json'
    if not summary.exists():
        return {}
    try:
        return json.loads(summary.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        return {}


def _first_finite(
    rows: list[dict[str, str]], summary: dict[str, object], keys: tuple[str, ...],
) -> float:
    for key in keys:
        value = _finite(summary.get(key))
        if math.isfinite(value):
            return value
        for row in rows:
            value = _finite(row.get(key))
            if math.isfinite(value):
                return value
    return math.nan


def _first_text(
    rows: list[dict[str, str]], summary: dict[str, object], keys: tuple[str, ...],
    fallback: str,
) -> str:
    for key in keys:
        raw_value = summary.get(key, '')
        if not isinstance(raw_value, (list, tuple, dict)):
            value = str(raw_value or '').strip()
            if value:
                return value
        for row in rows:
            value = _text(row, key)
            if value:
                return value
    return fallback


def _diagnostic_intervention_reason(rows: list[dict[str, str]]) -> str:
    """Exclude F3 identification injections from predictive-utility data.

    These runs remain part of the data-availability audit, but injected shadow
    demand, diagnostic elevator bias, or physical dither must not become F5
    evidence about naturally occurring transition capability.
    """
    modes = {
        _text(row, 'f3b2_test_mode') for row in rows
        if _text(row, 'f3b2_test_mode') not in {'', 'none'}
    }
    if modes:
        return 'f3b2_diagnostic_injection:' + ','.join(sorted(modes))
    if any(
        math.isfinite(_finite(row.get('elevator_id_dither_scale_command')))
        and abs(_finite(row.get('elevator_id_dither_scale_command'))) > 1.0e-9
        for row in rows
    ):
        return 'f3b1_elevator_identification_dither'
    return 'none'


def _number_token(value: float) -> str:
    return 'na' if not math.isfinite(value) else f'{value:.6g}'


def _run_metadata(
    path: Path,
    owner: Path,
    rows: list[dict[str, str]],
    summary: dict[str, object],
) -> dict[str, object]:
    try:
        relative = path.parent.relative_to(owner)
        run_id = f'{owner.name}/{relative.as_posix()}'
    except ValueError:
        run_id = f'{owner.name}/{path.parent.name}'
    source = owner.name
    condition_id = _first_text(
        rows, summary, ('condition_id',), path.parent.parent.name,
    )
    va_target = _first_finite(
        rows, summary, ('va_target_config', 'va_target_mps'),
    )
    lambda_target = _first_finite(
        rows, summary, ('lambda_target_config', 'lambda_target'),
    )
    mass_scale = _first_finite(rows, summary, ('mass_scale_config',))
    actual_mass = _first_finite(
        rows, summary, ('actual_model_mass_kg_config',),
    )
    gust = _first_finite(
        rows, summary, ('gust_amplitude_mps_config', 'gust_amplitude_mps'),
    )
    wind_e = _first_finite(rows, summary, ('wind_cmd_e_enu_mps',))
    wind_n = _first_finite(rows, summary, ('wind_cmd_n_enu_mps',))
    wind_u = _first_finite(rows, summary, ('wind_cmd_u_enu_mps',))
    wind_model = _first_text(
        rows, summary, ('wind_model_config',), 'none',
    )
    test_mode = _first_text(rows, summary, ('f3b2_test_mode',), 'none')
    stage = _first_finite(rows, summary, ('f3b2_stage_index',))
    condition_group = '|'.join((
        f'va={_number_token(va_target)}',
        f'lambda={_number_token(lambda_target)}',
        f'actual_mass={_number_token(actual_mass)}',
        f'gust={_number_token(gust)}',
        f'wind_e={_number_token(wind_e)}',
        f'wind_n={_number_token(wind_n)}',
        f'wind_u={_number_token(wind_u)}',
        f'wind={wind_model}',
        f'mode={test_mode}',
        f'stage={_number_token(stage)}',
    ))
    return {
        'run_id': run_id,
        'run_group': str(path.resolve()),
        'condition_id': condition_id,
        'condition_group': condition_group,
        'source_experiment': source,
        'source_csv': str(path.resolve()),
        'mass_scale': mass_scale,
        'actual_model_mass_kg': actual_mass,
        'gust_amplitude_mps': gust,
        'wind_model': wind_model,
    }


def _coverage(rows: list[dict[str, str]], key: str) -> float:
    if not rows:
        return 0.0
    return sum(math.isfinite(_finite(row.get(key))) for row in rows) / len(rows)


def _future_label(
    rows: list[dict[str, str]],
    start_index: int,
    config: F5LabelConfig,
) -> tuple[int | None, str]:
    start_time = _finite(rows[start_index].get('time_s'))
    if not math.isfinite(start_time):
        return None, 'invalid_start_time'
    target_time = start_time + config.horizon_s
    future: list[dict[str, str]] = []
    reached_horizon = False
    for row in rows[start_index:]:
        time_s = _finite(row.get('time_s'))
        if not math.isfinite(time_s) or time_s < start_time:
            continue
        future.append(row)
        if time_s >= target_time:
            reached_horizon = True
            break
    if not future:
        return None, 'no_future_rows'

    violations = sorted({
        reason
        for row in future
        for reason in _hard_violation_reasons(row, config)
    })
    if violations:
        return 0, ';'.join(violations)
    if not reached_horizon:
        return None, 'incomplete_2s_horizon'

    times = [_finite(row.get('time_s')) for row in future]
    gaps = [right - left for left, right in zip(times, times[1:])]
    if any(gap <= 0.0 or gap > config.maximum_gap_s for gap in gaps):
        return None, 'telemetry_gap_in_horizon'
    for key in ('altitude_error_m', 'vz_up_mps', 'failsafe'):
        if _coverage(future, key) < config.minimum_channel_coverage:
            return None, f'insufficient_{key}_coverage'
    return 1, 'safe_2s'


def build_f5_dataset(
    inputs: Iterable[Path],
    *,
    config: F5LabelConfig | None = None,
) -> tuple[list[dict[str, object]], dict[str, object]]:
    config = config or F5LabelConfig()
    config.validate()
    dataset: list[dict[str, object]] = []
    run_audit: list[dict[str, object]] = []
    telemetry = discover_telemetry(inputs)

    for path, owner in telemetry:
        rows, fields = _read_csv(path)
        summary = _read_summary(path)
        metadata = _run_metadata(path, owner, rows, summary)
        intervention_reason = _diagnostic_intervention_reason(rows)
        eligible_indices = [
            index for index, row in enumerate(rows)
            if _effective_phase(row) in ELIGIBLE_PHASES
            and math.isfinite(_finite(row.get('time_s')))
        ]
        eta_l_covered = sum(
            _flag(rows[index], 'eta_l_valid')
            and math.isfinite(_finite(rows[index].get('eta_l')))
            for index in eligible_indices
        )
        eta_c_covered = sum(
            _flag(rows[index], 'eta_c_valid')
            and math.isfinite(_finite(rows[index].get('eta_c')))
            for index in eligible_indices
        )
        feature_indices = (
            [index for index in eligible_indices if _feature_valid(rows[index])]
            if intervention_reason == 'none' else []
        )
        selected = 0
        unavailable_horizon = 0
        labels = {0: 0, 1: 0}
        last_sample_time = -math.inf
        schema = _first_finite(rows, summary, ('schema_version', 'telemetry_schema_version'))
        for index in feature_indices:
            row = rows[index]
            time_s = _finite(row.get('time_s'))
            if time_s - last_sample_time + 1.0e-9 < config.sample_period_s:
                continue
            last_sample_time = time_s
            label, reason = _future_label(rows, index, config)
            if label is None:
                unavailable_horizon += 1
                continue
            labels[label] += 1
            selected += 1
            dataset.append({
                **metadata,
                'schema_version': schema,
                'time_s': time_s,
                'va_mps': _finite(row.get('selected_airspeed_mps')),
                'eta_l': _finite(row.get('eta_l')),
                'eta_c': _finite(row.get('eta_c')),
                'lambda_exec': _finite(row.get('lambda_exec')),
                'label_safe_2s': label,
                'label_reason': reason,
                'alpha_rad': _finite(row.get('alpha_est_rad')),
                'altitude_error_m': _finite(row.get('altitude_error_m')),
                'vz_up_mps': _finite(row.get('vz_up_mps')),
                'phase': _effective_phase(row),
            })
        denominator = len(eligible_indices)
        missing = [key for key in REQUIRED_COLUMNS if key not in fields]
        run_audit.append({
            **metadata,
            'schema_version': schema,
            'row_count': len(rows),
            'eligible_row_count': denominator,
            'feature_complete_row_count': len(feature_indices),
            'samples_with_2s_horizon': selected,
            'safe_sample_count': labels[1],
            'unsafe_sample_count': labels[0],
            'horizon_unavailable_candidate_count': unavailable_horizon,
            'eta_l_coverage': eta_l_covered / denominator if denominator else 0.0,
            'eta_c_coverage': eta_c_covered / denominator if denominator else 0.0,
            'missing_required_columns': missing,
            'scientific_exclusion_reason': intervention_reason,
            'usable': selected > 0,
        })

    safe = sum(int(row['label_safe_2s']) == 1 for row in dataset)
    unsafe = len(dataset) - safe
    usable_runs = [row for row in run_audit if bool(row['usable'])]
    usable_conditions = {str(row['condition_group']) for row in usable_runs}
    unsafe_conditions = {
        str(row['condition_group']) for row in usable_runs
        if int(row['unsafe_sample_count']) > 0
    }
    safe_conditions = {
        str(row['condition_group']) for row in usable_runs
        if int(row['safe_sample_count']) > 0
    }
    total_eligible = sum(int(row['eligible_row_count']) for row in run_audit)
    eta_l_weighted = sum(
        float(row['eta_l_coverage']) * int(row['eligible_row_count'])
        for row in run_audit
    )
    eta_c_weighted = sum(
        float(row['eta_c_coverage']) * int(row['eligible_row_count'])
        for row in run_audit
    )
    audit = {
        'analysis_type': 'f5_data_audit',
        'label_definition': {
            'candidate': 'executed lambda at telemetry time t',
            'safe': 'no frozen hard transition violation in [t,t+2s]',
            'uses_eta_l_or_eta_c': False,
            **config.__dict__,
            'hard_abort_reasons': sorted(HARD_ABORT_REASONS),
        },
        'total_runs': len(run_audit),
        'usable_runs': len(usable_runs),
        'usable_conditions': len(usable_conditions),
        'conditions_with_safe_samples': len(safe_conditions),
        'conditions_with_unsafe_samples': len(unsafe_conditions),
        'samples_with_2s_horizon': len(dataset),
        'eta_l_coverage': eta_l_weighted / total_eligible if total_eligible else 0.0,
        'eta_c_coverage': eta_c_weighted / total_eligible if total_eligible else 0.0,
        'class_balance': {
            'safe': safe,
            'unsafe': unsafe,
            'safe_fraction': safe / len(dataset) if dataset else math.nan,
            'unsafe_fraction': unsafe / len(dataset) if dataset else math.nan,
        },
        'predictive_utility_ready': bool(
            len(usable_conditions) >= 5
            and len(safe_conditions) >= 5
            and len(unsafe_conditions) >= 5
        ),
        'runs': run_audit,
    }
    return dataset, audit


def write_f5_outputs(
    dataset: list[dict[str, object]],
    audit: dict[str, object],
    output_dir: Path,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / 'f5_dataset.csv').open(
        'w', newline='', encoding='utf-8',
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=DATASET_FIELDS)
        writer.writeheader()
        for row in dataset:
            writer.writerow({
                key: '' if isinstance(value, float) and not math.isfinite(value) else value
                for key, value in row.items() if key in DATASET_FIELDS
            })
    (output_dir / 'f5_data_audit.json').write_text(
        json.dumps(_json_safe(audit), indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    run_fields = (
        'run_id', 'condition_group', 'source_experiment', 'source_csv',
        'schema_version', 'row_count', 'eligible_row_count',
        'feature_complete_row_count', 'samples_with_2s_horizon',
        'safe_sample_count', 'unsafe_sample_count',
        'horizon_unavailable_candidate_count', 'eta_l_coverage',
        'eta_c_coverage', 'missing_required_columns', 'usable',
        'scientific_exclusion_reason',
    )
    with (output_dir / 'f5_data_audit_runs.csv').open(
        'w', newline='', encoding='utf-8',
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=run_fields)
        writer.writeheader()
        for row in audit['runs']:
            item = {key: row.get(key, '') for key in run_fields}
            item['missing_required_columns'] = ';'.join(
                row.get('missing_required_columns', []))
            writer.writerow(item)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inputs', nargs='+', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--horizon-s', type=float, default=2.0)
    parser.add_argument('--sample-period-s', type=float, default=0.1)
    parser.add_argument('--alpha-hard-limit-rad', type=float, default=0.3391428111)
    parser.add_argument('--altitude-hard-limit-m', type=float, default=5.0)
    parser.add_argument('--vertical-speed-hard-limit-mps', type=float, default=2.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    config = F5LabelConfig(
        horizon_s=args.horizon_s,
        sample_period_s=args.sample_period_s,
        alpha_hard_limit_rad=args.alpha_hard_limit_rad,
        altitude_error_hard_limit_m=args.altitude_hard_limit_m,
        vertical_speed_hard_limit_mps=args.vertical_speed_hard_limit_mps,
    )
    dataset, audit = build_f5_dataset(args.inputs, config=config)
    write_f5_outputs(dataset, audit, args.output_dir)
    print(json.dumps(_json_safe({
        key: audit[key] for key in (
            'total_runs', 'usable_runs', 'usable_conditions',
            'samples_with_2s_horizon', 'eta_l_coverage', 'eta_c_coverage',
            'conditions_with_safe_samples', 'conditions_with_unsafe_samples',
            'class_balance', 'predictive_utility_ready',
        )
    }), indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
