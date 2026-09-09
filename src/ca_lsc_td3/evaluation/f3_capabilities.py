"""F3 physical validation and outcome-space diagnostics for eta_L/eta_C."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import fmean
from typing import Iterable


OUTCOME_COLORS = {
    'N': '#2ca02c',
    'B': '#f2c94c',
    'I': '#f2994a',
    'X': '#d62728',
}

PHYSICAL_TO_SYMBOL = {
    'nominal_feasible': 'N',
    'boundary_feasible': 'B',
    'nonconvergent': 'I',
    'physical_unsafe': 'X',
    'N': 'N', 'B': 'B', 'I': 'I', 'X': 'X',
}


def _finite(value: object) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def _mean(values: Iterable[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return fmean(finite) if finite else math.nan


def _percentile(values: Iterable[float], fraction: float) -> float:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return math.nan
    position = (len(finite) - 1) * fraction
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return finite[lower]
    weight = position - lower
    return finite[lower] * (1.0 - weight) + finite[upper] * weight


def _rmse(estimates: list[float], truth: list[float]) -> float:
    if not estimates or len(estimates) != len(truth):
        return math.nan
    return math.sqrt(fmean(
        (estimate - actual) ** 2
        for estimate, actual in zip(estimates, truth)
    ))


def _r_squared(estimates: list[float], truth: list[float]) -> float:
    if len(estimates) < 2 or len(estimates) != len(truth):
        return math.nan
    center = fmean(truth)
    total = sum((actual - center) ** 2 for actual in truth)
    if total <= 1.0e-12:
        return math.nan
    residual = sum(
        (estimate - actual) ** 2
        for estimate, actual in zip(estimates, truth)
    )
    return 1.0 - residual / total


def _json_safe(value: object) -> object:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _effective_phase(row: dict[str, str]) -> str:
    """Return the scientific phase without mixing inactive gust labels in.

    Non-gust schema-17 rows still contain ``gust_phase=inactive``.  Treating
    that non-empty string as the primary phase masks the real Va-hold state.
    A completed fixed window is split from the active measurement so the PX4
    release tail cannot be presented as measurement data.
    """
    schedule_mode = (row.get('schedule_mode') or '').strip()
    if schedule_mode == 'gust_target':
        return (
            (row.get('gust_phase') or '').strip()
            or (row.get('va_hold_phase') or '').strip()
            or 'unknown'
        )
    if _finite(row.get('va_hold_measurement_complete')) > 0.5:
        return 'post_measurement_release'
    if _finite(row.get('va_hold_measurement_active')) > 0.5:
        return 'measurement'
    return (row.get('va_hold_phase') or '').strip() or 'unknown'


def _first_nonempty(rows: list[dict[str, str]], key: str, fallback: str) -> str:
    for row in rows:
        value = (row.get(key) or '').strip()
        if value:
            return value
    return fallback


def _protocol_status(path: Path) -> tuple[bool, str]:
    """Return whether a telemetry file is a valid target-condition attempt."""
    summary = path.parent / 'summary.json'
    if not summary.exists():
        return True, 'summary_not_available'
    try:
        item = json.loads(summary.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        return False, 'summary_unreadable'
    for quality_key, reason_key in (
        ('f1_attempt_quality', 'f1_attempt_quality_reason'),
        ('f2b_attempt_quality', 'f2b_attempt_quality_reason'),
    ):
        quality = str(item.get(quality_key, '')).strip()
        if quality:
            reason = str(item.get(reason_key, 'none')).strip() or 'none'
            if quality != 'valid' and reason == 'none':
                reason = 'protocol_invalid'
            return quality == 'valid', reason
    primary = str(item.get('primary_failure_cause', 'none')).strip()
    if primary not in {'', 'none'} and not bool(item.get('f1_measurement_pass', True)):
        return False, primary
    return True, 'legacy_summary_without_quality'


def _force_model_valid(row: dict[str, str]) -> bool:
    """Mask samples suitable for eta_L force-regression validation.

    Schema-18 logs write this explicitly.  Older archived logs fall back to
    the schema-17 gate so they remain auditable without pretending they had a
    longitudinal-domain check.
    """
    explicit = row.get('eta_l_force_model_valid')
    if explicit not in {None, ''}:
        return _finite(explicit) > 0.5
    return (
        _finite(row.get('eta_l_valid')) > 0.5
        and _finite(row.get('selected_airspeed_mps')) >= 5.0
    )


def _eta_l_available(row: dict[str, str]) -> bool:
    explicit = row.get('eta_l_available')
    if explicit not in {None, ''}:
        return _finite(explicit) > 0.5
    return _finite(row.get('eta_l_valid')) > 0.5


SCIENTIFIC_FORCE_PHASES = {
    'airspeed_settle',
    'lambda_settle',
    'measurement',
    'post_measurement_release',
    'gust_baseline',
    'gust_exposure',
    'gust_recovery',
    'hard_abort_prefix',
}


def _scientific_force_phase(row: dict[str, str]) -> bool:
    return _effective_phase(row) in SCIENTIFIC_FORCE_PHASES


def _validation_rows(
    path: Path,
) -> tuple[list[dict[str, str]], int, int, int, int, list[dict[str, str]]]:
    selected: list[dict[str, str]] = []
    force_model_valid_rows = 0
    force_model_valid_raw_rows = 0
    scientific_phase_excluded_rows = 0
    eta_l_available_rows = 0
    all_csv_rows: list[dict[str, str]] = []
    with path.open(newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            all_csv_rows.append(row)
            if _eta_l_available(row):
                eta_l_available_rows += 1
            if not _force_model_valid(row):
                continue
            force_model_valid_raw_rows += 1
            if not _scientific_force_phase(row):
                scientific_phase_excluded_rows += 1
                continue
            force_model_valid_rows += 1
            if (
                _finite(row.get('wing_lift_gt_valid')) > 0.5
                and all(math.isfinite(_finite(row.get(key))) for key in (
                    'wing_vertical_support_est_n',
                    'wing_lift_gt_vertical_enu_n',
                    'eta_l', 'eta_l_gt',
                ))
            ):
                selected.append(row)
    return (
        selected,
        force_model_valid_rows,
        force_model_valid_raw_rows,
        scientific_phase_excluded_rows,
        eta_l_available_rows,
        all_csv_rows,
    )


def _metrics(
    rows: list[dict[str, str]],
    *,
    relative_force_floor_n: float,
) -> dict[str, object]:
    estimate = [_finite(row['wing_vertical_support_est_n']) for row in rows]
    truth = [_finite(row['wing_lift_gt_vertical_enu_n']) for row in rows]
    eta_estimate = [_finite(row['eta_l']) for row in rows]
    eta_truth = [_finite(row['eta_l_gt']) for row in rows]
    errors = [predicted - actual for predicted, actual in zip(estimate, truth)]
    absolute = [abs(value) for value in errors]
    relative = [
        abs(error) / abs(actual)
        for error, actual in zip(errors, truth)
        if abs(actual) >= relative_force_floor_n
    ]
    mean_abs_truth = _mean(abs(value) for value in truth)
    force_rmse = _rmse(estimate, truth)
    phases: dict[str, int] = {}
    for row in rows:
        phase = _effective_phase(row)
        phases[phase] = phases.get(phase, 0) + 1
    return {
        'sample_count': len(rows),
        'force_gt_mean_n': _mean(truth),
        'force_estimate_mean_n': _mean(estimate),
        'force_bias_n': _mean(errors),
        'force_mae_n': _mean(absolute),
        'force_rmse_n': force_rmse,
        'force_nrmse_by_mean_abs_gt': (
            force_rmse / mean_abs_truth
            if math.isfinite(force_rmse) and mean_abs_truth > 1.0e-9
            else math.nan
        ),
        'force_r_squared': _r_squared(estimate, truth),
        'force_relative_error_sample_count': len(relative),
        'force_absolute_relative_error_median': _percentile(relative, 0.5),
        'force_absolute_relative_error_p95': _percentile(relative, 0.95),
        'eta_l_rmse': _rmse(eta_estimate, eta_truth),
        'eta_l_r_squared': _r_squared(eta_estimate, eta_truth),
        'phase_sample_counts': dict(sorted(phases.items())),
    }


def _diagnostic_means(
    rows: list[dict[str, str]], *, suffix: str = '',
) -> dict[str, float]:
    """Return run-level wind/body-flow diagnostics.

    Schema-18 separates online eta_L availability from samples that are valid
    for longitudinal force-model regression.  Keeping both summaries makes a
    wind smoke easy to interpret: the full available window tells us whether
    the wind field acted on the aircraft, while the force-valid subset tells us
    what was actually used for the eta_L-vs-Gazebo-lift comparison.
    """
    label = f'_{suffix}' if suffix else ''
    return {
        f'mean_yaw{label}_rad': _mean(
            _finite(row.get('yaw_rad')) for row in rows
        ),
        f'mean_wind_heading_parallel{label}_mps': _mean(
            _finite(row.get('wind_heading_parallel_mps')) for row in rows
        ),
        f'mean_wind_heading_cross{label}_mps': _mean(
            _finite(row.get('wind_heading_cross_mps')) for row in rows
        ),
        f'mean_vrel_body_u{label}_mps': _mean(
            _finite(row.get('vrel_body_u_mps')) for row in rows
        ),
        f'mean_vrel_body_v{label}_mps': _mean(
            _finite(row.get('vrel_body_v_mps')) for row in rows
        ),
        f'mean_vrel_body_w{label}_mps': _mean(
            _finite(row.get('vrel_body_w_mps')) for row in rows
        ),
        f'mean_beta{label}_rad': _mean(
            _finite(row.get('beta_est_rad')) for row in rows
        ),
        f'mean_abs_beta{label}_rad': _mean(
            abs(_finite(row.get('beta_est_rad'))) for row in rows
        ),
    }


def validate_eta_l(
    root: Path,
    *,
    minimum_samples: int = 200,
    minimum_gt_coverage: float = 0.90,
    minimum_r_squared: float = 0.90,
    maximum_nrmse: float = 0.20,
    maximum_median_relative_error: float = 0.20,
    relative_force_floor_n: float = 5.0,
    minimum_condition_samples: int = 30,
) -> tuple[dict[str, object], list[dict[str, str]]]:
    paths = sorted(root.rglob('telemetry.csv'))
    if not paths:
        raise ValueError(f'no telemetry.csv files under {root}')
    all_rows: list[dict[str, str]] = []
    run_results: list[dict[str, object]] = []
    condition_rows: dict[str, list[dict[str, str]]] = {}
    condition_eligible: dict[str, int] = {}
    operating_point_rows: dict[tuple[str, str, str, str], list[dict[str, str]]] = {}
    operating_point_eligible: dict[tuple[str, str, str, str], int] = {}
    eligible_total = 0
    force_model_valid_raw_total = 0
    scientific_phase_excluded_total = 0
    available_total = 0
    protocol_invalid_sample_count = 0
    for path in paths:
        (
            rows,
            eligible,
            raw_force_valid,
            scientific_phase_excluded,
            available,
            csv_rows,
        ) = _validation_rows(path)
        protocol_valid, protocol_reason = _protocol_status(path)
        condition_id = _first_nonempty(csv_rows, 'condition_id', path.parent.name)
        available_total += available
        force_model_valid_raw_total += raw_force_valid
        scientific_phase_excluded_total += scientific_phase_excluded
        if protocol_valid:
            eligible_total += eligible
            all_rows.extend(rows)
            condition_rows.setdefault(condition_id, []).extend(rows)
            condition_eligible[condition_id] = (
                condition_eligible.get(condition_id, 0) + eligible
            )
        else:
            protocol_invalid_sample_count += eligible
        eta_l_available_csv_rows = [
            row for row in csv_rows if _eta_l_available(row)
        ]
        eta_l_force_valid_csv_rows = [
            row for row in csv_rows
            if _force_model_valid(row) and _scientific_force_phase(row)
        ]
        point_key = (
            condition_id,
            _first_nonempty(csv_rows, 'va_target_config', 'unknown'),
            _first_nonempty(csv_rows, 'lambda_target_config', 'unknown'),
            _first_nonempty(csv_rows, 'gust_amplitude_mps_config', '0'),
        )
        if protocol_valid:
            operating_point_rows.setdefault(point_key, []).extend(rows)
            operating_point_eligible[point_key] = (
                operating_point_eligible.get(point_key, 0) + eligible
            )
        result = _metrics(rows, relative_force_floor_n=relative_force_floor_n)
        result.update({
            'source_csv': str(path.resolve()),
            'protocol_valid': protocol_valid,
            'protocol_invalid_reason': (
                'none' if protocol_valid else protocol_reason
            ),
            'used_in_validation': protocol_valid,
            'condition_id': condition_id,
            'schedule_mode': _first_nonempty(csv_rows, 'schedule_mode', 'unknown'),
            'mass_scale': _mean(_finite(row.get('mass_scale_config')) for row in csv_rows),
            'va_target_mps': _mean(_finite(row.get('va_target_config')) for row in csv_rows),
            'lambda_target': _mean(_finite(row.get('lambda_target_config')) for row in csv_rows),
            'gust_amplitude_mps': _mean(
                _finite(row.get('gust_amplitude_mps_config')) for row in csv_rows
            ),
            'eta_l_available_sample_count': available,
            'eta_l_force_model_valid_sample_count': eligible,
            'eta_l_force_model_valid_raw_sample_count': raw_force_valid,
            'eta_l_scientific_phase_excluded_sample_count': (
                scientific_phase_excluded
            ),
            'eta_l_valid_sample_count': eligible,
            'gt_coverage_fraction': len(rows) / eligible if eligible else 0.0,
            **_diagnostic_means(csv_rows),
            **_diagnostic_means(
                eta_l_available_csv_rows, suffix='available',
            ),
            **_diagnostic_means(
                eta_l_force_valid_csv_rows, suffix='force_valid',
            ),
        })
        run_results.append(result)
    combined = _metrics(
        all_rows, relative_force_floor_n=relative_force_floor_n
    )
    coverage = len(all_rows) / eligible_total if eligible_total else 0.0
    condition_results: list[dict[str, object]] = []
    for condition_id in sorted(condition_rows):
        rows = condition_rows[condition_id]
        eligible = condition_eligible[condition_id]
        result = _metrics(
            rows, relative_force_floor_n=relative_force_floor_n
        )
        result.update({
            'condition_id': condition_id,
            'eta_l_force_model_valid_sample_count': eligible,
            'eta_l_valid_sample_count': eligible,
            'gt_coverage_fraction': len(rows) / eligible if eligible else 0.0,
        })
        result['minimum_samples_pass'] = (
            len(rows) >= minimum_condition_samples
        )
        result['ground_truth_coverage_pass'] = (
            result['gt_coverage_fraction'] >= minimum_gt_coverage
        )
        result['force_nrmse_pass'] = (
            _finite(result['force_nrmse_by_mean_abs_gt']) <= maximum_nrmse
        )
        result['median_relative_error_pass'] = (
            _finite(result['force_absolute_relative_error_median'])
            <= maximum_median_relative_error
        )
        result['condition_validation_pass'] = all((
            result['minimum_samples_pass'],
            result['ground_truth_coverage_pass'],
            result['force_nrmse_pass'],
            result['median_relative_error_pass'],
        ))
        condition_results.append(result)

    operating_point_results: list[dict[str, object]] = []
    for point_key in sorted(
        operating_point_rows,
        key=lambda item: (
            item[0], _finite(item[1]), _finite(item[2]), _finite(item[3]),
        ),
    ):
        condition_id, va_text, lambda_text, gust_text = point_key
        rows = operating_point_rows[point_key]
        eligible = operating_point_eligible[point_key]
        result = _metrics(
            rows, relative_force_floor_n=relative_force_floor_n
        )
        result.update({
            'condition_id': condition_id,
            'va_target_mps': _finite(va_text),
            'lambda_target': _finite(lambda_text),
            'gust_amplitude_mps': _finite(gust_text),
            'eta_l_force_model_valid_sample_count': eligible,
            'eta_l_valid_sample_count': eligible,
            'gt_coverage_fraction': len(rows) / eligible if eligible else 0.0,
        })
        result['minimum_samples_pass'] = (
            len(rows) >= minimum_condition_samples
        )
        result['ground_truth_coverage_pass'] = (
            result['gt_coverage_fraction'] >= minimum_gt_coverage
        )
        result['force_nrmse_pass'] = (
            _finite(result['force_nrmse_by_mean_abs_gt']) <= maximum_nrmse
        )
        result['median_relative_error_pass'] = (
            _finite(result['force_absolute_relative_error_median'])
            <= maximum_median_relative_error
        )
        result['operating_point_validation_pass'] = all((
            result['minimum_samples_pass'],
            result['ground_truth_coverage_pass'],
            result['force_nrmse_pass'],
            result['median_relative_error_pass'],
        ))
        operating_point_results.append(result)

    phase_results: list[dict[str, object]] = []
    for phase in sorted({_effective_phase(row) for row in all_rows}):
        rows = [row for row in all_rows if _effective_phase(row) == phase]
        result = _metrics(
            rows, relative_force_floor_n=relative_force_floor_n
        )
        result['phase'] = phase
        phase_results.append(result)

    checks = {
        'minimum_samples': len(all_rows) >= minimum_samples,
        'ground_truth_coverage': coverage >= minimum_gt_coverage,
        'force_r_squared': (
            _finite(combined['force_r_squared']) >= minimum_r_squared
        ),
        'force_nrmse': (
            _finite(combined['force_nrmse_by_mean_abs_gt']) <= maximum_nrmse
        ),
        'median_relative_error': (
            _finite(combined['force_absolute_relative_error_median'])
            <= maximum_median_relative_error
        ),
        'all_runs_ground_truth_coverage': all(
            _finite(run['gt_coverage_fraction']) >= minimum_gt_coverage
            for run in run_results
            if bool(run.get('protocol_valid'))
        ),
        'all_conditions_valid': bool(condition_results) and all(
            bool(condition['condition_validation_pass'])
            for condition in condition_results
        ),
        'all_operating_points_valid': bool(operating_point_results) and all(
            bool(point['operating_point_validation_pass'])
            for point in operating_point_results
        ),
    }
    payload = {
        'experiment': 'F3A_eta_L_physical_validation',
        'root': str(root.resolve()),
        'ground_truth_definition': (
            'sum of the two exact main-wing lift vectors applied to Gazebo '
            'physics by ca_lsc::InstrumentedLiftDrag, world ENU z component'
        ),
        'estimate_definition': (
            '0.5*rho*selected_airspeed_mps^2*S*CL(alpha)*cos(gamma)*cos(phi)'
        ),
        'relative_error_force_floor_n': relative_force_floor_n,
        'thresholds': {
            'minimum_samples': minimum_samples,
            'minimum_gt_coverage': minimum_gt_coverage,
            'minimum_r_squared': minimum_r_squared,
            'maximum_nrmse': maximum_nrmse,
            'maximum_median_relative_error': maximum_median_relative_error,
            'minimum_condition_samples': minimum_condition_samples,
        },
        'telemetry_file_count': len(paths),
        'protocol_valid_telemetry_file_count': sum(
            bool(run.get('protocol_valid')) for run in run_results
        ),
        'protocol_invalid_telemetry_file_count': sum(
            not bool(run.get('protocol_valid')) for run in run_results
        ),
        'protocol_invalid_force_model_sample_count': (
            protocol_invalid_sample_count
        ),
        'eta_l_available_sample_count': available_total,
        'eta_l_force_model_valid_sample_count': eligible_total,
        'eta_l_force_model_valid_raw_sample_count': (
            force_model_valid_raw_total
        ),
        'eta_l_scientific_phase_excluded_sample_count': (
            scientific_phase_excluded_total
        ),
        'eta_l_valid_sample_count': eligible_total,
        'ground_truth_sample_count': len(all_rows),
        'ground_truth_coverage_fraction': coverage,
        'combined_metrics': combined,
        'checks': checks,
        'eta_l_physical_validation_pass': all(checks.values()),
        'conditions': condition_results,
        'operating_points': operating_point_results,
        'phases': phase_results,
        'runs': run_results,
    }
    return payload, all_rows


def _render_validation(
    payload: dict[str, object], rows: list[dict[str, str]], output_dir: Path,
) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    output_dir.mkdir(parents=True, exist_ok=True)
    truth = [_finite(row['wing_lift_gt_vertical_enu_n']) for row in rows]
    estimate = [_finite(row['wing_vertical_support_est_n']) for row in rows]
    airspeed = [_finite(row['selected_airspeed_mps']) for row in rows]
    figure, axes = plt.subplots(1, 2, figsize=(11, 4.6))
    scatter = axes[0].scatter(truth, estimate, c=airspeed, s=7, alpha=0.45)
    bounds = [min(truth + estimate), max(truth + estimate)]
    axes[0].plot(bounds, bounds, 'k--', linewidth=1, label='ideal')
    axes[0].set_xlabel(r'Gazebo applied main-wing $L_z^{GT}$ [N]')
    axes[0].set_ylabel(r'Estimated $\hat L_z$ [N]')
    axes[0].grid(alpha=0.25)
    axes[0].legend()
    figure.colorbar(scatter, ax=axes[0], label=r'$V_a$ [m/s]')
    errors = [predicted - actual for predicted, actual in zip(estimate, truth)]
    axes[1].scatter(airspeed, errors, s=7, alpha=0.45)
    axes[1].axhline(0.0, color='black', linestyle='--', linewidth=1)
    axes[1].set_xlabel(r'`selected_airspeed_mps` $V_a$ [m/s]')
    axes[1].set_ylabel(r'$\hat L_z-L_z^{GT}$ [N]')
    axes[1].grid(alpha=0.25)
    metrics = payload['combined_metrics']
    figure.suptitle(
        r'F3-A $eta_L$ physical validation: '
        f"$R^2$={_finite(metrics['force_r_squared']):.3f}, "
        f"RMSE={_finite(metrics['force_rmse_n']):.2f} N"
    )
    figure.tight_layout()
    figure.savefig(output_dir / 'f3a_eta_l_force_validation.png', dpi=180)
    plt.close(figure)


def write_eta_l_validation(
    payload: dict[str, object], rows: list[dict[str, str]], output_dir: Path,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    safe = _json_safe(payload)
    (output_dir / 'f3a_eta_l_validation.json').write_text(
        json.dumps(safe, indent=2, allow_nan=False) + '\n', encoding='utf-8'
    )
    metric_fields = (
        'sample_count', 'force_gt_mean_n', 'force_estimate_mean_n',
        'force_bias_n', 'force_mae_n', 'force_rmse_n',
        'force_nrmse_by_mean_abs_gt', 'force_r_squared',
        'force_absolute_relative_error_median',
        'force_absolute_relative_error_p95', 'eta_l_rmse',
        'eta_l_r_squared',
    )
    run_fields = (
        'source_csv', 'protocol_valid', 'protocol_invalid_reason',
        'used_in_validation', 'condition_id', 'schedule_mode', 'mass_scale',
        'va_target_mps', 'lambda_target', 'gust_amplitude_mps',
        'eta_l_available_sample_count',
        'eta_l_force_model_valid_sample_count',
        'eta_l_force_model_valid_raw_sample_count',
        'eta_l_scientific_phase_excluded_sample_count',
        'eta_l_valid_sample_count',
        'gt_coverage_fraction', 'mean_yaw_rad',
        'mean_wind_heading_parallel_mps', 'mean_wind_heading_cross_mps',
        'mean_vrel_body_u_mps', 'mean_vrel_body_v_mps',
        'mean_vrel_body_w_mps', 'mean_beta_rad', 'mean_abs_beta_rad',
        'mean_yaw_available_rad',
        'mean_wind_heading_parallel_available_mps',
        'mean_wind_heading_cross_available_mps',
        'mean_vrel_body_u_available_mps',
        'mean_vrel_body_v_available_mps',
        'mean_vrel_body_w_available_mps',
        'mean_beta_available_rad',
        'mean_abs_beta_available_rad',
        'mean_yaw_force_valid_rad',
        'mean_wind_heading_parallel_force_valid_mps',
        'mean_wind_heading_cross_force_valid_mps',
        'mean_vrel_body_u_force_valid_mps',
        'mean_vrel_body_v_force_valid_mps',
        'mean_vrel_body_w_force_valid_mps',
        'mean_beta_force_valid_rad',
        'mean_abs_beta_force_valid_rad',
        *metric_fields,
    )
    with (output_dir / 'f3a_eta_l_validation_runs.csv').open(
        'w', newline='', encoding='utf-8'
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=run_fields)
        writer.writeheader()
        for run in payload['runs']:
            writer.writerow({key: run.get(key) for key in run_fields})
    condition_fields = (
        'condition_id', 'eta_l_force_model_valid_sample_count',
        'eta_l_valid_sample_count', 'gt_coverage_fraction',
        *metric_fields, 'minimum_samples_pass',
        'ground_truth_coverage_pass', 'force_nrmse_pass',
        'median_relative_error_pass', 'condition_validation_pass',
    )
    with (output_dir / 'f3a_eta_l_validation_conditions.csv').open(
        'w', newline='', encoding='utf-8'
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=condition_fields)
        writer.writeheader()
        for condition in payload['conditions']:
            writer.writerow({
                key: condition.get(key) for key in condition_fields
            })
    operating_point_fields = (
        'condition_id', 'va_target_mps', 'lambda_target',
        'gust_amplitude_mps', 'eta_l_force_model_valid_sample_count',
        'eta_l_valid_sample_count',
        'gt_coverage_fraction', *metric_fields, 'minimum_samples_pass',
        'ground_truth_coverage_pass', 'force_nrmse_pass',
        'median_relative_error_pass', 'operating_point_validation_pass',
    )
    with (output_dir / 'f3a_eta_l_validation_operating_points.csv').open(
        'w', newline='', encoding='utf-8'
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=operating_point_fields)
        writer.writeheader()
        for point in payload['operating_points']:
            writer.writerow({
                key: point.get(key) for key in operating_point_fields
            })
    phase_fields = ('phase', *metric_fields)
    with (output_dir / 'f3a_eta_l_validation_phases.csv').open(
        'w', newline='', encoding='utf-8'
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=phase_fields)
        writer.writeheader()
        for phase in payload['phases']:
            writer.writerow({key: phase.get(key) for key in phase_fields})
    _render_validation(payload, rows, output_dir)


def eta_l_outcome_space(point_csvs: list[Path]) -> list[dict[str, object]]:
    points: list[dict[str, object]] = []
    for path in point_csvs:
        with path.open(newline='', encoding='utf-8') as stream:
            for row in csv.DictReader(stream):
                quality = row.get('data_quality', 'valid')
                eta_l = _finite(row.get('eta_l_proxy_mean'))
                va = _finite(row.get('va_target_mps'))
                lam = _finite(row.get('lambda_target'))
                physical = (
                    row.get('physical_class') or row.get('outcome_class')
                )
                outcome = PHYSICAL_TO_SYMBOL.get(physical or '')
                if (
                    quality in {'valid', 'valid_with_retries'}
                    and outcome in OUTCOME_COLORS
                    and all(math.isfinite(value) for value in (eta_l, va, lam))
                ):
                    points.append({
                        'source_csv': str(path.resolve()),
                        'condition_id': row.get('condition_id', path.stem),
                        'mass_scale': _finite(row.get('mass_scale')),
                        'va_target_mps': va,
                        'lambda_target': lam,
                        'eta_l': eta_l,
                        'outcome_class': outcome,
                    })
    if not points:
        raise ValueError('no valid eta_L outcome points found')
    return points


def write_eta_l_outcome_space(
    points: list[dict[str, object]], output_dir: Path,
) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    output_dir.mkdir(parents=True, exist_ok=True)
    fields = (
        'source_csv', 'condition_id', 'mass_scale', 'va_target_mps',
        'lambda_target', 'eta_l', 'outcome_class',
    )
    with (output_dir / 'f3a_eta_l_outcome_points.csv').open(
        'w', newline='', encoding='utf-8'
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(points)
    figure, axes = plt.subplots(1, 2, figsize=(11, 4.8), sharey=True)
    mass_scales = sorted({
        _finite(point['mass_scale']) for point in points
        if math.isfinite(_finite(point['mass_scale']))
    })
    markers = ['o', 's', '^', 'D']
    for index, mass in enumerate(mass_scales or [math.nan]):
        subset = [
            point for point in points
            if (
                not mass_scales
                or abs(_finite(point['mass_scale']) - mass) <= 1.0e-9
            )
        ]
        for outcome, color in OUTCOME_COLORS.items():
            selected = [
                point for point in subset
                if point['outcome_class'] == outcome
            ]
            if not selected:
                continue
            label_mass = f'{mass:g}$m_0$' if math.isfinite(mass) else 'dataset'
            label = f'{label_mass}, {outcome}'
            kwargs = dict(
                c=color, marker=markers[index % len(markers)], s=48,
                edgecolors='black', linewidths=0.35, alpha=0.85, label=label,
            )
            axes[0].scatter(
                [point['va_target_mps'] for point in selected],
                [point['lambda_target'] for point in selected], **kwargs,
            )
            axes[1].scatter(
                [point['eta_l'] for point in selected],
                [point['lambda_target'] for point in selected], **kwargs,
            )
    axes[0].set_xlabel(r'Nominal $V_a$ [m/s]')
    axes[1].set_xlabel(r'Estimated wing-support capability $\eta_L$')
    axes[0].set_ylabel(r'Transition allocation $\lambda$')
    axes[0].set_title(r'$V_a$--$\lambda$ space')
    axes[1].set_title(r'$\eta_L$--$\lambda$ space')
    for axis in axes:
        axis.grid(alpha=0.25)
        axis.set_ylim(-0.03, 1.03)
    handles, labels = axes[1].get_legend_handles_labels()
    figure.legend(
        handles, labels, loc='lower center', bbox_to_anchor=(0.5, -0.01),
        ncol=4,
    )
    figure.suptitle('F3-A capability-coordinate comparison across payloads')
    figure.tight_layout(rect=(0, 0.12, 1, 1))
    figure.savefig(output_dir / 'f3a_va_vs_eta_l_outcomes.png', dpi=180)
    plt.close(figure)
    counts = {
        outcome: sum(point['outcome_class'] == outcome for point in points)
        for outcome in OUTCOME_COLORS
    }
    payload = {
        'experiment': 'F3A_eta_L_lambda_outcome_space',
        'point_count': len(points),
        'outcome_counts': counts,
        'note': (
            'This plot is descriptive. Predictive information gain is tested '
            'later in F5 and is not inferred from visual overlap alone.'
        ),
    }
    (output_dir / 'f3a_eta_l_outcomes.json').write_text(
        json.dumps(payload, indent=2, allow_nan=False) + '\n', encoding='utf-8'
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest='command', required=True)
    validation = subparsers.add_parser('validate-eta-l')
    validation.add_argument('root', type=Path)
    validation.add_argument('--output-dir', type=Path, required=True)
    outcomes = subparsers.add_parser('plot-outcomes')
    outcomes.add_argument('point_csv', type=Path, nargs='+')
    outcomes.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    if args.command == 'validate-eta-l':
        payload, rows = validate_eta_l(args.root)
        write_eta_l_validation(payload, rows, args.output_dir)
        print(json.dumps(_json_safe(payload), indent=2, allow_nan=False))
    else:
        points = eta_l_outcome_space(args.point_csv)
        write_eta_l_outcome_space(points, args.output_dir)
        print(json.dumps({
            'experiment': 'F3A_eta_L_lambda_outcome_space',
            'point_count': len(points),
            'output_dir': str(args.output_dir.resolve()),
        }, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
