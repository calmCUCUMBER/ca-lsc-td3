"""F3-B1 directional remaining elevator-authority validation.

This validator checks the denominator used by eta_C,

    M_ava ~= qbar * |K_delta| * Delta(delta_e, directional limit).

The directional limit must follow the sign convention used by the online
eta_C implementation: the requested moment increment is converted to an
elevator-angle direction through the signed moment derivative.  The ground
truth comparator is reconstructed from the locally fitted elevator GT moment
model

    M_e^GT = b + K_delta * (qbar * delta_e) + K_alpha * (qbar * alpha).

The fitted K_alpha is a nuisance term; it cancels when evaluating the same
aerodynamic state at the directional elevator limit, but keeping it in the
fit prevents alpha-dependent trim moment from biasing K_delta.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import fmean, median
from typing import Iterable

from ca_lsc_td3.evaluation.f3b_moment_validation import (
    _centered_cell_metrics,
    _elevator_effective_angle,
    _finite,
    _json_safe,
    _mean,
    _r_squared,
    _rmse,
    _run_quality,
    _two_variable_fit,
)


def _percentile(values: Iterable[float], percentile: float) -> float:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return math.nan
    if len(finite) == 1:
        return finite[0]
    position = (len(finite) - 1) * percentile
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return finite[lower]
    fraction = position - lower
    return finite[lower] * (1.0 - fraction) + finite[upper] * fraction


def _value(row: dict[str, str], primary: str, fallback: str = '') -> float:
    value = _finite(row.get(primary))
    if math.isfinite(value) or not fallback:
        return value
    return _finite(row.get(fallback))


def _row_target(row: dict[str, str], summary_value: float, key: str) -> float:
    if math.isfinite(summary_value):
        return summary_value
    return _finite(row.get(key))


def _directional_limit(
    row: dict[str, str],
    *,
    epsilon: float = 1.0e-12,
) -> tuple[float, float, str]:
    qbar = _finite(row.get('qbar_selected_pa'))
    derivative_per_q = _finite(
        row.get('elevator_dmoment_ddelta_per_q_m3_config'))
    moment_derivative = qbar * derivative_per_q
    request = _value(
        row,
        'shadow_fw_pitch_moment_req_nm',
        'shadow_pitch_moment_increment_nm',
    )
    elevator_angle = _elevator_effective_angle(row)
    minimum = _finite(row.get('elevator_min_rad_config'))
    maximum = _finite(row.get('elevator_max_rad_config'))
    if not all(math.isfinite(value) for value in (
        moment_derivative,
        request,
        elevator_angle,
        minimum,
        maximum,
    )):
        return math.nan, math.nan, 'invalid'
    if maximum <= minimum or abs(moment_derivative) <= epsilon:
        return math.nan, math.nan, 'invalid'
    elevator_direction = request / moment_derivative
    if elevator_direction > epsilon:
        limit = maximum
        direction = 'toward_max'
    elif elevator_direction < -epsilon:
        limit = minimum
        direction = 'toward_min'
    else:
        margin_to_max = maximum - elevator_angle
        margin_to_min = elevator_angle - minimum
        if margin_to_max <= margin_to_min:
            limit = maximum
            direction = 'zero_request_nearest_max'
        else:
            limit = minimum
            direction = 'zero_request_nearest_min'
    remaining = max(0.0, abs(limit - elevator_angle))
    return limit, remaining, direction


def _fit_cell(rows: list[dict[str, str]]) -> tuple[float, float, float]:
    metrics = _centered_cell_metrics(rows)
    return (
        0.0,
        _finite(metrics.get('fit_derivative_per_q_m3')),
        _finite(metrics.get('fit_alpha_derivative_per_q_m3')),
    )


def _authority_rows(
    rows: list[dict[str, str]],
    *,
    intercept: float,
    k_delta: float,
    k_alpha: float,
    minimum_gt_available_moment_nm: float,
) -> tuple[list[dict[str, object]], dict[str, int]]:
    samples: list[dict[str, object]] = []
    direction_counts: dict[str, int] = {}
    for row in rows:
        if _finite(row.get('eta_c_valid')) <= 0.5:
            continue
        qbar = _finite(row.get('qbar_selected_pa'))
        alpha = _finite(row.get('alpha_est_rad'))
        elevator = _elevator_effective_angle(row)
        estimate = _value(
            row,
            'moment_available_est_nm',
            'eta_c_available_increment_moment_nm',
        )
        recorded_remaining = _value(
            row,
            'elevator_remaining_directional_rad',
            'eta_c_remaining_elevator_angle_rad',
        )
        limit, remaining, direction = _directional_limit(row)
        if not all(math.isfinite(value) for value in (
            qbar,
            alpha,
            elevator,
            estimate,
            limit,
            remaining,
            recorded_remaining,
            k_delta,
        )):
            continue
        # For a directional remaining-authority comparison, the GT moment is
        # evaluated at the same qbar and alpha.  The run-specific intercept and
        # qbar*alpha nuisance term cancel, leaving only the local K_delta.
        available_gt = abs(k_delta * qbar * (limit - elevator))
        if available_gt < minimum_gt_available_moment_nm:
            continue
        direction_counts[direction] = direction_counts.get(direction, 0) + 1
        samples.append({
            'source_csv': row.get('_source_csv', ''),
            'qbar_selected_pa': qbar,
            'alpha_est_rad': alpha,
            'elevator_angle_rad': elevator,
            'directional_limit_rad': limit,
            'direction': direction,
            'directional_remaining_recomputed_rad': remaining,
            'directional_remaining_recorded_rad': recorded_remaining,
            'moment_available_est_nm': estimate,
            'moment_available_gt_local_fit_nm': available_gt,
            'moment_available_error_nm': estimate - available_gt,
            'moment_available_relative_error': (
                abs(estimate - available_gt) / available_gt
                if available_gt > 0.0 else math.nan
            ),
        })
    return samples, direction_counts


def _cell_result(
    key: tuple[float, float],
    rows: list[dict[str, str]],
    *,
    minimum_samples_per_cell: int,
    minimum_gt_available_moment_nm: float,
    maximum_normalized_rmse: float,
    maximum_median_relative_error: float,
    maximum_p95_relative_error: float,
    minimum_remaining_match_fraction: float,
    remaining_match_tolerance_rad: float,
) -> dict[str, object]:
    intercept, k_delta, k_alpha = _fit_cell(rows)
    samples, direction_counts = _authority_rows(
        rows,
        intercept=intercept,
        k_delta=k_delta,
        k_alpha=k_alpha,
        minimum_gt_available_moment_nm=minimum_gt_available_moment_nm,
    )
    estimates = [
        float(sample['moment_available_est_nm']) for sample in samples
    ]
    truth = [
        float(sample['moment_available_gt_local_fit_nm'])
        for sample in samples
    ]
    errors = [estimate - actual for estimate, actual in zip(estimates, truth)]
    relative_errors = [
        abs(error) / actual
        for error, actual in zip(errors, truth)
        if actual > 0.0
    ]
    remaining_matches = [
        abs(
            float(sample['directional_remaining_recomputed_rad'])
            - float(sample['directional_remaining_recorded_rad'])
        ) <= remaining_match_tolerance_rad
        for sample in samples
    ]
    rmse = _rmse(estimates, truth)
    gt_abs_mean = _mean(abs(value) for value in truth)
    normalized_rmse = (
        rmse / gt_abs_mean
        if math.isfinite(rmse) and math.isfinite(gt_abs_mean)
        and gt_abs_mean > 0.0 else math.nan
    )
    median_relative_error = (
        median(relative_errors) if relative_errors else math.nan
    )
    p95_relative_error = _percentile(relative_errors, 0.95)
    remaining_match_fraction = (
        sum(remaining_matches) / len(remaining_matches)
        if remaining_matches else math.nan
    )
    checks = {
        'minimum_samples': len(samples) >= minimum_samples_per_cell,
        'remaining_direction_matches_online': (
            math.isfinite(remaining_match_fraction)
            and remaining_match_fraction >= minimum_remaining_match_fraction
        ),
        'normalized_rmse': (
            math.isfinite(normalized_rmse)
            and normalized_rmse <= maximum_normalized_rmse
        ),
        'median_relative_error': (
            math.isfinite(median_relative_error)
            and median_relative_error <= maximum_median_relative_error
        ),
        'p95_relative_error': (
            math.isfinite(p95_relative_error)
            and p95_relative_error <= maximum_p95_relative_error
        ),
    }
    return {
        'va_target_mps': key[0],
        'lambda_target': key[1],
        'sample_count': len(samples),
        'source_csv_count': len({
            str(sample['source_csv']) for sample in samples
        }),
        'fit_model': 'qbar_delta_plus_qbar_alpha_nuisance',
        'local_fit_intercept_nm': intercept,
        'local_fit_derivative_per_q_m3': k_delta,
        'local_fit_alpha_derivative_per_q_m3': k_alpha,
        'moment_available_gt_mean_nm': _mean(truth),
        'moment_available_est_mean_nm': _mean(estimates),
        'moment_available_bias_nm': _mean(errors),
        'moment_available_rmse_nm': rmse,
        'moment_available_normalized_rmse': normalized_rmse,
        'moment_available_mae_nm': _mean(abs(error) for error in errors),
        'moment_available_r_squared_diagnostic': _r_squared(estimates, truth),
        'moment_available_median_relative_error': median_relative_error,
        'moment_available_p95_relative_error': p95_relative_error,
        'directional_remaining_match_fraction': remaining_match_fraction,
        'direction_counts': dict(sorted(direction_counts.items())),
        'checks': checks,
        'cell_validation_pass': all(checks.values()),
    }


def validate_remaining_authority(
    root: Path,
    *,
    minimum_samples_per_cell: int = 50,
    minimum_valid_cells: int = 4,
    required_airspeeds_mps: Iterable[float] = (6.0, 10.0, 14.0, 18.0),
    minimum_gt_coverage: float = 0.90,
    maximum_altitude_rmse_m: float = 3.0,
    maximum_abs_altitude_error_m: float = 5.0,
    minimum_gt_available_moment_nm: float = 0.05,
    maximum_normalized_rmse: float = 0.15,
    maximum_median_relative_error: float = 0.15,
    maximum_p95_relative_error: float = 0.25,
    minimum_remaining_match_fraction: float = 0.95,
    remaining_match_tolerance_rad: float = 1.0e-5,
) -> dict[str, object]:
    grouped: dict[tuple[float, float], list[dict[str, str]]] = {}
    valid_run_count = 0
    excluded_runs: list[dict[str, object]] = []
    for path in sorted(root.rglob('telemetry.csv')):
        ok, result, selected_rows, _ = _run_quality(
            path,
            minimum_gt_coverage=minimum_gt_coverage,
            maximum_altitude_rmse_m=maximum_altitude_rmse_m,
            maximum_abs_altitude_error_m=maximum_abs_altitude_error_m,
        )
        if not ok:
            excluded_runs.append(result)
            continue
        valid_run_count += 1
        va = result['va_target_mps']
        lam = result['lambda_target']
        if selected_rows:
            va = _row_target(selected_rows[0], _finite(va), 'va_target_config')
            lam = _row_target(
                selected_rows[0], _finite(lam), 'lambda_target_config')
        key = (round(float(va), 6), round(float(lam), 6))
        grouped.setdefault(key, []).extend(selected_rows)

    cell_results = [
        _cell_result(
            key,
            rows,
            minimum_samples_per_cell=minimum_samples_per_cell,
            minimum_gt_available_moment_nm=minimum_gt_available_moment_nm,
            maximum_normalized_rmse=maximum_normalized_rmse,
            maximum_median_relative_error=maximum_median_relative_error,
            maximum_p95_relative_error=maximum_p95_relative_error,
            minimum_remaining_match_fraction=minimum_remaining_match_fraction,
            remaining_match_tolerance_rad=remaining_match_tolerance_rad,
        )
        for key, rows in sorted(grouped.items())
    ]
    passing_cells = [
        cell for cell in cell_results if bool(cell['cell_validation_pass'])
    ]
    present_required = {
        round(float(cell['va_target_mps']), 6)
        for cell in passing_cells
    }
    missing_required = [
        float(value) for value in required_airspeeds_mps
        if round(float(value), 6) not in present_required
    ]
    checks = {
        'minimum_valid_cells': len(passing_cells) >= minimum_valid_cells,
        'required_airspeeds_present': not missing_required,
        'all_quality_valid_cells_authority_pass': (
            len(passing_cells) == len(cell_results)
            and bool(cell_results)
        ),
    }
    return {
        'experiment': 'F3B1_directional_remaining_authority_validation',
        'root': str(root.resolve()),
        'thresholds': {
            'minimum_samples_per_cell': minimum_samples_per_cell,
            'minimum_valid_cells': minimum_valid_cells,
            'required_airspeeds_mps': [
                float(value) for value in required_airspeeds_mps
            ],
            'minimum_gt_coverage': minimum_gt_coverage,
            'maximum_altitude_rmse_m': maximum_altitude_rmse_m,
            'maximum_abs_altitude_error_m': maximum_abs_altitude_error_m,
            'minimum_gt_available_moment_nm': minimum_gt_available_moment_nm,
            'maximum_normalized_rmse': maximum_normalized_rmse,
            'maximum_median_relative_error': maximum_median_relative_error,
            'maximum_p95_relative_error': maximum_p95_relative_error,
            'minimum_remaining_match_fraction': (
                minimum_remaining_match_fraction
            ),
            'remaining_match_tolerance_rad': remaining_match_tolerance_rad,
        },
        'valid_run_count': valid_run_count,
        'excluded_run_count': len(excluded_runs),
        'excluded_runs': excluded_runs,
        'cell_results': cell_results,
        'valid_cell_count': len(cell_results),
        'authority_pass_cell_count': len(passing_cells),
        'missing_required_airspeeds_mps': missing_required,
        'moment_available_normalized_rmse_max': max((
            _finite(cell['moment_available_normalized_rmse'])
            for cell in passing_cells
        ), default=math.nan),
        'moment_available_median_relative_error_max': max((
            _finite(cell['moment_available_median_relative_error'])
            for cell in passing_cells
        ), default=math.nan),
        'moment_available_p95_relative_error_max': max((
            _finite(cell['moment_available_p95_relative_error'])
            for cell in passing_cells
        ), default=math.nan),
        'checks': checks,
        'f3b1_remaining_authority_validation_pass': all(checks.values()),
        'interpretation': (
            'directional available elevator moment is validated against a '
            'local GT moment model fitted from instrumented elevator wrench; '
            'absolute RMSE is diagnostic because available moment magnitude '
            'scales strongly with dynamic pressure'
        ),
    }


def write_remaining_authority(payload: dict[str, object], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    safe = _json_safe(payload)
    (output_dir / 'f3b1_remaining_authority_validation.json').write_text(
        json.dumps(safe, indent=2, allow_nan=False) + '\n',
        encoding='utf-8',
    )
    cell_path = output_dir / 'f3b1_remaining_authority_cells.csv'
    fieldnames = [
        'va_target_mps',
        'lambda_target',
        'cell_validation_pass',
        'sample_count',
        'source_csv_count',
        'local_fit_derivative_per_q_m3',
        'moment_available_gt_mean_nm',
        'moment_available_est_mean_nm',
        'moment_available_bias_nm',
        'moment_available_rmse_nm',
        'moment_available_normalized_rmse',
        'moment_available_r_squared_diagnostic',
        'moment_available_median_relative_error',
        'moment_available_p95_relative_error',
        'directional_remaining_match_fraction',
        'direction_counts',
        'checks',
    ]
    with cell_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for cell in payload['cell_results']:
            row = {
                key: (
                    json.dumps(cell[key], sort_keys=True)
                    if isinstance(cell.get(key), dict) else cell.get(key)
                )
                for key in fieldnames
            }
            writer.writerow(row)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--minimum-valid-cells', type=int, default=4)
    parser.add_argument(
        '--required-va',
        type=float,
        nargs='*',
        default=[6.0, 10.0, 14.0, 18.0],
    )
    args = parser.parse_args()
    payload = validate_remaining_authority(
        args.root,
        minimum_valid_cells=args.minimum_valid_cells,
        required_airspeeds_mps=args.required_va,
    )
    write_remaining_authority(payload, args.output_dir)
    print(json.dumps(_json_safe(payload), indent=2, allow_nan=False))
    if not payload['f3b1_remaining_authority_validation_pass']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
