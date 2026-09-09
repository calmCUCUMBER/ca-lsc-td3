"""F3-B2 eta_C end-to-end behavior validation.

F3-B1 validated the elevator incremental derivative and the directional
available moment.  F3-B2 stitches those pieces into a ground-truth capability
metric

    eta_C^GT = g_q * sat(1 - |M_req^FW| / (M_ava^GT + eps))

and compares it against the online eta_C telemetry.  The GT available moment
uses the run-intercept / within-run-centered K_delta fit from F3-B1, so this
is not merely replaying the online denominator.
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
)
from ca_lsc_td3.evaluation.f3b_remaining_authority import _directional_limit


def _unit_interval(value: float) -> float:
    if not math.isfinite(value):
        return math.nan
    return min(1.0, max(0.0, value))


def _smoothstep(value: float, lower: float, upper: float) -> float:
    if not all(math.isfinite(item) for item in (value, lower, upper)):
        return math.nan
    if upper <= lower:
        return math.nan
    if value <= lower:
        return 0.0
    if value >= upper:
        return 1.0
    t = (value - lower) / (upper - lower)
    return t * t * (3.0 - 2.0 * t)


def _mae(predicted: Iterable[float], actual: Iterable[float]) -> float:
    pairs = [
        (prediction, truth)
        for prediction, truth in zip(predicted, actual)
        if math.isfinite(prediction) and math.isfinite(truth)
    ]
    if not pairs:
        return math.nan
    return fmean(abs(prediction - truth) for prediction, truth in pairs)


def _median_abs_error(predicted: Iterable[float], actual: Iterable[float]) -> float:
    errors = [
        abs(prediction - truth)
        for prediction, truth in zip(predicted, actual)
        if math.isfinite(prediction) and math.isfinite(truth)
    ]
    return median(errors) if errors else math.nan


def _rank(values: list[float]) -> list[float]:
    order = sorted(range(len(values)), key=lambda index: values[index])
    ranks = [0.0] * len(values)
    index = 0
    while index < len(values):
        end = index
        while (
            end + 1 < len(values)
            and values[order[end + 1]] == values[order[index]]
        ):
            end += 1
        rank = 0.5 * (index + end) + 1.0
        for item in range(index, end + 1):
            ranks[order[item]] = rank
        index = end + 1
    return ranks


def _pearson(left: list[float], right: list[float]) -> float:
    pairs = [
        (x, y) for x, y in zip(left, right)
        if math.isfinite(x) and math.isfinite(y)
    ]
    if len(pairs) < 2:
        return math.nan
    xs = [x for x, _ in pairs]
    ys = [y for _, y in pairs]
    x_mean = fmean(xs)
    y_mean = fmean(ys)
    numerator = sum((x - x_mean) * (y - y_mean) for x, y in pairs)
    denominator = math.sqrt(
        sum((x - x_mean) ** 2 for x in xs)
        * sum((y - y_mean) ** 2 for y in ys)
    )
    return numerator / denominator if denominator > 1.0e-12 else math.nan


def _spearman(left: list[float], right: list[float]) -> float:
    pairs = [
        (x, y) for x, y in zip(left, right)
        if math.isfinite(x) and math.isfinite(y)
    ]
    if len(pairs) < 2:
        return math.nan
    return _pearson(
        _rank([x for x, _ in pairs]),
        _rank([y for _, y in pairs]),
    )


def _strictly_increasing(values: list[float]) -> bool:
    return all(later > earlier for earlier, later in zip(values, values[1:]))


def _strictly_decreasing(values: list[float]) -> bool:
    return all(later < earlier for earlier, later in zip(values, values[1:]))


def _directional_margin_for_request(
    row: dict[str, str],
    *,
    request_nm: float,
    elevator_angle_rad: float,
    epsilon: float = 1.0e-12,
) -> tuple[float, float, str]:
    qbar = _finite(row.get('qbar_selected_pa'))
    derivative_per_q = _finite(
        row.get('elevator_dmoment_ddelta_per_q_m3_config'))
    moment_derivative = qbar * derivative_per_q
    minimum = _finite(row.get('elevator_min_rad_config'))
    maximum = _finite(row.get('elevator_max_rad_config'))
    if not all(math.isfinite(value) for value in (
        moment_derivative, request_nm, elevator_angle_rad, minimum, maximum,
    )):
        return math.nan, math.nan, 'invalid'
    if maximum <= minimum or abs(moment_derivative) <= epsilon:
        return math.nan, math.nan, 'invalid'
    elevator_direction = request_nm / moment_derivative
    if elevator_direction > epsilon:
        limit = maximum
        direction = 'toward_max'
    elif elevator_direction < -epsilon:
        limit = minimum
        direction = 'toward_min'
    else:
        margin_to_max = maximum - elevator_angle_rad
        margin_to_min = elevator_angle_rad - minimum
        if margin_to_max <= margin_to_min:
            limit = maximum
            direction = 'zero_request_nearest_max'
        else:
            limit = minimum
            direction = 'zero_request_nearest_min'
    return limit, max(0.0, abs(limit - elevator_angle_rad)), direction


def _nearly_constant(values: list[float], *, relative: float = 0.05,
                     absolute: float = 0.05) -> bool:
    finite = [abs(value) for value in values if math.isfinite(value)]
    if len(finite) < 3:
        return False
    spread = max(finite) - min(finite)
    center = fmean(finite)
    return spread <= absolute or (
        center > 1.0e-12 and spread / center <= relative
    )


def _cell_key(summary: dict[str, object], rows: list[dict[str, str]]) -> tuple[float, float]:
    va = _finite(summary.get('va_target_mps'))
    lam = _finite(summary.get('lambda_target'))
    if not math.isfinite(va):
        va = _finite(summary.get('va_target_config'))
    if not math.isfinite(lam):
        lam = _finite(summary.get('lambda_target_config'))
    if rows:
        if not math.isfinite(va):
            va = _finite(rows[0].get('va_target_config'))
        if not math.isfinite(lam):
            lam = _finite(rows[0].get('lambda_target_config'))
    return round(va, 6), round(lam, 6)


def _sample_rows(
    grouped_rows: dict[tuple[float, float], list[dict[str, str]]],
    *,
    epsilon: float,
) -> list[dict[str, object]]:
    samples: list[dict[str, object]] = []
    for key, rows in sorted(grouped_rows.items()):
        fit = _centered_cell_metrics(rows)
        k_delta = _finite(fit.get('fit_derivative_per_q_m3'))
        for row in rows:
            if _finite(row.get('eta_c_valid')) <= 0.5:
                continue
            qbar = _finite(row.get('qbar_selected_pa'))
            elevator = _finite(row.get('elevator_control_angle_for_eta_c_rad'))
            if not math.isfinite(elevator):
                elevator = _elevator_effective_angle(row)
            lower = _finite(row.get('eta_c_pressure_gate_lower_pa_config'))
            upper = _finite(row.get('eta_c_pressure_gate_upper_pa_config'))
            request_raw = _finite(row.get('shadow_fw_pitch_moment_req_raw_nm'))
            if not math.isfinite(request_raw):
                request_raw = _finite(row.get('shadow_fw_pitch_moment_req_nm'))
            request = _finite(row.get('shadow_fw_pitch_moment_req_for_eta_c_nm'))
            if not math.isfinite(request):
                request = request_raw
            eta_est = _finite(row.get('eta_c'))
            eta_raw_est = _finite(row.get('eta_c_raw'))
            q_gate_est = _finite(row.get('eta_c_q_gate'))
            if not math.isfinite(q_gate_est):
                q_gate_est = _finite(row.get('eta_c_pressure_gate'))
            raw_limit, raw_remaining, raw_direction = _directional_limit(row)
            limit, remaining, direction = _directional_margin_for_request(
                row,
                request_nm=request,
                elevator_angle_rad=elevator,
            )
            if not all(math.isfinite(value) for value in (
                qbar, elevator, lower, upper, request,
                eta_est, limit, remaining,
            )):
                continue
            available_gt = _finite(row.get('moment_available_gt_nm'))
            if not math.isfinite(available_gt) and math.isfinite(k_delta):
                available_gt = abs(k_delta * qbar * (limit - elevator))
            if not math.isfinite(available_gt):
                continue
            q_gate_gt = _smoothstep(qbar, lower, upper)
            if available_gt <= epsilon:
                moment_margin_gt = 0.0
            else:
                moment_margin_gt = _unit_interval(
                    1.0 - abs(request) / (available_gt + epsilon)
                )
            eta_gt = _unit_interval(q_gate_gt * moment_margin_gt)
            mode = (row.get('f3b2_test_mode') or '').strip()
            stage = _finite(row.get('f3b2_stage_index'))
            samples.append({
                'source_csv': row.get('_source_csv', ''),
                'va_target_mps': key[0],
                'lambda_target': key[1],
                'f3b2_test_mode': mode,
                'f3b2_stage_index': stage,
                'qbar_selected_pa': qbar,
                'eta_c_q_gate_est': q_gate_est,
                'eta_c_q_gate_gt': q_gate_gt,
                'shadow_fw_pitch_moment_req_nm': request,
                'shadow_fw_pitch_moment_req_raw_nm': request_raw,
                'shadow_fw_pitch_moment_req_for_eta_c_nm': request,
                'elevator_angle_rad': elevator,
                'raw_directional_limit_rad': raw_limit,
                'raw_directional_remaining_rad': raw_remaining,
                'raw_direction': raw_direction,
                'directional_limit_rad': limit,
                'directional_remaining_rad': remaining,
                'directional_remaining_for_eta_c_rad': remaining,
                'direction': direction,
                'moment_available_est_nm': _finite(
                    row.get('eta_c_available_increment_moment_nm')),
                'moment_available_gt_nm': available_gt,
                'eta_c_raw_est': eta_raw_est,
                'eta_c_raw_gt': moment_margin_gt,
                'eta_c_est': eta_est,
                'eta_c_gt': eta_gt,
                'eta_c_error': eta_est - eta_gt,
                'eta_c_modifies_px4_blending': _finite(
                    row.get('eta_c_modifies_px4_blending')),
            })
    return samples


def _metrics(samples: list[dict[str, object]]) -> dict[str, object]:
    estimates = [_finite(sample.get('eta_c_est')) for sample in samples]
    truth = [_finite(sample.get('eta_c_gt')) for sample in samples]
    abs_errors = [
        abs(estimate - actual)
        for estimate, actual in zip(estimates, truth)
        if math.isfinite(estimate) and math.isfinite(actual)
    ]
    return {
        'sample_count': len(samples),
        'eta_c_gt_rmse': _rmse(estimates, truth),
        'eta_c_gt_mae': _mae(estimates, truth),
        'eta_c_gt_median_abs_error': (
            median(abs_errors) if abs_errors else math.nan
        ),
        'eta_c_gt_r_squared': _r_squared(estimates, truth),
        'eta_c_gt_spearman': _spearman(estimates, truth),
        'eta_c_est_mean': _mean(estimates),
        'eta_c_gt_mean': _mean(truth),
        'eta_c_error_mean': _mean(
            estimate - actual for estimate, actual in zip(estimates, truth)
        ),
        'eta_c_error_abs_max': max(abs_errors) if abs_errors else math.nan,
    }


def _stage_summaries(samples: list[dict[str, object]]) -> list[dict[str, object]]:
    grouped: dict[tuple[str, float], list[dict[str, object]]] = {}
    for sample in samples:
        mode = str(sample.get('f3b2_test_mode') or '').strip()
        stage = _finite(sample.get('f3b2_stage_index'))
        if not mode or not math.isfinite(stage):
            continue
        grouped.setdefault((mode, stage), []).append(sample)
    summaries: list[dict[str, object]] = []
    for (mode, stage), items in sorted(grouped.items()):
        summaries.append({
            'f3b2_test_mode': mode,
            'f3b2_stage_index': stage,
            'sample_count': len(items),
            'qbar_mean_pa': _mean(
                _finite(item.get('qbar_selected_pa')) for item in items),
            'q_gate_mean': _mean(
                _finite(item.get('eta_c_q_gate_gt')) for item in items),
            'abs_shadow_moment_req_mean_nm': _mean(
                abs(_finite(item.get(
                    'shadow_fw_pitch_moment_req_for_eta_c_nm')))
                for item in items
            ),
            'abs_shadow_moment_req_raw_mean_nm': _mean(
                abs(_finite(item.get('shadow_fw_pitch_moment_req_raw_nm')))
                for item in items
            ),
            'abs_shadow_moment_req_for_eta_c_mean_nm': _mean(
                abs(_finite(item.get(
                    'shadow_fw_pitch_moment_req_for_eta_c_nm')))
                for item in items
            ),
            'directional_remaining_raw_mean_rad': _mean(
                _finite(item.get('raw_directional_remaining_rad'))
                for item in items
            ),
            'directional_remaining_mean_rad': _mean(
                _finite(item.get('directional_remaining_for_eta_c_rad'))
                for item in items
            ),
            'directional_remaining_for_eta_c_mean_rad': _mean(
                _finite(item.get('directional_remaining_for_eta_c_rad'))
                for item in items
            ),
            'moment_available_gt_mean_nm': _mean(
                _finite(item.get('moment_available_gt_nm'))
                for item in items
            ),
            'eta_c_est_mean': _mean(
                _finite(item.get('eta_c_est')) for item in items),
            'eta_c_gt_mean': _mean(
                _finite(item.get('eta_c_gt')) for item in items),
        })
    return summaries


def _ordering_verdicts(stage_summaries: list[dict[str, object]]) -> dict[str, object]:
    by_mode: dict[str, list[dict[str, object]]] = {}
    for item in stage_summaries:
        by_mode.setdefault(str(item['f3b2_test_mode']), []).append(item)
    for items in by_mode.values():
        items.sort(key=lambda item: _finite(item.get('f3b2_stage_index')))

    q_gate_items = by_mode.get('q_gate', [])
    demand_items = by_mode.get('demand', [])
    remaining_items = by_mode.get('remaining_travel', [])
    q_gate_pass = None
    demand_pass = None
    remaining_pass = None
    remaining_request_fixed_pass = None
    remaining_qbar_constant_pass = None
    remaining_available_decreases_pass = None
    remaining_eta_gt_decreases_pass = None
    remaining_eta_est_decreases_pass = None
    if len(q_gate_items) >= 3:
        q_gate_pass = (
            _strictly_increasing([
                _finite(item['qbar_mean_pa']) for item in q_gate_items
            ])
            and _strictly_increasing([
                _finite(item['q_gate_mean']) for item in q_gate_items
            ])
        )
    if len(demand_items) >= 3:
        demand_pass = (
            _strictly_increasing([
                _finite(item['abs_shadow_moment_req_for_eta_c_mean_nm'])
                for item in demand_items
            ])
            and _strictly_decreasing([
                _finite(item['eta_c_gt_mean']) for item in demand_items
            ])
            and _strictly_decreasing([
                _finite(item['eta_c_est_mean']) for item in demand_items
            ])
        )
    if len(remaining_items) >= 3:
        remaining_request_fixed_pass = _nearly_constant([
            _finite(item['abs_shadow_moment_req_for_eta_c_mean_nm'])
            for item in remaining_items
        ])
        remaining_qbar_constant_pass = _nearly_constant([
            _finite(item['qbar_mean_pa']) for item in remaining_items
        ], relative=0.05, absolute=5.0)
        remaining_available_decreases_pass = _strictly_decreasing([
            _finite(item['moment_available_gt_mean_nm'])
            for item in remaining_items
        ])
        remaining_eta_gt_decreases_pass = _strictly_decreasing([
            _finite(item['eta_c_gt_mean']) for item in remaining_items
        ])
        remaining_eta_est_decreases_pass = _strictly_decreasing([
            _finite(item['eta_c_est_mean']) for item in remaining_items
        ])
        remaining_pass = (
            remaining_request_fixed_pass
            and remaining_qbar_constant_pass
            and
            _strictly_decreasing([
                _finite(item['directional_remaining_for_eta_c_mean_rad'])
                for item in remaining_items
            ])
            and remaining_available_decreases_pass
            and remaining_eta_gt_decreases_pass
            and remaining_eta_est_decreases_pass
        )
    available = all(value is not None for value in (
        q_gate_pass, demand_pass, remaining_pass,
    ))
    return {
        'behavior_ordering_available': available,
        'q_gate_behavior_pass': q_gate_pass,
        'demand_behavior_pass': demand_pass,
        'remaining_travel_behavior_pass': remaining_pass,
        'remaining_travel_request_fixed_pass': (
            remaining_request_fixed_pass
        ),
        'remaining_travel_qbar_constant_pass': remaining_qbar_constant_pass,
        'remaining_travel_available_decreases_pass': (
            remaining_available_decreases_pass
        ),
        'remaining_travel_eta_gt_decreases_pass': (
            remaining_eta_gt_decreases_pass
        ),
        'remaining_travel_eta_est_decreases_pass': (
            remaining_eta_est_decreases_pass
        ),
    }


def validate_eta_c_behavior(
    root: Path,
    *,
    minimum_samples: int = 200,
    maximum_eta_c_rmse: float = 0.10,
    maximum_eta_c_median_abs_error: float = 0.08,
    minimum_eta_c_spearman: float = 0.90,
    minimum_gt_coverage: float = 0.90,
    maximum_altitude_rmse_m: float = 3.0,
    maximum_abs_altitude_error_m: float = 5.0,
    epsilon: float = 1.0e-12,
) -> dict[str, object]:
    grouped_rows: dict[tuple[float, float], list[dict[str, str]]] = {}
    run_quality_results: list[dict[str, object]] = []
    excluded_runs: list[dict[str, object]] = []
    for path in sorted(root.rglob('telemetry.csv')):
        ok, quality, selected_rows, all_rows = _run_quality(
            path,
            minimum_gt_coverage=minimum_gt_coverage,
            maximum_altitude_rmse_m=maximum_altitude_rmse_m,
            maximum_abs_altitude_error_m=maximum_abs_altitude_error_m,
        )
        run_quality_results.append(quality)
        if not ok:
            excluded_runs.append(quality)
            continue
        key = _cell_key(quality, selected_rows or all_rows)
        if all(math.isfinite(value) for value in key):
            grouped_rows.setdefault(key, []).extend(selected_rows)

    samples = _sample_rows(grouped_rows, epsilon=epsilon)
    metrics = _metrics(samples)
    stage_summaries = _stage_summaries(samples)
    behavior_observed = bool(stage_summaries)
    ordering = _ordering_verdicts(stage_summaries)
    modifies_values = [
        _finite(sample.get('eta_c_modifies_px4_blending'))
        for sample in samples
    ]
    checks = {
        'minimum_samples': metrics['sample_count'] >= minimum_samples,
        'eta_c_gt_rmse': (
            math.isfinite(_finite(metrics['eta_c_gt_rmse']))
            and _finite(metrics['eta_c_gt_rmse']) <= maximum_eta_c_rmse
        ),
        'eta_c_gt_median_abs_error': (
            math.isfinite(_finite(metrics['eta_c_gt_median_abs_error']))
            and _finite(metrics['eta_c_gt_median_abs_error'])
            <= maximum_eta_c_median_abs_error
        ),
        'eta_c_gt_spearman': (
            math.isfinite(_finite(metrics['eta_c_gt_spearman']))
            and _finite(metrics['eta_c_gt_spearman'])
            >= minimum_eta_c_spearman
        ),
        'eta_c_modifies_px4_blending_pass': (
            bool(modifies_values)
            and all(value == 0.0 for value in modifies_values
                    if math.isfinite(value))
        ),
    }
    offline_pass = all(checks.values())
    behavior_values = [
        ordering['q_gate_behavior_pass'],
        ordering['demand_behavior_pass'],
        ordering['remaining_travel_behavior_pass'],
    ]
    behavior_smoke_pass = None
    if behavior_observed:
        behavior_smoke_pass = (
            offline_pass
            and ordering['behavior_ordering_available']
            and all(value is True for value in behavior_values)
        )
    return {
        'experiment': 'F3B2_eta_C_behavior_validation',
        'root': str(root.resolve()),
        'analysis_type': (
            'behavior_smoke' if behavior_observed else 'offline_presmoke'
        ),
        'thresholds': {
            'minimum_samples': minimum_samples,
            'maximum_eta_c_rmse': maximum_eta_c_rmse,
            'maximum_eta_c_median_abs_error': maximum_eta_c_median_abs_error,
            'minimum_eta_c_spearman': minimum_eta_c_spearman,
            'minimum_gt_coverage': minimum_gt_coverage,
            'maximum_altitude_rmse_m': maximum_altitude_rmse_m,
            'maximum_abs_altitude_error_m': maximum_abs_altitude_error_m,
        },
        'valid_run_count': sum(
            1 for item in run_quality_results if bool(item.get('pass'))
        ),
        'excluded_run_count': len(excluded_runs),
        'excluded_runs': excluded_runs,
        'valid_cell_count': len(grouped_rows),
        **metrics,
        'stage_summaries': stage_summaries,
        **ordering,
        'checks': checks,
        'f3b2_eta_c_offline_presmoke_pass': offline_pass,
        'f3b2_eta_c_behavior_smoke_pass': behavior_smoke_pass,
    }


def write_eta_c_behavior(payload: dict[str, object], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / 'f3b2_eta_c_behavior_validation.json').write_text(
        json.dumps(_json_safe(payload), indent=2, allow_nan=False) + '\n',
        encoding='utf-8',
    )
    with (output_dir / 'f3b2_eta_c_behavior_stages.csv').open(
        'w', newline='', encoding='utf-8',
    ) as stream:
        fieldnames = (
            'f3b2_test_mode', 'f3b2_stage_index', 'sample_count',
            'qbar_mean_pa', 'q_gate_mean',
            'abs_shadow_moment_req_mean_nm',
            'abs_shadow_moment_req_raw_mean_nm',
            'abs_shadow_moment_req_for_eta_c_mean_nm',
            'directional_remaining_raw_mean_rad',
            'directional_remaining_mean_rad',
            'directional_remaining_for_eta_c_mean_rad',
            'moment_available_gt_mean_nm',
            'eta_c_est_mean', 'eta_c_gt_mean',
        )
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for item in payload.get('stage_summaries', []):
            writer.writerow({key: _json_safe(item.get(key)) for key in fieldnames})


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument(
        '--allow-incomplete-behavior',
        action='store_true',
        help=(
            'Write diagnostics without failing when only a partial F3-B2 '
            'behavior matrix is present.  Full merged behavior verdicts '
            'should leave this disabled.'
        ),
    )
    args = parser.parse_args()
    payload = validate_eta_c_behavior(args.root)
    write_eta_c_behavior(payload, args.output_dir)
    print(json.dumps(_json_safe(payload), indent=2, allow_nan=False))
    if not payload['f3b2_eta_c_offline_presmoke_pass']:
        raise SystemExit(1)
    if (
        payload['f3b2_eta_c_behavior_smoke_pass'] is False
        and not args.allow_incomplete_behavior
    ):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
