"""Generate auditable representative-run F1 time-series diagnostics."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path
from statistics import median

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from ca_lsc_td3.evaluation.f1_grid import (
    _data_quality_failure_reason,
    _finite,
    _is_nonconvergent,
    _is_physical_abort,
    _json_safe,
    _load_json,
    _physical_abort_reason,
    _nonconvergent_reason,
    _power_sanity,
)


DEFAULT_POINTS = ('16:0.6', '10:0.8', '12:0.6', '8:0.9')

REPRESENTATIVE_KEYS = (
    'va_hold_mean_airspeed_mps',
    'va_hold_altitude_rmse_m',
    'va_hold_max_abs_altitude_error_m',
    'va_hold_mean_lambda_exec',
    'va_hold_mean_lift_thrust_n',
    'va_hold_mean_lift_power_w',
    'va_hold_mean_pusher_power_w',
    'va_hold_mean_total_power_w',
    'va_hold_max_abs_alpha_rad',
    'va_hold_pitch_rate_p95_rad_s',
)


def _physical_class_display(value: object) -> str:
    return {
        'nominal_feasible': 'N nominal',
        'boundary_feasible': 'B marginal feasible',
        'nonconvergent': 'I non-convergent',
        'physical_unsafe': 'X physical unsafe',
    }.get(str(value), str(value))


def _rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def _abort_time(rows: list[dict[str, str]]) -> float:
    for row in rows:
        if str(row.get('va_hold_abort_reason', 'none')) not in {'', 'none'}:
            return _finite(row.get('time_s'))
    return math.nan


def _median_distance_scores(
    candidates: list[tuple[Path, dict[str, object]]]
) -> dict[str, float]:
    centers: dict[str, float] = {}
    scales: dict[str, float] = {}
    for key in REPRESENTATIVE_KEYS:
        values = [_finite(summary.get(key)) for _, summary in candidates]
        values = [value for value in values if math.isfinite(value)]
        if not values:
            continue
        center = median(values)
        deviations = [abs(value - center) for value in values]
        scale = median(deviations)
        centers[key] = center
        scales[key] = scale if scale > 1e-12 else 1.0
    scores: dict[str, float] = {}
    for run_dir, summary in candidates:
        normalized: list[float] = []
        for key, center in centers.items():
            value = _finite(summary.get(key))
            if math.isfinite(value):
                normalized.append(abs(value - center) / scales[key])
        scores[run_dir.name] = (
            sum(normalized) / len(normalized) if normalized else math.inf
        )
    return scores


def select_representative_run(
    point_dir: Path,
) -> tuple[Path, dict[str, object], str, dict[str, float]]:
    candidates: list[tuple[Path, dict[str, object]]] = []
    for summary_path in sorted(point_dir.glob('run_*/summary.json')):
        summary = _load_json(summary_path)
        if _data_quality_failure_reason(summary) is None:
            candidates.append((summary_path.parent, summary))
    if not candidates:
        raise ValueError(f'no data-quality-valid runs in {point_dir}')

    physical_failures = [
        item for item in candidates if _is_physical_abort(item[1])
    ]
    if physical_failures:
        reasons = [_physical_abort_reason(summary)
                   for _, summary in physical_failures]
        modal_reason = Counter(reasons).most_common(1)[0][0]
        same_reason = [
            item for item in physical_failures
            if _physical_abort_reason(item[1]) == modal_reason
        ]
        timed = [
            (run_dir, summary, _abort_time(_rows(run_dir / 'telemetry.csv')))
            for run_dir, summary in same_reason
        ]
        finite_times = [time for _, _, time in timed if math.isfinite(time)]
        center = median(finite_times) if finite_times else math.nan
        selected = min(
            timed,
            key=lambda item: (
                abs(item[2] - center) if math.isfinite(item[2])
                and math.isfinite(center) else math.inf,
                item[0].name,
            ),
        )
        return selected[0], selected[1], (
            f'modal physical abort ({modal_reason}); median abort timing'
        ), {item[0].name: item[2] for item in timed}

    nonconvergent = [
        item for item in candidates if _is_nonconvergent(item[1])
    ]
    if nonconvergent:
        reasons = [_nonconvergent_reason(summary)
                   for _, summary in nonconvergent]
        modal_reason = Counter(reasons).most_common(1)[0][0]
        same_reason = [
            item for item in nonconvergent
            if _nonconvergent_reason(item[1]) == modal_reason
        ]
        timed = [
            (run_dir, summary, _abort_time(_rows(run_dir / 'telemetry.csv')))
            for run_dir, summary in same_reason
        ]
        finite_times = [time for _, _, time in timed if math.isfinite(time)]
        center = median(finite_times) if finite_times else math.nan
        selected = min(
            timed,
            key=lambda item: (
                abs(item[2] - center) if math.isfinite(item[2])
                and math.isfinite(center) else math.inf,
                item[0].name,
            ),
        )
        return selected[0], selected[1], (
            f'modal non-convergent outcome ({modal_reason})'
        ), {item[0].name: item[2] for item in timed}

    completed = [
        item for item in candidates
        if bool(item[1].get('va_hold_measurement_complete'))
    ] or candidates
    scores = _median_distance_scores(completed)
    selected = min(completed, key=lambda item: (scores[item[0].name],
                                                 item[0].name))
    return selected[0], selected[1], (
        'minimum multimetric median/MAD distance among valid completed runs'
    ), scores


def _array(rows: list[dict[str, str]], key: str) -> np.ndarray:
    return np.asarray([_finite(row.get(key)) for row in rows], dtype=float)


def _trim(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    start = next((index for index, row in enumerate(rows)
                  if row.get('va_hold_phase', 'inactive') != 'inactive'), 0)
    stop = len(rows)
    for index in range(start, len(rows)):
        row = rows[index]
        complete = _finite(row.get('va_hold_measurement_complete')) > 0.5
        abort = str(row.get('va_hold_abort_reason', 'none')) not in {'', 'none'}
        if complete or abort:
            stop = min(len(rows), index + 2)
            break
        if index > start and row.get('va_hold_phase', 'inactive') == 'inactive':
            # Settle-timeout runs may release into back-transition/landing
            # without publishing a va_hold_abort_reason.  That recovery tail
            # is not part of the fixed-(Va, lambda) physical diagnosis.
            stop = index
            break
    return rows[start:stop]


def _shade_measurement(axis: plt.Axes, rows: list[dict[str, str]]) -> None:
    times = _array(rows, 'time_s')
    active = np.asarray([
        _finite(row.get('va_hold_measurement_active')) > 0.5
        and _finite(row.get('va_hold_measurement_complete')) <= 0.5
        for row in rows
    ])
    indices = np.flatnonzero(active)
    if indices.size:
        starts = [indices[0]]
        ends: list[int] = []
        for left, right in zip(indices, indices[1:]):
            if right != left + 1:
                ends.append(left)
                starts.append(right)
        ends.append(indices[-1])
        for number, (start, end) in enumerate(zip(starts, ends)):
            axis.axvspan(
                times[start], times[end], color='#b2df8a', alpha=0.22,
                label='measurement' if number == 0 else None,
            )
    abort = _abort_time(rows)
    if math.isfinite(abort):
        axis.axvline(abort, color='#d62728', linestyle='--', linewidth=1.0,
                     label='abort')


def plot_run(
    run_dir: Path, summary: dict[str, object], output: Path, title: str
) -> None:
    rows = _trim(_rows(run_dir / 'telemetry.csv'))
    if not rows:
        raise ValueError(f'empty telemetry in {run_dir}')
    t = _array(rows, 'time_s')
    t = t - t[0]
    # Keep plotting helpers on a relative time basis.
    plotted_rows = [dict(row, time_s=str(time)) for row, time in zip(rows, t)]
    fig, axes = plt.subplots(4, 2, figsize=(13, 12), sharex=True,
                             constrained_layout=True)
    axes = axes.ravel()
    for axis in axes:
        _shade_measurement(axis, plotted_rows)
        axis.grid(True, alpha=0.2)

    axes[0].plot(t, _array(rows, 'airspeed_mps'), label='$V_a$')
    axes[0].plot(t, _array(rows, 'va_target_config'), '--', label='target')
    axes[0].set_ylabel('Airspeed (m/s)'); axes[0].legend(fontsize=7)

    axes[1].plot(t, _array(rows, 'altitude_relative_m'), label='h')
    axes[1].plot(t, _array(rows, 'target_altitude_relative_m'), '--',
                 label='target h')
    twin = axes[1].twinx()
    twin.plot(t, _array(rows, 'altitude_error_m'), color='#d62728',
              alpha=0.65, label='$e_h$')
    axes[1].set_ylabel('Altitude (m)'); twin.set_ylabel('$e_h$ (m)')
    axes[1].legend(fontsize=7, loc='upper left')

    axes[2].plot(t, _array(rows, 'lambda_command'), label='command')
    axes[2].plot(t, _array(rows, 'lambda_exec'), '--', label='executed')
    axes[2].plot(t, _array(rows, 'lambda_target_config'), ':', label='target')
    axes[2].set_ylabel('$\\lambda$'); axes[2].legend(fontsize=7)

    lift_omega = np.vstack([_array(rows, f'omega_{index}_rad_s')
                            for index in range(4)])
    mean_omega = np.nanmean(lift_omega, axis=0)
    axes[3].plot(t, _array(rows, 'lift_thrust_n'), label='$T_L$')
    twin = axes[3].twinx()
    twin.plot(t, mean_omega, color='#ff7f00', label='mean $\\omega_L$')
    twin.fill_between(t, np.nanmin(lift_omega, axis=0),
                      np.nanmax(lift_omega, axis=0), color='#ff7f00', alpha=.15)
    axes[3].set_ylabel('$T_L$ (N)'); twin.set_ylabel('$\\omega_L$ (rad/s)')

    axes[4].plot(t, _array(rows, 'lift_power_w'), label='$P_L$ proxy')
    axes[4].set_ylabel('$P_L$ proxy (W)'); axes[4].legend(fontsize=7)

    axes[5].plot(t, _array(rows, 'pusher_thrust_n'), label='$T_P$ model')
    twin = axes[5].twinx()
    twin.plot(t, _array(rows, 'pusher_power_w'), color='#984ea3',
              label='$P_P$ proxy')
    axes[5].set_ylabel('$T_P$ (N)'); twin.set_ylabel('$P_P$ proxy (W)')

    axes[6].plot(t, np.degrees(_array(rows, 'pitch_rad')), label='$\\theta$')
    alpha = _array(rows, 'alpha_est_rad')
    alpha[_array(rows, 'alpha_valid') <= 0.5] = np.nan
    axes[6].plot(t, np.degrees(alpha), label='$\\alpha$')
    axes[6].set_ylabel('Angle (deg)'); axes[6].legend(fontsize=7)

    axes[7].plot(t, _array(rows, 'q_rad_s'), label='q')
    axes[7].plot(t, _array(rows, 'vz_up_mps'), label='$v_z$ up')
    axes[7].set_ylabel('Rate / speed'); axes[7].legend(fontsize=7)
    axes[6].set_xlabel('Time from F1 phase start (s)')
    axes[7].set_xlabel('Time from F1 phase start (s)')
    fig.suptitle(title, fontsize=12)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=200)
    plt.close(fig)


def _plot_composite(
    selections: list[dict[str, object]], output: Path,
) -> None:
    row_defs = (
        ('airspeed_mps', 'va_target_config', r'$V_a$ (m/s)'),
        ('lambda_exec', 'lambda_target_config', r'$\lambda_{exec}$'),
        ('altitude_error_m', None, r'$e_h$ (m)'),
        ('vz_up_mps', None, r'$v_z$ up (m/s)'),
        ('pitch_rad', None, r'$\theta$ (deg)'),
        ('mc_pitch_weight_actual', 'fw_pitch_weight_actual', 'MC/FW weight'),
    )
    columns = len(selections)
    if not columns:
        return
    fig, axes = plt.subplots(
        len(row_defs), columns,
        figsize=(3.7 * columns, 10.8),
        sharex='col',
        constrained_layout=True,
    )
    axes = np.asarray(axes)
    if axes.ndim == 1:
        axes = axes[:, None]

    for column, item in enumerate(selections):
        run_dir = Path(str(item['selected_run_dir']))
        rows = _trim(_rows(run_dir / 'telemetry.csv'))
        if not rows:
            continue
        t = _array(rows, 'time_s')
        t = t - t[0]
        plotted_rows = [
            dict(row, time_s=str(time)) for row, time in zip(rows, t)
        ]
        title = (
            f"{_physical_class_display(item['point_physical_class'])}\n"
            f"Va={item['va_target_mps']:g}, "
            f"lambda={item['lambda_target']:g}, "
            f"{item['selected_run']}"
        )
        axes[0, column].set_title(title, fontsize=9)
        for row_index, (primary, secondary, ylabel) in enumerate(row_defs):
            axis = axes[row_index, column]
            _shade_measurement(axis, plotted_rows)
            axis.grid(True, alpha=0.18)
            if primary == 'pitch_rad':
                axis.plot(t, np.degrees(_array(rows, primary)), color='#1f77b4')
            elif primary == 'mc_pitch_weight_actual':
                axis.plot(t, _array(rows, primary), label='MC', color='#1f77b4')
                axis.plot(t, _array(rows, str(secondary)), label='FW',
                          color='#ff7f0e')
                axis.set_ylim(-0.05, 1.05)
            else:
                axis.plot(t, _array(rows, primary), color='#1f77b4')
                if secondary is not None:
                    axis.plot(t, _array(rows, secondary), '--',
                              color='#4d4d4d', linewidth=1.0)
            if column == 0:
                axis.set_ylabel(ylabel)
            if row_index == len(row_defs) - 1:
                axis.set_xlabel('Time from F1 phase start (s)')
    axes[-1, 0].legend(fontsize=7, loc='upper right')
    fig.suptitle('F1 representative N/B/I/X operating-point time histories',
                 fontsize=12)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=220)
    plt.close(fig)


def generate(
    payload: dict[str, object], output_dir: Path,
    requested: list[tuple[float, float]], manifest_path: Path,
) -> dict[str, object]:
    points = payload.get('points')
    if not isinstance(points, list):
        raise ValueError('summary JSON has no points list')
    manifest: list[dict[str, object]] = []
    for va, lam in requested:
        point = next((item for item in points
                      if abs(_finite(item.get('va_target_mps')) - va) < 1e-9
                      and abs(_finite(item.get('lambda_target')) - lam) < 1e-9),
                     None)
        if not isinstance(point, dict):
            raise ValueError(f'F1 point ({va:g}, {lam:g}) not found')
        source = point.get('source_point_dir')
        if not isinstance(source, str) or not Path(source).is_dir():
            raise ValueError(f'no source_point_dir for ({va:g}, {lam:g})')
        run_dir, summary, reason, scores = select_representative_run(Path(source))
        output = output_dir / f'va_{va:g}_lambda_{lam:g}_{run_dir.name}.png'
        title = (
            f'F1 representative: Va={va:g} m/s, lambda={lam:g}, '
            f'{run_dir.name}; '
            f'physical={_physical_class_display(point.get("physical_class"))}'
        )
        plot_run(run_dir, summary, output, title)
        sanity = _power_sanity([summary])
        manifest.append({
            'va_target_mps': va,
            'lambda_target': lam,
            'point_physical_class': point.get('physical_class'),
            'point_data_quality': point.get('data_quality'),
            'point_passes': point.get('passes'),
            'point_total': point.get('total'),
            'selected_run': run_dir.name,
            'selected_run_dir': str(run_dir.resolve()),
            'selection_reason': reason,
            'candidate_scores_or_abort_times': scores,
            'selected_primary_failure_cause': summary.get(
                'primary_failure_cause'
            ),
            'selected_f1_measurement_pass': summary.get(
                'f1_measurement_pass'
            ),
            'power_sanity': sanity,
            'figure': str(output.resolve()),
        })
    composite = output_dir / 'f1_representative_nbix_timeseries.png'
    _plot_composite(manifest, composite)
    result = {
        'selection_policy': (
            'physical failures: modal abort reason and median abort time; '
            'otherwise minimum multimetric median/MAD distance'
        ),
        'composite_figure': str(composite.resolve()),
        'representatives': manifest,
    }
    manifest_path.write_text(
        json.dumps(_json_safe(result), indent=2, allow_nan=False) + '\n',
        encoding='utf-8',
    )
    return result


def _parse_point(text: str) -> tuple[float, float]:
    left, right = text.split(':', 1)
    return float(left), float(right)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('summary_json', type=Path)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--manifest', type=Path)
    parser.add_argument('--point', action='append', default=[])
    args = parser.parse_args()
    payload = json.loads(args.summary_json.read_text(encoding='utf-8'))
    output_dir = args.output_dir or args.summary_json.parent / 'representative_timeseries'
    requested = [_parse_point(item) for item in (args.point or DEFAULT_POINTS)]
    manifest = args.manifest or args.summary_json.parent / 'f1_representative_runs.json'
    result = generate(payload, output_dir, requested, manifest)
    print(json.dumps(_json_safe(result), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
