"""Evaluate deterministic-gust F2-B runs and aggregate the partial grid.

F2-B deliberately does not reuse the F1 fixed-air-speed occupancy test during
gust exposure.  The target condition must be established first; exposure is
then judged by hard safety, and the final phase by recovery quality.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path
from typing import Iterable

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import BoundaryNorm, ListedColormap


OUTCOME_STYLE = {
    'N': (0, '#4daf4a'),
    'B': (1, '#ffe066'),
    'I': (2, '#ff9f43'),
    'X': (3, '#d73027'),
    'unresolved': (4, '#bdbdbd'),
}
PHYSICAL_ABORTS = {
    'airspeed_hold_runaway',
    'va_hold_altitude_runaway',
    'va_hold_vertical_speed_runaway',
    'px4_failsafe',
}


def _finite(value: object) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def _truth(value: object) -> bool:
    number = _finite(value)
    return math.isfinite(number) and number > 0.5


def _values(rows: Iterable[dict[str, str]], key: str) -> list[float]:
    return [value for row in rows if math.isfinite(value := _finite(row.get(key)))]


def _duration_over(rows: list[dict[str, str]], key: str, limit: float) -> float:
    selected = [
        (_finite(row.get('time_s')), abs(_finite(row.get(key))) > limit)
        for row in rows
    ]
    selected = [(time, active) for time, active in selected if math.isfinite(time)]
    longest = current = 0.0
    previous = math.nan
    for time, active in selected:
        dt = min(max(time - previous, 0.0), 0.1) if math.isfinite(previous) else 0.0
        current = current + dt if active else 0.0
        longest = max(longest, current)
        previous = time
    return longest


def _phase_duration(rows: list[dict[str, str]]) -> float:
    times = _values(rows, 'time_s')
    return max(times) - min(times) if len(times) >= 2 else 0.0


def evaluate_run(csv_path: Path) -> dict[str, object]:
    with csv_path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f'empty telemetry: {csv_path}')

    first = rows[0]
    amplitude = _finite(first.get('gust_amplitude_mps_config'))
    duration_config = _finite(first.get('gust_duration_s_config'))
    recovery_config = _finite(first.get('gust_recovery_s_config'))
    va_target = _finite(first.get('va_target_config'))
    lambda_target = _finite(first.get('lambda_target_config'))
    tolerance = _finite(first.get('lambda_target_tolerance'))
    exposure = [row for row in rows if row.get('gust_phase') == 'gust_exposure']
    recovery = [row for row in rows if row.get('gust_phase') == 'gust_recovery']
    baseline = [row for row in rows if row.get('gust_phase') == 'gust_baseline']
    analysis_rows = exposure + recovery

    bridge_fraction = (
        sum(_truth(row.get('gust_bridge_ready')) for row in analysis_rows)
        / len(analysis_rows) if analysis_rows else 0.0
    )
    command_norm = [
        math.sqrt(sum(_finite(row.get(key)) ** 2 for key in (
            'wind_cmd_e_enu_mps', 'wind_cmd_n_enu_mps',
            'wind_cmd_u_enu_mps',
        ))) for row in exposure
    ]
    actual_norm = [
        math.sqrt(sum(_finite(row.get(key)) ** 2 for key in (
            'wind_actual_e_enu_mps', 'wind_actual_n_enu_mps',
            'wind_actual_u_enu_mps',
        ))) for row in exposure
    ]
    finite_pairs = [
        (command, actual) for command, actual in zip(command_norm, actual_norm)
        if math.isfinite(command) and math.isfinite(actual)
    ]
    profile_rmse = (
        math.sqrt(sum((command - actual) ** 2 for command, actual in finite_pairs)
                  / len(finite_pairs)) if finite_pairs else math.nan
    )
    # The command is updated inside the recorder tick and the acknowledged
    # Gazebo publication is therefore observed one tick later.  For the
    # 20 Hz recorder, derive the admissible alignment error from the maximum
    # slope of the frozen 1-cos pulse instead of applying an unrealistically
    # static 0.1 m/s threshold.
    profile_rmse_limit = max(
        0.1,
        amplitude * math.pi / duration_config * 0.06
        if math.isfinite(amplitude) and math.isfinite(duration_config)
        and duration_config > 0.0 else math.nan,
    )
    peak_actual = max((actual for _, actual in finite_pairs), default=math.nan)
    profile_peak_ok = (
        math.isfinite(amplitude) and math.isfinite(peak_actual)
        and (
            peak_actual <= 0.15 if amplitude <= 0.05
            else peak_actual >= 0.9 * amplitude
        )
    )
    profile_valid = (
        bridge_fraction >= 0.95
        and profile_peak_ok
        and math.isfinite(profile_rmse)
        and profile_rmse <= profile_rmse_limit
    )

    lambda_values = _values(baseline + exposure, 'lambda_exec')
    target_established = (
        bool(baseline) and bool(lambda_values)
        and math.isfinite(lambda_target) and math.isfinite(tolerance)
        and max(abs(value - lambda_target) for value in lambda_values) <= tolerance
    )
    duration_valid = (
        _phase_duration(exposure) >= max(0.0, duration_config - 0.15)
        and _phase_duration(recovery) >= max(0.0, recovery_config - 0.15)
    )

    abort_reasons = {
        str(row.get('va_hold_abort_reason', '')) for row in rows
        if str(row.get('va_hold_abort_reason', '')) not in {'', 'none'}
    }
    failsafe = any(_truth(row.get('failsafe')) for row in analysis_rows)
    hard_altitude = _duration_over(analysis_rows, 'altitude_error_m', 5.0) >= 0.5
    hard_vertical = _duration_over(analysis_rows, 'vz_up_mps', 2.0) >= 0.5
    hard_violation = bool(abort_reasons & PHYSICAL_ABORTS) or failsafe \
        or hard_altitude or hard_vertical

    recovery_times = _values(recovery, 'time_s')
    if recovery_times:
        cutoff = max(recovery_times) - 0.5
        final_recovery = [
            row for row in recovery if _finite(row.get('time_s')) >= cutoff
        ]
    else:
        final_recovery = []
    recovery_good = []
    for row in final_recovery:
        va_error = abs(_finite(row.get('selected_airspeed_mps')) - va_target)
        lambda_error = abs(_finite(row.get('lambda_exec')) - lambda_target)
        recovery_good.append(
            math.isfinite(va_error) and va_error <= 0.6
            and abs(_finite(row.get('altitude_error_m'))) <= 1.0
            and abs(_finite(row.get('vz_up_mps'))) <= 0.5
            and math.isfinite(lambda_error) and lambda_error <= tolerance
            and not _truth(row.get('failsafe'))
        )
    recovery_fraction = (
        sum(recovery_good) / len(recovery_good) if recovery_good else 0.0
    )
    recovery_pass = recovery_fraction >= 0.8

    max_abs_va = max((abs(value - va_target) for value in
                      _values(analysis_rows, 'selected_airspeed_mps')),
                     default=math.nan)
    max_abs_height = max((abs(value) for value in
                          _values(analysis_rows, 'altitude_error_m')),
                         default=math.nan)
    max_abs_vz = max((abs(value) for value in
                      _values(analysis_rows, 'vz_up_mps')),
                     default=math.nan)
    alpha_values = [
        abs(_finite(row.get('alpha_est_rad'))) for row in analysis_rows
        if _truth(row.get('alpha_valid'))
        and math.isfinite(_finite(row.get('alpha_est_rad')))
    ]
    max_abs_alpha = max(alpha_values, default=math.nan)
    soft_degradation = (
        (math.isfinite(max_abs_va) and max_abs_va > 2.0)
        or (math.isfinite(max_abs_height) and max_abs_height > 2.0)
        or (math.isfinite(max_abs_vz) and max_abs_vz > 1.0)
        or (math.isfinite(max_abs_alpha) and max_abs_alpha > math.radians(15.0))
    ) and not hard_violation

    hard_exposure_evidence = (
        hard_violation and bool(exposure) and bridge_fraction >= 0.95
        and (
            amplitude <= 0.05
            or (math.isfinite(peak_actual) and peak_actual > 0.1)
        )
    )
    quality_reasons = []
    if not target_established:
        quality_reasons.append('target_condition_not_established')
    if not hard_exposure_evidence:
        if not exposure or not recovery or not duration_valid:
            quality_reasons.append('gust_window_incomplete')
        if not profile_valid:
            quality_reasons.append('gust_profile_not_verified')
    quality = 'valid' if not quality_reasons else 'protocol_invalid'
    if quality != 'valid':
        outcome = 'unresolved'
    elif hard_violation:
        outcome = 'X'
    elif not recovery_pass:
        outcome = 'I'
    elif soft_degradation:
        outcome = 'B'
    else:
        outcome = 'N'

    return {
        'experiment': 'F2B_deterministic_gust_run',
        'source_csv': str(csv_path.resolve()),
        'telemetry_schema_version': _finite(first.get('schema_version')),
        'condition_id': first.get('condition_id'),
        'va_target_config': va_target,
        'lambda_target_config': lambda_target,
        'gust_amplitude_mps_config': amplitude,
        'f2b_attempt_quality': quality,
        'f2b_attempt_quality_reasons': ','.join(quality_reasons),
        'f2b_run_outcome': outcome,
        'f2b_target_condition_established': target_established,
        'f2b_gust_profile_valid': profile_valid,
        'f2b_gust_bridge_ready_fraction': bridge_fraction,
        'f2b_gust_peak_actual_mps': peak_actual,
        'f2b_gust_command_actual_rmse_mps': profile_rmse,
        'f2b_gust_command_actual_rmse_limit_mps': profile_rmse_limit,
        'f2b_exposure_duration_s': _phase_duration(exposure),
        'f2b_recovery_duration_s': _phase_duration(recovery),
        'f2b_hard_safety_violation': hard_violation,
        'f2b_hard_exposure_evidence': hard_exposure_evidence,
        'f2b_hard_safety_reasons': ','.join(sorted(abort_reasons & PHYSICAL_ABORTS)),
        'f2b_recovery_pass': recovery_pass,
        'f2b_recovery_good_fraction': recovery_fraction,
        'f2b_soft_degradation': soft_degradation,
        'f2b_max_abs_delta_va_mps': max_abs_va,
        'f2b_max_abs_altitude_error_m': max_abs_height,
        'f2b_max_abs_vz_mps': max_abs_vz,
        'f2b_max_abs_alpha_rad': max_abs_alpha,
    }


def classify_point(runs: list[dict[str, object]]) -> dict[str, object]:
    valid = [row for row in runs if row.get('f2b_attempt_quality') == 'valid']
    counts = Counter(str(row.get('f2b_run_outcome')) for row in valid)
    if len(valid) < 5:
        outcome = 'unresolved'
    elif counts['X'] >= 2:
        outcome = 'X'
    elif counts['I'] >= 2 or (counts['N'] + counts['B']) / len(valid) < 0.6:
        outcome = 'I'
    elif counts['B'] >= 2 or counts['X'] + counts['I'] >= 1:
        outcome = 'B'
    else:
        outcome = 'N'
    return {
        'outcome_class': outcome,
        'valid_repeat_count': len(valid),
        'attempt_count': len(runs),
        'protocol_invalid_count': len(runs) - len(valid),
        'run_N_count': counts['N'],
        'run_B_count': counts['B'],
        'run_I_count': counts['I'],
        'run_X_count': counts['X'],
        'recovery_pass_rate': (
            sum(bool(row.get('f2b_recovery_pass')) for row in valid) / len(valid)
            if valid else math.nan
        ),
        'max_abs_delta_va_mps_mean': (
            np.mean([_finite(row.get('f2b_max_abs_delta_va_mps')) for row in valid])
            if valid else math.nan
        ),
        'max_abs_altitude_error_m_mean': (
            np.mean([_finite(row.get('f2b_max_abs_altitude_error_m')) for row in valid])
            if valid else math.nan
        ),
    }


def aggregate(root: Path) -> dict[str, object]:
    points = []
    for point_dir in sorted(root.glob('gust_*/va*_lam*')):
        runs = []
        for path in sorted(point_dir.glob('run_*/summary.json')):
            payload = json.loads(path.read_text(encoding='utf-8'))
            if payload.get('experiment') == 'F2B_deterministic_gust_run':
                runs.append(payload)
        if not runs:
            continue
        row = classify_point(runs)
        row.update({
            'va_target_mps': _finite(runs[0].get('va_target_config')),
            'lambda_target': _finite(runs[0].get('lambda_target_config')),
            'gust_amplitude_mps': _finite(
                runs[0].get('gust_amplitude_mps_config')
            ),
            'source_point_dir': str(point_dir.resolve()),
        })
        points.append(row)
    return {
        'experiment': 'F2B_deterministic_gust_partial_scan',
        'root': str(root.resolve()),
        'point_count': len(points),
        'protocol': {
            'profile': 'one_minus_cosine_pulse',
            'delay_s': 0.5,
            'duration_s': 2.0,
            'recovery_s': 2.0,
            'gust_frame': 'longitudinal_tailwind_world_ENU',
            'valid_repeats_per_cell': 5,
        },
        'points': sorted(points, key=lambda row: (
            _finite(row.get('gust_amplitude_mps')),
            _finite(row.get('va_target_mps')),
            _finite(row.get('lambda_target')),
        )),
    }


def write_aggregate(payload: dict[str, object], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / 'f2b_gust_summary.json').write_text(
        json.dumps(payload, indent=2, allow_nan=False, default=lambda value: float(value)) + '\n',
        encoding='utf-8',
    )
    fields = [
        'gust_amplitude_mps', 'va_target_mps', 'lambda_target',
        'outcome_class', 'valid_repeat_count', 'attempt_count',
        'protocol_invalid_count', 'run_N_count', 'run_B_count',
        'run_I_count', 'run_X_count', 'recovery_pass_rate',
        'max_abs_delta_va_mps_mean', 'max_abs_altitude_error_m_mean',
        'source_point_dir',
    ]
    with (output_dir / 'f2b_gust_points.csv').open(
        'w', newline='', encoding='utf-8'
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(payload['points'])
    _render_maps(payload['points'], output_dir)


def _render_maps(points: list[dict[str, object]], output_dir: Path) -> None:
    amplitudes = sorted({_finite(row['gust_amplitude_mps']) for row in points})
    vas = sorted({_finite(row['va_target_mps']) for row in points})
    lambdas = sorted({_finite(row['lambda_target']) for row in points})
    for key, filename, title, cmap_name in (
        ('outcome_class', 'f2b_gust_outcome_maps.png', 'N/B/I/X outcome', None),
        ('recovery_pass_rate', 'f2b_gust_recovery_pass_rate_maps.png',
         'Recovery pass rate', 'viridis'),
        ('max_abs_altitude_error_m_mean', 'f2b_gust_delta_h_maps.png',
         r'Mean $\max |e_h|$ (m)', 'magma'),
    ):
        figure, axes = plt.subplots(1, len(amplitudes), figsize=(5 * len(amplitudes), 4.8),
                                    constrained_layout=True, squeeze=False)
        image = None
        for axis, amplitude in zip(axes[0], amplitudes):
            matrix = np.full((len(vas), len(lambdas)), np.nan)
            labels = {}
            for row in points:
                if _finite(row['gust_amplitude_mps']) != amplitude:
                    continue
                i = vas.index(_finite(row['va_target_mps']))
                j = lambdas.index(_finite(row['lambda_target']))
                if key == 'outcome_class':
                    label = str(row[key])
                    matrix[i, j] = OUTCOME_STYLE[label][0]
                    labels[(i, j)] = label if label != 'unresolved' else ''
                else:
                    matrix[i, j] = _finite(row[key])
            if key == 'outcome_class':
                cmap = ListedColormap([OUTCOME_STYLE[name][1] for name in
                                       ('N', 'B', 'I', 'X', 'unresolved')])
                norm = BoundaryNorm([-0.5, .5, 1.5, 2.5, 3.5, 4.5], cmap.N)
                image = axis.imshow(np.ma.masked_invalid(matrix), origin='lower',
                                    aspect='auto', cmap=cmap, norm=norm)
                for (i, j), label in labels.items():
                    axis.text(j, i, label, ha='center', va='center', fontweight='bold')
            else:
                cmap = plt.get_cmap(cmap_name).copy()
                cmap.set_bad('#bdbdbd')
                image = axis.imshow(np.ma.masked_invalid(matrix), origin='lower',
                                    aspect='auto', cmap=cmap,
                                    vmin=0 if key == 'recovery_pass_rate' else None,
                                    vmax=1 if key == 'recovery_pass_rate' else None)
            axis.set_xticks(range(len(lambdas)), [f'{value:g}' for value in lambdas])
            axis.set_yticks(range(len(vas)), [f'{value:g}' for value in vas])
            axis.set_xlabel(r'$\lambda$')
            axis.set_ylabel(r'$V_a$ (m/s)')
            axis.set_title(f'$A_g={amplitude:g}$ m/s')
        if image is not None:
            ticks = [0, 1, 2, 3, 4] if key == 'outcome_class' else None
            colorbar = figure.colorbar(image, ax=axes.ravel().tolist(), ticks=ticks)
            if key == 'outcome_class':
                colorbar.ax.set_yticklabels(['N', 'B', 'I', 'X', 'unresolved'])
            else:
                colorbar.set_label(title)
        figure.suptitle(f'F2-B deterministic gust: {title}')
        figure.savefig(output_dir / filename, dpi=220)
        plt.close(figure)


def _json_safe(value: object) -> object:
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    if isinstance(value, (float, np.floating)) and not math.isfinite(float(value)):
        return None
    if isinstance(value, np.generic):
        return value.item()
    return value


def main() -> None:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest='command', required=True)
    run_parser = subparsers.add_parser('run')
    run_parser.add_argument('csv', type=Path)
    run_parser.add_argument('--output', type=Path, required=True)
    aggregate_parser = subparsers.add_parser('aggregate')
    aggregate_parser.add_argument('root', type=Path)
    aggregate_parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    if args.command == 'run':
        payload = _json_safe(evaluate_run(args.csv))
        args.output.write_text(
            json.dumps(payload, indent=2, allow_nan=False) + '\n', encoding='utf-8'
        )
    else:
        payload = _json_safe(aggregate(args.root))
        write_aggregate(payload, args.output_dir)
    print(json.dumps(payload, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
