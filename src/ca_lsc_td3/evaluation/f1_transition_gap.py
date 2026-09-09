"""Diagnose a fixed-Va/fixed-lambda F1 transition-gap point.

Raw telemetry is never modified.  The main-wing support value is a Gazebo
LiftDrag model proxy reconstructed from geometric angle of attack and the SDF
``a0`` alpha offset.  It excludes control-surface deflection and is therefore
reported separately from measured/model rotor thrust.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import fmean
from typing import Iterable

from ca_lsc_td3.physics.airframe_calibration import parse_standard_vtol_sdf


def _finite(value: object) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def _mean(values: Iterable[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return fmean(finite) if finite else math.nan


def _maximum(values: Iterable[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return max(finite, default=math.nan)


def _minimum(values: Iterable[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return min(finite, default=math.nan)


def _fraction(values: Iterable[bool]) -> float:
    items = list(values)
    return sum(items) / len(items) if items else math.nan


def _json_safe(value: object) -> object:
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value


def _read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def _run_number(path: Path) -> int:
    return int(path.parent.name.split('_', 1)[1])


def _measurement(row: dict[str, str]) -> bool:
    return (
        _finite(row.get('va_hold_measurement_active')) > 0.5
        and _finite(row.get('va_hold_measurement_complete')) <= 0.5
    )


def _crossings(values: list[float]) -> int:
    signs = [1 if value > 0.0 else -1 if value < 0.0 else 0 for value in values]
    nonzero = [item for item in signs if item]
    return sum(left != right for left, right in zip(nonzero, nonzero[1:]))


def analyze(
    point_dir: Path,
    sdf_path: Path,
    *,
    fw_pitch_offset_deg: float = 0.0,
) -> tuple[dict[str, object], list[dict[str, object]], list[dict[str, object]]]:
    """Return point summary, per-run metrics, and aligned plot samples."""
    point = Path(point_dir).expanduser().resolve()
    calibration = parse_standard_vtol_sdf(Path(sdf_path))
    weight = calibration.mass_kg * 9.80665
    run_metrics: list[dict[str, object]] = []
    aligned: list[dict[str, object]] = []

    summary_paths = sorted(point.glob('run_*/summary.json'), key=_run_number)
    direct_summary = point / 'summary.json'
    if not summary_paths and direct_summary.exists():
        summary_paths = [direct_summary]
    for summary_path in summary_paths:
        telemetry_path = summary_path.parent / 'telemetry.csv'
        if not telemetry_path.exists():
            continue
        run = (
            _run_number(summary_path)
            if summary_path.parent.name.startswith('run_') else 1
        )
        summary = json.loads(summary_path.read_text(encoding='utf-8'))
        rows = _read_rows(telemetry_path)
        target = _finite(summary.get('lambda_target_config'))
        if not math.isfinite(target):
            target = _mean(_finite(row.get('lambda_target_config')) for row in rows)
        threshold = max(0.0, 0.95 * target)

        reference_altitude = math.nan
        for row in rows:
            if (
                _finite(row.get('lambda_external_active')) > 0.5
                and row.get('va_hold_phase') in {'lambda_settle', 'measurement'}
            ):
                reference_altitude = _finite(row.get('altitude_relative_m'))
                break

        event_time = math.nan
        for row in rows:
            if (
                row.get('va_hold_phase') in {'lambda_settle', 'measurement'}
                and _finite(row.get('lambda_external_active')) > 0.5
                and _finite(row.get('lambda_exec')) >= threshold
            ):
                event_time = _finite(row.get('time_s'))
                break
        if not math.isfinite(event_time):
            continue

        controlled: list[dict[str, object]] = []
        measurement: list[dict[str, object]] = []
        for row in rows:
            time_s = _finite(row.get('time_s'))
            if not math.isfinite(time_s) or time_s < event_time:
                continue
            if row.get('va_hold_phase') not in {'lambda_settle', 'measurement'}:
                continue
            if _finite(row.get('va_hold_measurement_complete')) > 0.5:
                break
            airspeed = _finite(row.get('airspeed_mps'))
            alpha = _finite(row.get('alpha_est_rad'))
            gamma = _finite(row.get('gamma_est_rad'))
            roll = _finite(row.get('roll_rad'))
            pitch = _finite(row.get('pitch_rad'))
            rotor_thrust = _finite(row.get('lift_thrust_n'))
            pusher_thrust = _finite(row.get('pusher_thrust_n'))
            wing_support = math.nan
            rotor_support = math.nan
            pusher_support = math.nan
            total_support = math.nan
            if (
                _finite(row.get('alpha_valid')) > 0.5
                and all(math.isfinite(value) for value in (
                    airspeed, alpha, gamma, roll
                ))
            ):
                lift = (
                    0.5 * calibration.air_density_kg_m3 * airspeed ** 2
                    * calibration.wing_area_m2
                    * calibration.main_wing_cl(alpha)
                )
                wing_support = lift * math.cos(gamma) * math.cos(roll)
            if all(math.isfinite(value) for value in (rotor_thrust, roll, pitch)):
                rotor_support = rotor_thrust * math.cos(roll) * math.cos(pitch)
            if all(math.isfinite(value) for value in (pusher_thrust, pitch)):
                pusher_support = pusher_thrust * math.sin(pitch)
            components = (wing_support, rotor_support, pusher_support)
            if all(math.isfinite(value) for value in components):
                total_support = sum(components)

            pitch_setpoint = _finite(row.get('pitch_setpoint_rad'))
            reconstructed_pitch_setpoint = math.nan
            altitude = _finite(row.get('altitude_relative_m'))
            vertical_speed_up = _finite(row.get('vz_up_mps'))
            kp = _finite(row.get('vt_unload_altitude_pitch_kp_config'))
            kd = _finite(row.get('vt_unload_vertical_speed_pitch_kd_config'))
            lower = _finite(row.get('vt_unload_pitch_min_deg_config'))
            upper = _finite(row.get('vt_unload_pitch_max_deg_config'))
            if all(math.isfinite(value) for value in (
                reference_altitude, altitude, vertical_speed_up,
                kp, kd, lower, upper,
            )):
                degrees = (
                    fw_pitch_offset_deg
                    - kp * (altitude - reference_altitude)
                    - kd * vertical_speed_up
                )
                reconstructed_pitch_setpoint = math.radians(
                    min(max(degrees, lower), upper)
                )

            item: dict[str, object] = {
                'run': run,
                'relative_time_s': time_s - event_time,
                'measurement': _measurement(row),
                'airspeed_mps': airspeed,
                'va_error_mps': _finite(row.get('va_error_mps')),
                'altitude_error_m': _finite(row.get('altitude_error_m')),
                'vertical_speed_up_mps': vertical_speed_up,
                'lambda_exec': _finite(row.get('lambda_exec')),
                'pitch_rad': pitch,
                'pitch_setpoint_rad': pitch_setpoint,
                'pitch_setpoint_reconstructed_rad': reconstructed_pitch_setpoint,
                'mc_pitch_weight_proxy': _finite(
                    row.get('mc_pitch_weight_proxy')
                ),
                'alpha_geometric_rad': alpha,
                'wing_vertical_support_proxy_n': wing_support,
                'lift_rotor_vertical_support_proxy_n': rotor_support,
                'pusher_vertical_support_proxy_n': pusher_support,
                'total_vertical_support_proxy_n': total_support,
                'weight_n': weight,
                'lift_thrust_n': rotor_thrust,
                'pusher_throttle_status': _finite(
                    row.get('pusher_throttle_status')
                ),
                'pusher_pi_integral_mps_s': _finite(
                    row.get('pusher_pi_integral_mps_s')
                ),
                'servo_2': _finite(row.get('servo_2')),
                'lift_collective_thrust_setpoint': _finite(
                    row.get('lift_collective_thrust_setpoint')
                ),
            }
            controlled.append(item)
            aligned.append(item)
            if item['measurement']:
                measurement.append(item)

        chosen = measurement if measurement else controlled
        controlled_va_errors = [
            _finite(item.get('va_error_mps')) for item in controlled
            if math.isfinite(_finite(item.get('va_error_mps')))
        ]
        va_errors = [
            _finite(item.get('va_error_mps')) for item in chosen
            if math.isfinite(_finite(item.get('va_error_mps')))
        ]
        support = [
            _finite(item.get('total_vertical_support_proxy_n')) for item in chosen
            if math.isfinite(_finite(item.get('total_vertical_support_proxy_n')))
        ]
        pitch_tracking = [
            _finite(item.get('pitch_rad')) - _finite(item.get('pitch_setpoint_rad'))
            for item in chosen
            if math.isfinite(_finite(item.get('pitch_rad')))
            and math.isfinite(_finite(item.get('pitch_setpoint_rad')))
        ]
        run_metrics.append({
            'run': run,
            'telemetry_schema_version': summary.get('telemetry_schema_version'),
            'primary_failure_cause': summary.get('primary_failure_cause'),
            'f1_measurement_pass': bool(summary.get('f1_measurement_pass')),
            'measurement_complete': bool(
                summary.get('va_hold_measurement_complete')
            ),
            'measurement_samples': len(measurement),
            'lambda_target_reached_time_s': event_time,
            'measurement_start_delay_from_lambda_target_s': (
                _minimum(
                    _finite(item['relative_time_s']) for item in measurement
                ) if measurement else math.nan
            ),
            'va_error_mean_mps': _mean(va_errors),
            'va_error_max_abs_mps': _maximum(abs(value) for value in va_errors),
            'va_target_crossings': _crossings(va_errors),
            'controlled_phase_va_target_crossings': _crossings(
                controlled_va_errors
            ),
            'altitude_error_max_abs_m': _maximum(
                abs(_finite(item.get('altitude_error_m'))) for item in chosen
            ),
            'pitch_max_abs_rad': _maximum(
                abs(_finite(item.get('pitch_rad'))) for item in chosen
            ),
            'pitch_tracking_rmse_rad': (
                math.sqrt(_mean(value * value for value in pitch_tracking))
                if pitch_tracking else math.nan
            ),
            'mc_pitch_weight_proxy_mean': _mean(
                _finite(item.get('mc_pitch_weight_proxy')) for item in chosen
            ),
            'mc_pitch_weight_proxy_min': _minimum(
                _finite(item.get('mc_pitch_weight_proxy')) for item in chosen
            ),
            'mc_pitch_weight_proxy_max': _maximum(
                _finite(item.get('mc_pitch_weight_proxy')) for item in chosen
            ),
            'alpha_geometric_mean_rad': _mean(
                _finite(item.get('alpha_geometric_rad')) for item in chosen
            ),
            'wing_vertical_support_proxy_mean_n': _mean(
                _finite(item.get('wing_vertical_support_proxy_n')) for item in chosen
            ),
            'lift_rotor_vertical_support_proxy_mean_n': _mean(
                _finite(item.get('lift_rotor_vertical_support_proxy_n'))
                for item in chosen
            ),
            'total_vertical_support_proxy_mean_n': _mean(support),
            'total_vertical_support_proxy_min_n': _minimum(support),
            'support_below_weight_fraction': _fraction(
                value < weight for value in support
            ),
            'pusher_throttle_mean': _mean(
                _finite(item.get('pusher_throttle_status')) for item in chosen
            ),
            'pusher_throttle_max': _maximum(
                _finite(item.get('pusher_throttle_status')) for item in chosen
            ),
            'pusher_throttle_saturation_fraction': _fraction(
                _finite(item.get('pusher_throttle_status')) >= 0.449
                for item in chosen
                if math.isfinite(_finite(item.get('pusher_throttle_status')))
            ),
            'lift_collective_saturation_fraction': _fraction(
                abs(_finite(item.get('lift_collective_thrust_setpoint'))) >= 0.95
                for item in chosen
                if math.isfinite(_finite(
                    item.get('lift_collective_thrust_setpoint')
                ))
            ),
            'servo_2_saturation_fraction': _fraction(
                abs(_finite(item.get('servo_2'))) >= 0.95
                for item in chosen
                if math.isfinite(_finite(item.get('servo_2')))
            ),
        })

    summary = {
        'point_dir': str(point),
        'run_count': len(run_metrics),
        'weight_n': weight,
        'lift_model_kind': (
            'gz_liftdrag_main_wing_proxy_excludes_control_deflection'
        ),
        'lift_drag_a0_semantics': 'alpha_plugin=alpha_geometric+a0',
        'fw_pitch_offset_deg_assumption': fw_pitch_offset_deg,
        'measurement_pass_count': sum(
            bool(item['f1_measurement_pass']) for item in run_metrics
        ),
        'measurement_complete_count': sum(
            bool(item['measurement_complete']) for item in run_metrics
        ),
        'repeated_pusher_saturation': sum(
            _finite(item['pusher_throttle_saturation_fraction']) > 0.05
            for item in run_metrics
        ) >= 2,
        'repeated_lift_collective_saturation': sum(
            _finite(item['lift_collective_saturation_fraction']) > 0.05
            for item in run_metrics
        ) >= 2,
        'repeated_servo_2_saturation': sum(
            _finite(item['servo_2_saturation_fraction']) > 0.05
            for item in run_metrics
        ) >= 2,
        'repeated_support_shortfall': sum(
            _finite(item['support_below_weight_fraction']) > 0.5
            for item in run_metrics
        ) >= 2,
        'mean_va_target_crossings': _mean(
            _finite(item['va_target_crossings']) for item in run_metrics
        ),
        'mean_controlled_phase_va_target_crossings': _mean(
            _finite(item['controlled_phase_va_target_crossings'])
            for item in run_metrics
        ),
        'diagnostic_interpretation': (
            'coupled_height_pitch_airspeed_response_without_actuator_saturation'
        ),
    }
    return summary, run_metrics, aligned


def _write_plot(
    samples: list[dict[str, object]], output: Path, *, va_target: float
) -> None:
    import matplotlib.pyplot as plt

    runs = sorted({int(item['run']) for item in samples})
    fig, axes = plt.subplots(6, 1, figsize=(11, 15), sharex=True)
    for run in runs:
        rows = [item for item in samples if item['run'] == run]
        t = [_finite(item['relative_time_s']) for item in rows]
        label = f'run {run}'
        axes[0].plot(t, [_finite(item['airspeed_mps']) for item in rows], label=label)
        axes[1].plot(t, [_finite(item['altitude_error_m']) for item in rows])
        axes[1].plot(t, [_finite(item['vertical_speed_up_mps']) for item in rows], '--', alpha=.55)
        axes[2].plot(t, [_finite(item['pitch_rad']) for item in rows])
        pitch_sp = [_finite(item['pitch_setpoint_rad']) for item in rows]
        if any(math.isfinite(value) for value in pitch_sp):
            axes[2].plot(t, pitch_sp, '--', alpha=.7)
        else:
            axes[2].plot(
                t,
                [_finite(item['pitch_setpoint_reconstructed_rad']) for item in rows],
                ':', alpha=.6,
            )
        axes[3].plot(t, [_finite(item['alpha_geometric_rad']) for item in rows])
        axes[4].plot(t, [_finite(item['wing_vertical_support_proxy_n']) for item in rows])
        axes[4].plot(t, [_finite(item['lift_rotor_vertical_support_proxy_n']) for item in rows], '--')
        axes[4].plot(t, [_finite(item['total_vertical_support_proxy_n']) for item in rows], ':')
        axes[5].plot(t, [_finite(item['pusher_throttle_status']) for item in rows])
    axes[0].axhline(va_target, color='black', linestyle=':', linewidth=1)
    axes[0].set_ylabel('Va [m/s]')
    axes[0].legend(ncol=min(5, len(runs)), fontsize=8)
    axes[1].set_ylabel('h err / vz [m, m/s]')
    axes[2].set_ylabel('pitch / sp [rad]')
    axes[3].set_ylabel('alpha geom [rad]')
    if samples:
        axes[4].axhline(_finite(samples[0]['weight_n']), color='black', linestyle='-', linewidth=1)
    axes[4].set_ylabel('support proxy [N]')
    axes[5].axhline(0.45, color='black', linestyle=':', linewidth=1)
    axes[5].set_ylabel('pusher throttle')
    axes[5].set_xlabel('time since lambda target reached [s]')
    for axis in axes:
        axis.grid(True, alpha=.25)
    fig.suptitle('F1 transition-gap aligned diagnosis')
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def main() -> None:
    project = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument('point_dir', type=Path)
    parser.add_argument(
        '--sdf', type=Path,
        default=(
            project / 'PX4-Autopilot/Tools/simulation/gz/models'
            / 'standard_vtol/model.sdf'
        ),
    )
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--fw-pitch-offset-deg', type=float, default=0.0)
    arguments = parser.parse_args()
    output = arguments.output_dir or arguments.point_dir / 'transition_gap_diagnostics'
    output.mkdir(parents=True, exist_ok=True)
    summary, runs, samples = analyze(
        arguments.point_dir,
        arguments.sdf,
        fw_pitch_offset_deg=arguments.fw_pitch_offset_deg,
    )
    (output / 'transition_gap_summary.json').write_text(
        json.dumps(_json_safe(summary), indent=2, allow_nan=False) + '\n',
        encoding='utf-8',
    )
    if runs:
        with (output / 'transition_gap_runs.csv').open(
            'w', newline='', encoding='utf-8'
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=tuple(runs[0]))
            writer.writeheader()
            writer.writerows(_json_safe(runs))
    va_target = _mean(
        _finite(row.get('airspeed_mps')) - _finite(row.get('va_error_mps'))
        for row in samples
    )
    _write_plot(samples, output / 'transition_gap_timeseries.png', va_target=va_target)
    print(json.dumps(_json_safe(summary), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
