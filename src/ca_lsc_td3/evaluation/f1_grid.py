"""Aggregate fixed-Va, fixed-lambda Phase-0.75 runs into an F1 map.

F1 is a landscape-building experiment, not a pure pass/fail gate.  This
module therefore preserves all completed, boundary, and aborted points and
classifies each grid cell from repeat-level evidence.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import fmean, median
from typing import Iterable

from ca_lsc_td3.physics.capabilities import wing_vertical_support_capability


RUN_KEYS = (
    'va_hold_measurement_duration_s',
    'va_hold_mean_airspeed_mps',
    'va_hold_std_airspeed_mps',
    'va_hold_max_abs_airspeed_error_mps',
    'va_hold_mean_lambda_exec',
    'va_hold_max_abs_lambda_error',
    'va_hold_measurement_pusher_external_active_fraction',
    'va_hold_mean_down_velocity_command_mps',
    'va_hold_max_abs_down_velocity_command_mps',
    'va_hold_transition_max_abs_down_velocity_command_mps',
    'va_hold_mean_pusher_throttle_command',
    'va_hold_mean_pusher_throttle_status',
    'va_hold_mean_airspeed_filtered_mps',
    'va_hold_mean_pusher_airspeed_error_filtered_mps',
    'va_hold_max_abs_pusher_airspeed_error_filtered_mps',
    'va_hold_mean_pusher_pi_integral_mps_s',
    'va_hold_max_abs_pusher_pi_integral_mps_s',
    'va_hold_mean_pusher_throttle_unsaturated',
    'va_hold_measurement_segment_count',
    'va_hold_mean_lift_thrust_n',
    'va_hold_mean_lift_rotor_omega_rad_s',
    'va_hold_max_lift_rotor_omega_rad_s',
    'va_hold_mean_lift_power_w',
    'va_hold_mean_pusher_power_w',
    'va_hold_mean_total_power_w',
    'va_hold_lift_energy_proxy_j',
    'va_hold_pusher_energy_proxy_j',
    'va_hold_total_energy_proxy_j',
    'va_hold_altitude_rmse_m',
    'va_hold_max_abs_altitude_error_m',
    'va_hold_vertical_speed_rms_mps',
    'va_hold_max_abs_vertical_speed_mps',
    'va_hold_mean_alpha_rad',
    'va_hold_max_abs_alpha_rad',
    'va_hold_mean_pitch_rad',
    'va_hold_mean_pitch_setpoint_rad',
    'va_hold_pitch_tracking_rmse_rad',
    'va_hold_mean_mc_pitch_weight_proxy',
    'va_hold_min_mc_pitch_weight_proxy',
    'va_hold_max_mc_pitch_weight_proxy',
    'va_hold_mean_mc_pitch_weight_actual',
    'va_hold_mean_fw_pitch_weight_actual',
    'va_hold_mc_pitch_weight_lambda_sync_rmse',
    'va_hold_fw_pitch_weight_lambda_sync_rmse',
    'transition_mc_pitch_weight_lambda_sync_rmse',
    'transition_fw_pitch_weight_lambda_sync_rmse',
    'va_hold_pitch_rate_rmse_rad_s',
    'va_hold_pitch_rate_p95_rad_s',
    'va_hold_servo_saturation_fraction',
    'va_hold_lift_rotor_spike_samples',
    'va_hold_lift_rotor_saturation_fraction',
    'va_hold_lift_collective_saturation_fraction',
    'va_hold_pusher_throttle_upper_saturation_fraction',
    'va_hold_pusher_rotor_saturation_fraction',
    'lift_rotor_spike_samples',
    'max_lift_rotor_omega_rad_s',
)

CSV_FIELDS = (
    'point_dir',
    'source_point_dir',
    'va_target_mps',
    'lambda_target',
    'vt_airspeed_blend_mps_config',
    'vt_transition_airspeed_mps_config',
    'classification',
    'outcome_class',
    'physical_class',
    'data_quality',
    'physical_class_reasons',
    'physical_reason_counts',
    'boundary_reasons',
    'unsafe_reasons',
    'data_quality_reasons',
    'single_run_outlier_reasons',
    'unsafe_run_count',
    'unsafe_repeatability',
    'nonconvergent_run_count',
    'protocol_invalid_run_count',
    'attempt_count',
    'valid_repeat_target',
    'protocol_invalid_attempt_rate',
    'passes',
    'total',
    'pass_rate',
    'measurement_complete_rate',
    'abort_rate',
    'pre_measurement_transient_count',
    'pre_measurement_transient_rate',
    'valid_f1_total',
    'measurement_eligible_run_count',
    'valid_f1_passes',
    'valid_f1_pass_rate',
    'valid_f1_abort_rate',
    'primary_failure_causes',
    'va_hold_abort_reasons',
    'eta_l_proxy_mean',
    'eta_l_proxy_std',
    'eta_l_proxy_samples',
    'eta_c_inputs_available',
    'pusher_thrust_power_w_mean',
    'pusher_propulsive_efficiency_proxy_mean',
    'pusher_propulsive_efficiency_proxy_median',
    'pusher_propulsive_efficiency_proxy_p95',
    'pusher_propulsive_efficiency_proxy_max',
    'pusher_power_sanity_samples',
    'pusher_power_sanity_violation_fraction',
    'pusher_power_sanity_pass',
    'pusher_power_sanity_status',
    'power_model_reportable',
    'total_power_identity_max_abs_error_w',
    'telemetry_schema_versions',
    *tuple(f'{key}_mean' for key in RUN_KEYS),
    *tuple(f'{key}_std' for key in RUN_KEYS),
    *tuple(f'{key}_median' for key in RUN_KEYS),
    *tuple(f'{key}_p90' for key in RUN_KEYS),
    *tuple(f'{key}_p95' for key in RUN_KEYS),
    *tuple(f'{key}_min' for key in RUN_KEYS),
    *tuple(f'{key}_max' for key in RUN_KEYS),
)

BOUNDARY_CSV_FIELDS = (
    'va_target_mps',
    'tested_lambda_count',
    'feasible_lambda_count',
    'lambda_max_feasible',
    'lambda_max_feasible_display',
    'lambda_max_feasible_is_lower_bound',
    'maximum_tested_lambda',
    'lambda_first_infeasible_above',
    'lambda_first_unsafe_above',
    'lambda_feasible_interval_display',
    'feasibility_nonmonotonic',
    'lambda_energy_optimal',
    'minimum_feasible_total_power_w',
    'energy_optimum_reportable',
    'lambda_energy_optimal_proxy',
    'minimum_feasible_total_power_proxy_w',
    'lambda_grid_step_max',
    'lambda_grid_has_0p1_resolution',
)

QUALITY_CSV_FIELDS = (
    'va_target_mps',
    'lambda_target',
    'outcome_class',
    'data_quality',
    'data_quality_reasons',
    'protocol_invalid_run_count',
    'attempt_count',
    'valid_repeat_target',
    'protocol_invalid_attempt_rate',
    'nonconvergent_run_count',
    'pre_measurement_transient_count',
    'valid_f1_total',
    'source_point_dir',
)

PROTOCOL_KEYS = (
    'vt_unload_altitude_pitch_kp_config',
    'vt_unload_vertical_speed_pitch_kd_config',
    'vt_unload_pitch_min_deg_config',
    'vt_unload_pitch_max_deg_config',
)
V9_PROTOCOL_KEYS = (
    'va_target_dwell_s_config',
    'lambda_target_dwell_s_config',
    'va_measurement_duration_s_config',
    'va_hold_measurement_min_band_fraction_config',
    'va_hold_measurement_max_abs_airspeed_error_mps_config',
)
V13_PROTOCOL_KEYS = (
    'lambda_attitude_blend_start_config',
    'lambda_attitude_blend_full_config',
)

MIN_VALID_RUNS = 5
POWER_SANITY_EFFICIENCY_TOLERANCE = 1.05
PROTOCOL_FLOAT_DECIMALS = 6
PHYSICAL_ABORT_CAUSES = {
    'va_hold_altitude_runaway',
    'va_hold_vertical_speed_runaway',
    'airspeed_hold_runaway',
    'px4_failsafe',
    'altitude_runaway',
    'aoa_limit',
    'descent_rate_limit',
    'pitch_rate_limit',
}
OUTCOME_CLASSES = (
    'nominal_feasible', 'boundary_feasible', 'nonconvergent',
    'physical_unsafe', 'protocol_invalid', 'missing',
)
PHYSICAL_CLASSES = (
    'nominal_feasible', 'boundary_feasible', 'nonconvergent',
    'physical_unsafe', 'unknown',
)
DATA_QUALITY_CLASSES = (
    'valid', 'valid_with_retries', 'protocol_unstable',
    'protocol_invalid', 'missing',
)

# Soft-envelope thresholds are evaluated per run.  A point is classified as
# boundary only when at least two valid runs repeat the same degradation.  A
# one-off exceedance remains visible in ``single_run_outlier_reasons`` instead
# of silently turning a whole five-run cell into a repeatable physical trend.
SOFT_LIMITS = (
    ('va_hold_altitude_rmse_m', 1.0, 'altitude_rmse'),
    ('va_hold_max_abs_altitude_error_m', 1.5, 'max_altitude_error'),
    ('va_hold_max_abs_vertical_speed_mps', 1.0, 'vertical_speed'),
    ('va_hold_max_abs_alpha_rad', 0.30, 'aoa'),
    ('va_hold_pitch_rate_p95_rad_s', 0.50, 'pitch_rate'),
    ('va_hold_servo_saturation_fraction', 0.05, 'servo_saturation'),
    ('va_hold_lift_rotor_spike_samples', 0.0, 'rotor_spike'),
    ('va_hold_lift_collective_saturation_fraction', 0.05,
     'lift_collective_saturation'),
    ('va_hold_pusher_throttle_upper_saturation_fraction', 0.05,
     'pusher_throttle_saturation'),
)


def _finite(value: object) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def _protocol_keys(summaries: list[dict[str, object]]) -> tuple[str, ...]:
    schema_versions = [
        _finite(item.get('telemetry_schema_version')) for item in summaries
    ]
    maximum = max(
        (value for value in schema_versions if math.isfinite(value)),
        default=1.0,
    )
    return (
        PROTOCOL_KEYS + V9_PROTOCOL_KEYS + V13_PROTOCOL_KEYS
        if maximum >= 13.0 else
        PROTOCOL_KEYS + V9_PROTOCOL_KEYS
        if maximum >= 9.0 else PROTOCOL_KEYS
    )


def _protocol_value(value: object) -> float:
    finite = _finite(value)
    return (
        round(finite, PROTOCOL_FLOAT_DECIMALS)
        if math.isfinite(finite) else math.nan
    )


def _protocol_configuration(
    item: dict[str, object],
    keys: tuple[str, ...],
) -> tuple[float, ...] | None:
    values = tuple(_protocol_value(item.get(key)) for key in keys)
    return values if all(math.isfinite(value) for value in values) else None


def _mean(values: Iterable[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return fmean(finite) if finite else math.nan


def _std(values: Iterable[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return math.nan
    mean = fmean(finite)
    return math.sqrt(fmean((value - mean) ** 2 for value in finite))


def _median(values: Iterable[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return median(finite) if finite else math.nan


def _percentile(values: Iterable[float], probability: float) -> float:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return math.nan
    position = (len(finite) - 1) * probability
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    fraction = position - lower
    return finite[lower] * (1.0 - fraction) + finite[upper] * fraction


def _min(values: Iterable[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return min(finite) if finite else math.nan


def _max(values: Iterable[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return max(finite) if finite else math.nan


def _json_safe(value: object) -> object:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value


def _load_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding='utf-8'))


def _calibration(path: Path | None) -> dict[str, object] | None:
    if path is None or not path.exists():
        return None
    payload = _load_json(path)
    required = ('air_density_kg_m3', 'mass_kg', 'wing_area_m2', 'main_wings')
    return payload if all(key in payload for key in required) else None


def _surface_cl(surface: dict[str, object], alpha: float) -> float:
    alpha_offset = _finite(surface.get('alpha_offset_rad'))
    cla = _finite(surface.get('lift_slope_per_rad'))
    alpha_stall = _finite(surface.get('stall_angle_rad'))
    cla_stall = _finite(surface.get('post_stall_slope_per_rad'))
    if not all(math.isfinite(value) for value in (
        alpha_offset, cla, alpha_stall, cla_stall
    )):
        return math.nan
    effective_alpha = alpha + alpha_offset
    if effective_alpha > alpha_stall:
        return cla * alpha_stall + cla_stall * (effective_alpha - alpha_stall)
    if effective_alpha < -alpha_stall:
        return -cla * alpha_stall + cla_stall * (effective_alpha + alpha_stall)
    return cla * effective_alpha


def _main_wing_cl(calibration: dict[str, object], alpha: float) -> float:
    surfaces = calibration.get('main_wings')
    if not isinstance(surfaces, list):
        return math.nan
    wing_area = _finite(calibration.get('wing_area_m2'))
    if wing_area <= 0.0:
        return math.nan
    weighted = 0.0
    for surface in surfaces:
        if not isinstance(surface, dict):
            return math.nan
        area = _finite(surface.get('area_m2'))
        cl = _surface_cl(surface, alpha)
        if not math.isfinite(area) or not math.isfinite(cl):
            return math.nan
        weighted += area * cl
    return weighted / wing_area


def _measurement_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    segments: list[list[dict[str, str]]] = []
    current: list[dict[str, str]] = []
    for row in rows:
        active = (
            _finite(row.get('va_hold_measurement_active')) > 0.5
            and _finite(row.get('va_hold_measurement_complete')) <= 0.5
        )
        if active:
            current.append(row)
        elif current:
            segments.append(current)
            current = []
    if current:
        segments.append(current)
    return segments[-1] if segments else []


def _air_relative_speed(row: dict[str, object]) -> float:
    """Return the experiment's authoritative air-relative speed.

    Schema 14+ records ``selected_airspeed_mps`` so wind experiments use the
    same ``|V_g - V_w|`` quantity for control, evaluation, and capability
    reconstruction.  Older frozen F1 data predate that column and retain the
    native ``airspeed_mps`` fallback.
    """
    selected = _finite(row.get('selected_airspeed_mps'))
    return selected if math.isfinite(selected) else _finite(
        row.get('airspeed_mps')
    )


def _power_sanity(
    summaries: list[dict[str, object]],
) -> dict[str, object]:
    """Check the model-based pusher power against the thrust-power bound.

    For forward flight, shaft/mechanical power cannot be smaller than the
    useful propulsive power ``T * Va``.  The Gazebo motor plugin uses a static
    ``T=k_T*omega^2, P=Q*omega`` model, so this check deliberately exposes the
    advance-ratio regime where that proxy is no longer physically reportable.
    """
    thrust_power: list[float] = []
    efficiency: list[float] = []
    identity_errors: list[float] = []
    for summary in summaries:
        summary_lift = _finite(summary.get('va_hold_mean_lift_power_w'))
        summary_pusher = _finite(summary.get('va_hold_mean_pusher_power_w'))
        summary_total = _finite(summary.get('va_hold_mean_total_power_w'))
        if all(math.isfinite(value) for value in (
            summary_lift, summary_pusher, summary_total
        )):
            identity_errors.append(abs(
                summary_total - summary_lift - summary_pusher
            ))
        source = summary.get('source_csv')
        if not isinstance(source, str):
            continue
        path = Path(source)
        if not path.exists():
            continue
        for row in _measurement_rows(path):
            va = _air_relative_speed(row)
            thrust = _finite(row.get('pusher_thrust_n'))
            pusher_power = _finite(row.get('pusher_power_w'))
            lift_power = _finite(row.get('lift_power_w'))
            if all(math.isfinite(value) for value in (
                pusher_power, lift_power
            )):
                total = pusher_power + lift_power
                recorded_total = _finite(row.get('total_power_w'))
                if math.isfinite(recorded_total):
                    identity_errors.append(abs(recorded_total - total))
            if not all(math.isfinite(value) for value in (
                va, thrust, pusher_power
            )):
                continue
            if va <= 0.5 or thrust < 0.0 or pusher_power <= 1e-6:
                continue
            alpha = _finite(row.get('alpha_est_rad'))
            alpha_valid = _finite(row.get('alpha_valid')) > 0.5
            axial_speed = va * math.cos(alpha) if (
                alpha_valid and math.isfinite(alpha)
            ) else va
            useful_power = thrust * max(axial_speed, 0.0)
            thrust_power.append(useful_power)
            efficiency.append(useful_power / pusher_power)

    violations = sum(
        value > POWER_SANITY_EFFICIENCY_TOLERANCE for value in efficiency
    )
    violation_fraction = (
        violations / len(efficiency) if efficiency else math.nan
    )
    enough_samples = len(efficiency) >= 30
    passed = bool(
        enough_samples
        and math.isfinite(violation_fraction)
        and violation_fraction <= 0.05
    )
    status = (
        'pass' if passed else
        'model_inconsistent_at_forward_speed' if enough_samples else
        'insufficient_data'
    )
    return {
        'pusher_thrust_power_w_mean': _mean(thrust_power),
        'pusher_propulsive_efficiency_proxy_mean': _mean(efficiency),
        'pusher_propulsive_efficiency_proxy_median': _median(efficiency),
        'pusher_propulsive_efficiency_proxy_p95': _percentile(
            efficiency, 0.95
        ),
        'pusher_propulsive_efficiency_proxy_max': _max(efficiency),
        'pusher_power_sanity_samples': len(efficiency),
        'pusher_power_sanity_violation_fraction': violation_fraction,
        'pusher_power_sanity_pass': passed,
        'pusher_power_sanity_status': status,
        # Passing a necessary energy lower-bound check is not a calibration.
        # The current static motor model has no advance-ratio dependence and
        # therefore remains a diagnostic effort proxy at every F1 cell.
        'power_model_reportable': False,
        'total_power_identity_max_abs_error_w': _max(identity_errors),
    }


def _eta_l_proxy_values(
    summaries: list[dict[str, object]],
    calibration: dict[str, object] | None,
) -> list[float]:
    if calibration is None:
        return []
    density = _finite(calibration.get('air_density_kg_m3'))
    calibration_mass = _finite(calibration.get('mass_kg'))
    area = _finite(calibration.get('wing_area_m2'))
    values: list[float] = []
    for summary in summaries:
        summary_mass = _finite(summary.get('actual_model_mass_kg_config'))
        source = summary.get('source_csv')
        if not isinstance(source, str):
            continue
        path = Path(source)
        if not path.exists():
            continue
        for row in _measurement_rows(path):
            if _finite(row.get('alpha_valid')) <= 0.5:
                continue
            airspeed = _air_relative_speed(row)
            row_mass = _finite(row.get('actual_model_mass_kg_config'))
            mass = next(
                (
                    candidate for candidate in (
                        summary_mass, row_mass, calibration_mass,
                    )
                    if math.isfinite(candidate) and candidate > 0.0
                ),
                math.nan,
            )
            alpha = _finite(row.get('alpha_est_rad'))
            gamma = _finite(row.get('gamma_est_rad'))
            roll = _finite(row.get('roll_rad'))
            cl = _main_wing_cl(calibration, alpha)
            if not all(math.isfinite(value) for value in (airspeed, alpha, gamma, roll, cl)):
                continue
            values.append(
                wing_vertical_support_capability(
                    air_density_kg_m3=density,
                    airspeed_mps=airspeed,
                    wing_area_m2=area,
                    lift_coefficient=cl,
                    flight_path_angle_rad=gamma,
                    roll_angle_rad=roll,
                    estimated_mass_kg=mass,
                )
            )
    return values


def _point_summaries(point: Path) -> list[dict[str, object]]:
    summaries: list[dict[str, object]] = []
    runs = sorted(
        point.glob('run_*/summary.json'),
        key=lambda path: int(path.parent.name.split('_', 1)[1]),
    )
    for run in runs:
        summaries.append(_load_json(run))
    return summaries


def _derived_by_airspeed(points: list[dict[str, object]]) -> list[dict[str, object]]:
    """Extract the feasible unloading boundary and energy optimum per Va."""
    grouped: dict[float, list[dict[str, object]]] = {}
    for point in points:
        va = _finite(point.get('va_target_mps'))
        lam = _finite(point.get('lambda_target'))
        if math.isfinite(va) and math.isfinite(lam):
            grouped.setdefault(va, []).append(point)

    derived: list[dict[str, object]] = []
    feasible_labels = {'nominal_feasible', 'boundary_feasible'}
    for va in sorted(grouped):
        va_points = sorted(
            grouped[va], key=lambda item: _finite(item.get('lambda_target'))
        )
        feasible = [
            item for item in va_points
            if _quality_usable(item.get('data_quality'))
            and item.get('physical_class') in feasible_labels
        ]
        powered_feasible = [
            item for item in feasible
            if math.isfinite(_finite(
                item.get('va_hold_mean_total_power_w_mean')
            ))
        ]
        tested_lambdas = [_finite(item.get('lambda_target')) for item in va_points]
        spacings = [
            right - left
            for left, right in zip(tested_lambdas, tested_lambdas[1:])
        ]
        proxy_energy_optimum = min(
            powered_feasible,
            key=lambda item: _finite(item.get('va_hold_mean_total_power_w_mean')),
            default=None,
        )
        energy_optimum_reportable = bool(powered_feasible) and (
            len(powered_feasible) == len(feasible)
        ) and all(
            bool(item.get('pusher_power_sanity_pass'))
            and bool(item.get('power_model_reportable'))
            for item in powered_feasible
        )
        energy_optimum = proxy_energy_optimum if energy_optimum_reportable else None
        maximum_tested_lambda = max(tested_lambdas, default=math.nan)
        lambda_max_feasible = (
            max(_finite(item.get('lambda_target')) for item in feasible)
            if feasible else math.nan
        )
        lower_bound = bool(
            math.isfinite(lambda_max_feasible)
            and math.isfinite(maximum_tested_lambda)
            and abs(lambda_max_feasible - maximum_tested_lambda) <= 1e-9
            and maximum_tested_lambda < 1.0 - 1e-9
        )
        infeasible_lambdas = sorted(
            _finite(item.get('lambda_target')) for item in va_points
            if _quality_usable(item.get('data_quality'))
            and item.get('physical_class') in {
                'nonconvergent', 'physical_unsafe'
            }
        )
        infeasible_above = [
            value for value in infeasible_lambdas
            if not math.isfinite(lambda_max_feasible)
            or value > lambda_max_feasible + 1e-9
        ]
        first_infeasible_above = min(infeasible_above, default=math.nan)
        unsafe_lambdas = sorted(
            _finite(item.get('lambda_target')) for item in va_points
            if _quality_usable(item.get('data_quality'))
            and item.get('physical_class') == 'physical_unsafe'
        )
        unsafe_above = [
            value for value in unsafe_lambdas
            if not math.isfinite(lambda_max_feasible)
            or value > lambda_max_feasible + 1e-9
        ]
        first_unsafe_above = min(unsafe_above, default=math.nan)
        nonmonotonic = bool(
            math.isfinite(lambda_max_feasible)
            and any(value < lambda_max_feasible - 1e-9
                    for value in infeasible_lambdas)
        )
        interval_display = (
            f'>={lambda_max_feasible:g}' if lower_bound
            else f'[{lambda_max_feasible:g},{first_infeasible_above:g})'
            if math.isfinite(lambda_max_feasible)
            and math.isfinite(first_infeasible_above)
            else f'{lambda_max_feasible:g}'
            if math.isfinite(lambda_max_feasible) else 'unknown'
        )
        derived.append({
            'va_target_mps': va,
            'tested_lambda_count': len(va_points),
            'feasible_lambda_count': len(feasible),
            'lambda_max_feasible': lambda_max_feasible,
            'lambda_max_feasible_display': (
                f'>={lambda_max_feasible:g}' if lower_bound
                else f'{lambda_max_feasible:g}'
                if math.isfinite(lambda_max_feasible) else 'unknown'
            ),
            'lambda_max_feasible_is_lower_bound': lower_bound,
            'maximum_tested_lambda': maximum_tested_lambda,
            'lambda_first_infeasible_above': first_infeasible_above,
            'lambda_first_unsafe_above': first_unsafe_above,
            'lambda_feasible_interval_display': interval_display,
            'feasibility_nonmonotonic': nonmonotonic,
            'lambda_energy_optimal': (
                _finite(energy_optimum.get('lambda_target'))
                if energy_optimum is not None else math.nan
            ),
            'minimum_feasible_total_power_w': (
                _finite(energy_optimum.get('va_hold_mean_total_power_w_mean'))
                if energy_optimum is not None else math.nan
            ),
            'energy_optimum_reportable': energy_optimum_reportable,
            'lambda_energy_optimal_proxy': (
                _finite(proxy_energy_optimum.get('lambda_target'))
                if proxy_energy_optimum is not None else math.nan
            ),
            'minimum_feasible_total_power_proxy_w': (
                _finite(proxy_energy_optimum.get(
                    'va_hold_mean_total_power_w_mean'
                )) if proxy_energy_optimum is not None else math.nan
            ),
            'lambda_grid_step_max': max(spacings, default=math.nan),
            'lambda_grid_has_0p1_resolution': bool(
                len(tested_lambdas) >= 11
                and tested_lambdas[0] <= 1e-9
                and tested_lambdas[-1] >= 1.0 - 1e-9
                and max(spacings, default=math.inf) <= 0.1000001
            ),
        })
    return derived


def _f1_measurement_pass(summary: dict[str, object]) -> bool:
    """Return the F1-specific fixed-condition pass with legacy fallback.

    Newly evaluated runs expose ``f1_measurement_pass`` so the low-speed grid
    is independent of Phase-0.5's fixed 8 m/s acceleration diagnostic.  The
    fallback keeps archived summaries readable until their telemetry is
    explicitly re-evaluated.
    """
    if 'f1_measurement_pass' in summary:
        return bool(summary.get('f1_measurement_pass'))
    return bool(summary.get('phase_0_75_va_hold_pass'))


def _is_protocol_transient(item: dict[str, object]) -> bool:
    return bool(item.get('va_hold_pre_measurement_altitude_transient')) or (
        str(item.get('primary_failure_cause', 'none'))
        == 'pre_measurement_altitude_transient'
    ) or (
        str(item.get('va_hold_abort_reason', 'none'))
        == 'pre_measurement_altitude_transient'
    )


def _target_condition_established(item: dict[str, object]) -> bool:
    """Whether a failure can scientifically be attributed to the F1 cell."""
    if bool(item.get('f1_target_condition_established')):
        return True
    if bool(item.get('va_hold_measurement_complete')):
        return True
    if not bool(item.get('lambda_target_reached')):
        return False
    va_target = _finite(item.get('va_target_config'))
    va_tolerance = _finite(item.get('va_target_tolerance'))
    max_airspeed = _finite(item.get('maximum_transition_airspeed_mps'))
    if not all(math.isfinite(value) for value in (
        va_target, va_tolerance, max_airspeed
    )):
        return False
    return max_airspeed >= va_target - va_tolerance


def _target_airspeed_established(item: dict[str, object]) -> bool:
    va_target = _finite(item.get('va_target_config'))
    va_tolerance = _finite(item.get('va_target_tolerance'))
    max_airspeed = _finite(item.get('maximum_transition_airspeed_mps'))
    if not all(math.isfinite(value) for value in (
        va_target, va_tolerance, max_airspeed
    )):
        return False
    return max_airspeed >= va_target - va_tolerance


def _lambda_progressed_toward_target(item: dict[str, object]) -> bool:
    lambda_target = _finite(item.get('lambda_target_config'))
    tolerance = _finite(item.get('lambda_target_tolerance'))
    max_lambda = _finite(item.get('max_lambda_exec'))
    if not math.isfinite(tolerance):
        tolerance = 0.03
    if not math.isfinite(lambda_target) or not math.isfinite(max_lambda):
        return False
    if lambda_target <= max(tolerance, 1e-6):
        return max_lambda <= max(tolerance, 0.03)
    threshold = min(
        max(0.05, 0.5 * lambda_target),
        max(0.05, lambda_target - 2.0 * tolerance),
    )
    return max_lambda >= threshold


def _sync_failure_reason(item: dict[str, object]) -> str | None:
    schema = _finite(item.get('telemetry_schema_version'))
    if not math.isfinite(schema) or schema < 13.0:
        return None
    measurement_samples = _finite(item.get('va_hold_measurement_samples'))
    target_samples = _finite(item.get('target_lambda_attitude_sync_samples'))
    transition_samples = _finite(
        item.get('transition_lambda_attitude_sync_samples')
    )
    if bool(item.get('va_hold_measurement_complete')) or measurement_samples >= 30:
        if item.get('lambda_attitude_sync_pass') is False:
            return 'measurement_lambda_attitude_sync_failure'
        return None
    if target_samples >= 10:
        if item.get('target_lambda_attitude_sync_pass') is False:
            return 'target_lambda_attitude_sync_failure'
        return None
    if (
        transition_samples >= 10
        or _finite(item.get('lambda_external_active_fraction')) > 0.0
    ):
        if item.get('transition_lambda_attitude_sync_pass') is False:
            return 'transition_lambda_attitude_sync_failure'
    return None


def _hard_physical_causes(item: dict[str, object]) -> set[str]:
    primary = str(item.get('primary_failure_cause', 'none'))
    abort_reason = str(item.get('va_hold_abort_reason', 'none'))
    causes = {primary, abort_reason} - {'', 'none'}
    exact = causes & PHYSICAL_ABORT_CAUSES
    fuzzy = {
        cause for cause in causes
        if any(token in cause for token in (
            'altitude_runaway', 'aoa_limit', 'descent_rate',
            'pitch_rate_limit',
        ))
    }
    return exact | fuzzy


def _target_directed_physical_abort(item: dict[str, object]) -> bool:
    if not _hard_physical_causes(item):
        return False
    if _target_condition_established(item):
        return True
    if _is_protocol_transient(item):
        return False
    return bool(
        _target_airspeed_established(item)
        and _lambda_progressed_toward_target(item)
        and _finite(item.get('lambda_external_active_fraction')) > 0.0
        and item.get('transition_lambda_attitude_sync_pass') is not False
    )


def _data_quality_failure_reason(item: dict[str, object]) -> str | None:
    explicit_quality = str(item.get('f1_attempt_quality', ''))
    if explicit_quality == 'protocol_invalid':
        explicit_reason = str(
            item.get('f1_attempt_quality_reason', 'protocol_invalid')
        )
        if (
            explicit_reason == 'target_condition_not_established'
            and _target_directed_physical_abort(item)
        ):
            return None
        return explicit_reason if explicit_reason not in {'', 'none'} else (
            'protocol_invalid'
        )
    if _is_protocol_transient(item):
        return 'pre_measurement_altitude_transient'
    primary = str(item.get('primary_failure_cause', 'none'))
    protocol_causes = {
        'sequence_or_telemetry_incomplete',
        'pretransition_stability_timeout',
        'pretransition_abort_before_transition',
        'transition_did_not_reach_fw',
        'transition_pusher_undercommand',
    }
    if (
        primary in protocol_causes
        and not bool(item.get('va_hold_measurement_complete'))
        and not _f1_measurement_pass(item)
    ):
        return primary
    if item.get('test_3_telemetry_pass') is False:
        return 'telemetry_incomplete'
    sync_reason = _sync_failure_reason(item)
    if sync_reason is not None:
        return sync_reason
    # A physical limit observed before Va and lambda were established is a
    # protocol/precondition failure, not evidence that the requested F1 cell
    # itself is unsafe.
    if (
        primary not in {'', 'none'}
        and not bool(item.get('va_hold_measurement_complete'))
        and not _target_condition_established(item)
        and not _target_directed_physical_abort(item)
    ):
        return 'target_condition_not_established'
    return None


def _quality_usable(value: object) -> bool:
    return str(value) in {'valid', 'valid_with_retries'}


def _is_abort(item: dict[str, object]) -> bool:
    primary = str(item.get('primary_failure_cause', 'none'))
    abort_reason = str(item.get('va_hold_abort_reason', 'none'))
    return (
        abort_reason not in {'', 'none'}
        or (
            primary not in {'', 'none'}
            and not bool(item.get('va_hold_measurement_complete'))
        )
    )


def _is_physical_abort(item: dict[str, object]) -> bool:
    if _data_quality_failure_reason(item) is not None:
        return False
    return _target_directed_physical_abort(item)


def _is_nonconvergent(item: dict[str, object]) -> bool:
    if _data_quality_failure_reason(item) is not None:
        return False
    if _is_physical_abort(item):
        return False
    primary = str(item.get('primary_failure_cause', 'none'))
    abort_reason = str(item.get('va_hold_abort_reason', 'none'))
    explicit = {
        'va_hold_settle_timeout',
        'measurement_window_interrupted',
        'measurement_window_fragmented',
    }
    return (
        _finite(item.get('va_hold_measurement_segment_count')) > 1.0
        or item.get('va_hold_measurement_protocol_valid') is False
        or primary in explicit
        or abort_reason in explicit
        or not _f1_measurement_pass(item)
    )


def _physical_abort_reason(item: dict[str, object]) -> str:
    abort_reason = str(item.get('va_hold_abort_reason', 'none'))
    primary = str(item.get('primary_failure_cause', 'none'))
    raw = abort_reason if abort_reason not in {'', 'none'} else primary
    aliases = {
        'va_hold_altitude_runaway': 'altitude_runaway',
        'va_hold_vertical_speed_runaway': 'vertical_speed_runaway',
        'airspeed_hold_runaway': 'airspeed_runaway',
        'px4_failsafe': 'px4_failsafe',
    }
    return aliases.get(raw, raw)


def _nonconvergent_reason(item: dict[str, object]) -> str:
    primary = str(item.get('primary_failure_cause', 'none'))
    abort_reason = str(item.get('va_hold_abort_reason', 'none'))
    raw = abort_reason if abort_reason not in {'', 'none'} else primary
    aliases = {
        'va_hold_settle_timeout': 'settle_timeout',
        'measurement_window_interrupted': 'measurement_interrupted',
        'measurement_window_fragmented': 'measurement_fragmented',
    }
    if raw in {'', 'none'}:
        return 'measurement_or_tracking_failure'
    return aliases.get(raw, raw)


def _classify_point(
    summaries: list[dict[str, object]],
    valid_summaries: list[dict[str, object]],
) -> dict[str, object]:
    """Classify physical outcome independently of experiment data quality."""
    invalid_reasons = sorted({
        reason for item in summaries
        for reason in [_data_quality_failure_reason(item)]
        if reason is not None
    })
    transient_count = len(summaries) - len(valid_summaries)
    quality_observations: list[str] = []
    if invalid_reasons:
        quality_observations.extend(
            f'{reason}_excluded' for reason in invalid_reasons
        )
    protocol_invalid_run_count = transient_count
    usable_quality = (
        'valid_with_retries'
        if protocol_invalid_run_count and len(valid_summaries) >= MIN_VALID_RUNS
        else 'protocol_unstable'
        if protocol_invalid_run_count
        else 'valid'
    )

    if len(summaries) < 5:
        return {
            'outcome_class': 'missing',
            'physical_class': 'unknown',
            'data_quality': 'missing',
            'physical_class_reasons': 'incomplete_repeat_set',
            'physical_reason_counts': '',
            'data_quality_reasons': 'missing_run_summaries',
            'single_run_outlier_reasons': '',
            'unsafe_run_count': sum(
                _is_physical_abort(item) for item in valid_summaries
            ),
            'unsafe_repeatability': 'not_classified',
            'nonconvergent_run_count': 0,
            'protocol_invalid_run_count': protocol_invalid_run_count,
        }

    abort_summaries = [
        item for item in summaries if _is_physical_abort(item)
    ]
    abort_reasons = sorted({_physical_abort_reason(item)
                            for item in abort_summaries})
    if len(abort_summaries) >= 2:
        reason_counts = {
            reason: sum(
                _physical_abort_reason(item) == reason
                for item in abort_summaries
            ) for reason in abort_reasons
        }
        return {
            'outcome_class': 'physical_unsafe',
            'physical_class': 'physical_unsafe',
            'data_quality': usable_quality,
            'physical_class_reasons': ','.join(abort_reasons),
            'physical_reason_counts': ','.join(
                f'{reason}:{reason_counts[reason]}' for reason in abort_reasons
            ),
            'data_quality_reasons': ','.join(quality_observations),
            'single_run_outlier_reasons': '',
            'unsafe_run_count': len(abort_summaries),
            'unsafe_repeatability': 'repeated',
            'nonconvergent_run_count': sum(
                _is_nonconvergent(item) for item in valid_summaries
            ),
            'protocol_invalid_run_count': protocol_invalid_run_count,
        }
    if len(valid_summaries) < MIN_VALID_RUNS and not abort_summaries:
        return {
            'outcome_class': 'protocol_invalid',
            'physical_class': 'unknown',
            'data_quality': 'protocol_unstable',
            'physical_class_reasons': 'insufficient_valid_repeat_set',
            'physical_reason_counts': '',
            'data_quality_reasons': ','.join(invalid_reasons),
            'single_run_outlier_reasons': '',
            'unsafe_run_count': 0,
            'unsafe_repeatability': 'not_classified',
            'nonconvergent_run_count': sum(
                _is_nonconvergent(item) for item in valid_summaries
            ),
            'protocol_invalid_run_count': protocol_invalid_run_count,
        }

    repeated_reasons: list[str] = []
    single_reasons: list[str] = []
    reason_counts: dict[str, int] = {}
    if len(abort_summaries) == 1:
        unsafe_reason = _physical_abort_reason(abort_summaries[0])
        repeated_reasons.append('unsafe_outlier')
        single_reasons.append(unsafe_reason)
        reason_counts['unsafe_outlier'] = 1
    physical_metric_summaries = [
        item for item in valid_summaries
        if _f1_measurement_pass(item) and not _is_nonconvergent(item)
    ]
    for key, threshold, reason in SOFT_LIMITS:
        count = sum(
            math.isfinite(_finite(item.get(key)))
            and _finite(item.get(key)) > threshold
            for item in physical_metric_summaries
        )
        if count >= 2:
            repeated_reasons.append(reason)
            reason_counts[reason] = count
        elif count == 1:
            single_reasons.append(reason)

    nonconvergent_summaries = [
        item for item in valid_summaries if _is_nonconvergent(item)
    ]
    nonpassing = len(nonconvergent_summaries)
    valid_pass_rate = (
        (len(valid_summaries) - nonpassing) / len(valid_summaries)
        if valid_summaries else 0.0
    )
    if valid_pass_rate < 0.6:
        nonconvergent_reasons = sorted({
            _nonconvergent_reason(item) for item in nonconvergent_summaries
        })
        return {
            'outcome_class': 'nonconvergent',
            'physical_class': 'nonconvergent',
            'data_quality': usable_quality,
            'physical_class_reasons': ','.join(nonconvergent_reasons),
            'physical_reason_counts': (
                ','.join(
                    f'{reason}:{sum(_nonconvergent_reason(item) == reason for item in nonconvergent_summaries)}'
                    for reason in nonconvergent_reasons
                )
            ),
            'data_quality_reasons': ','.join(quality_observations),
            'single_run_outlier_reasons': '',
            'unsafe_run_count': 0,
            'unsafe_repeatability': 'none',
            'nonconvergent_run_count': nonpassing,
            'protocol_invalid_run_count': protocol_invalid_run_count,
        }
    if nonpassing:
        repeated_reasons.append('nonconvergent_repeat')
        reason_counts['nonconvergent_repeat'] = nonpassing

    return {
        'outcome_class': (
            'boundary_feasible' if repeated_reasons else 'nominal_feasible'
        ),
        'physical_class': (
            'boundary_feasible' if repeated_reasons else 'nominal_feasible'
        ),
        'data_quality': usable_quality,
        'physical_class_reasons': ','.join(sorted(set(repeated_reasons))),
        'physical_reason_counts': ','.join(
            f'{reason}:{reason_counts[reason]}'
            for reason in sorted(reason_counts)
        ),
        'data_quality_reasons': ','.join(quality_observations),
        'single_run_outlier_reasons': ','.join(sorted(set(single_reasons))),
        'unsafe_run_count': len(abort_summaries),
        'unsafe_repeatability': (
            'single_run' if len(abort_summaries) == 1 else 'none'
        ),
        'nonconvergent_run_count': nonpassing,
        'protocol_invalid_run_count': protocol_invalid_run_count,
    }


def _legacy_classification(classification: dict[str, object]) -> str:
    """Retain the former combined field for archived consumers only."""
    return str(classification.get('outcome_class', 'missing'))


def _explicit_reason_fields(
    classification: dict[str, object],
) -> dict[str, object]:
    reasons = str(classification.get('physical_class_reasons', ''))
    unsafe_outlier = str(classification.get('single_run_outlier_reasons', ''))
    return {
        'boundary_reasons': (
            reasons if classification.get('physical_class')
            == 'boundary_feasible' else ''
        ),
        'unsafe_reasons': (
            reasons
            if classification.get('physical_class') == 'physical_unsafe'
            else unsafe_outlier
            if _finite(classification.get('unsafe_run_count')) > 0
            else ''
        ),
    }


def aggregate(root: Path, calibration_path: Path | None = None) -> dict[str, object]:
    root = root.expanduser().resolve()
    calibration = _calibration(calibration_path)
    repeat_files = sorted(root.rglob('repeat_summary.json'))
    points: list[dict[str, object]] = []
    for repeat_file in repeat_files:
        point = repeat_file.parent
        repeat = _load_json(repeat_file)
        summaries = _point_summaries(point)
        if not summaries:
            continue
        va_target = _finite(summaries[0].get('va_target_config'))
        lambda_target = _finite(summaries[0].get('lambda_target_config'))
        total = len(summaries)
        passes = sum(_f1_measurement_pass(item) for item in summaries)
        measurement_complete = sum(
            bool(item.get('va_hold_measurement_complete')) for item in summaries
        )
        pre_measurement_transient = [
            item for item in summaries if _is_protocol_transient(item)
        ]
        quality_invalid = [
            item for item in summaries
            if _data_quality_failure_reason(item) is not None
        ]
        quality_invalid_ids = {id(item) for item in quality_invalid}
        pre_measurement_transient_count = len(pre_measurement_transient)
        valid_f1_summaries = [
            item for item in summaries
            if id(item) not in quality_invalid_ids
        ]
        measurement_eligible_summaries = [
            item for item in valid_f1_summaries
            if _f1_measurement_pass(item) and not _is_nonconvergent(item)
        ]
        valid_f1_total = len(valid_f1_summaries)
        valid_f1_passes = sum(
            _f1_measurement_pass(item) for item in valid_f1_summaries
        )

        aborts = sum(_is_abort(item) for item in summaries)
        valid_f1_aborts = sum(
            _is_abort(item) for item in valid_f1_summaries
        )
        physical_measurement_complete = sum(
            bool(item.get('va_hold_measurement_complete'))
            for item in valid_f1_summaries
        )
        metrics: dict[str, float] = {}
        for key in RUN_KEYS:
            # Fixed-condition physical statistics exclude runs rejected for a
            # pre-measurement protocol transient.  This prevents startup data
            # quality from contaminating the F1 landscape itself.
            values = [
                _finite(item.get(key))
                for item in measurement_eligible_summaries
            ]
            metrics[f'{key}_mean'] = _mean(values)
            metrics[f'{key}_std'] = _std(values)
            metrics[f'{key}_median'] = _median(values)
            metrics[f'{key}_p90'] = _percentile(values, 0.90)
            metrics[f'{key}_p95'] = _percentile(values, 0.95)
            metrics[f'{key}_min'] = _min(values)
            metrics[f'{key}_max'] = _max(values)

        eta_l = _eta_l_proxy_values(
            measurement_eligible_summaries, calibration
        )
        pass_rate = passes / total if total else 0.0
        measurement_rate = measurement_complete / total if total else 0.0
        abort_rate = aborts / total if total else 0.0
        valid_f1_pass_rate = (
            valid_f1_passes / valid_f1_total if valid_f1_total else 0.0
        )
        valid_f1_measurement_rate = (
            physical_measurement_complete / valid_f1_total
            if valid_f1_total else 0.0
        )
        valid_f1_abort_rate = (
            valid_f1_aborts / valid_f1_total if valid_f1_total else 0.0
        )
        pre_measurement_transient_rate = (
            pre_measurement_transient_count / total if total else 0.0
        )
        classification = _classify_point(summaries, valid_f1_summaries)
        power_sanity = _power_sanity(measurement_eligible_summaries)
        point_row: dict[str, object] = {
            'point_dir': str(point.relative_to(root)),
            'source_point_dir': str(point.resolve()),
            'va_target_mps': va_target,
            'lambda_target': lambda_target,
            'vt_airspeed_blend_mps_config': _mean(
                _finite(item.get('vt_airspeed_blend_mps_config'))
                for item in summaries
            ),
            'vt_transition_airspeed_mps_config': _mean(
                _finite(item.get('vt_transition_airspeed_mps_config'))
                for item in summaries
            ),
            'classification': _legacy_classification(classification),
            **classification,
            **_explicit_reason_fields(classification),
            'passes': passes,
            'total': total,
            'pass_rate': pass_rate,
            'measurement_complete_rate': measurement_rate,
            'abort_rate': abort_rate,
            'pre_measurement_transient_count': (
                pre_measurement_transient_count
            ),
            'pre_measurement_transient_rate': (
                pre_measurement_transient_rate
            ),
            'valid_f1_total': valid_f1_total,
            'attempt_count': total,
            'valid_repeat_target': MIN_VALID_RUNS,
            'protocol_invalid_attempt_rate': (
                len(quality_invalid) / total if total else 0.0
            ),
            'measurement_eligible_run_count': len(
                measurement_eligible_summaries
            ),
            'valid_f1_passes': valid_f1_passes,
            'valid_f1_pass_rate': valid_f1_pass_rate,
            'valid_f1_abort_rate': valid_f1_abort_rate,
            'primary_failure_causes': ','.join(sorted({
                str(item.get('primary_failure_cause', 'none'))
                for item in summaries
                if str(item.get('primary_failure_cause', 'none')) not in {'', 'none'}
            })),
            'va_hold_abort_reasons': ','.join(sorted({
                str(item.get('va_hold_abort_reason', 'none'))
                for item in summaries
                if str(item.get('va_hold_abort_reason', 'none')) not in {'', 'none'}
            })),
            'eta_l_proxy_mean': _mean(eta_l),
            'eta_l_proxy_std': _std(eta_l),
            'eta_l_proxy_samples': len(eta_l),
            'eta_c_inputs_available': False,
            'telemetry_schema_versions': ','.join(
                f'{value:g}' for value in sorted({
                    _finite(item.get('telemetry_schema_version'))
                    for item in summaries
                    if math.isfinite(_finite(item.get('telemetry_schema_version')))
                })
            ),
            **power_sanity,
            **metrics,
        }
        points.append(point_row)
    points.sort(key=lambda item: (
        _finite(item.get('va_target_mps')),
        _finite(item.get('lambda_target')),
        str(item.get('point_dir')),
    ))
    all_summaries = [
        summary
        for repeat_file in repeat_files
        for summary in _point_summaries(repeat_file.parent)
    ]
    schema_versions = sorted({
        _finite(item.get('telemetry_schema_version'))
        for item in all_summaries
        if math.isfinite(_finite(item.get('telemetry_schema_version')))
    })
    protocol_keys = _protocol_keys(all_summaries)
    protocol_configurations = sorted({
        configuration
        for item in all_summaries
        for configuration in [_protocol_configuration(item, protocol_keys)]
        if configuration is not None
    })
    derived = _derived_by_airspeed(points)
    return {
        'experiment': 'F1_Va_lambda_capability_map',
        'root': str(root),
        'point_count': len(points),
        'telemetry_schema_versions': schema_versions,
        'protocol_keys': list(protocol_keys),
        'protocol_configurations': [
            dict(zip(protocol_keys, values))
            for values in protocol_configurations
        ],
        'protocol_consistent': bool(
            len(schema_versions) == 1
            and len(protocol_configurations) == 1
        ),
        'classification_counts': {
            label: sum(item['classification'] == label for item in points)
            for label in OUTCOME_CLASSES
        },
        'outcome_class_counts': {
            label: sum(item['outcome_class'] == label for item in points)
            for label in OUTCOME_CLASSES
        },
        'physical_class_counts': {
            label: sum(item['physical_class'] == label for item in points)
            for label in PHYSICAL_CLASSES
        },
        'data_quality_counts': {
            label: sum(item['data_quality'] == label for item in points)
            for label in DATA_QUALITY_CLASSES
        },
        'power_sanity_counts': {
            label: sum(item['pusher_power_sanity_status'] == label
                       for item in points)
            for label in (
                'pass', 'model_inconsistent_at_forward_speed',
                'insufficient_data'
            )
        },
        'run_statistic_definition': (
            'Across data-quality-valid runs only; std is population std and '
            'P90/P95 use linear interpolation at (n-1)*p.'
        ),
        'eta_l_proxy_note': (
            'Computed offline from measurement-window selected air-relative '
            'speed (native airspeed fallback for pre-schema-14 data), alpha, '
            'gamma, roll, each run actual_model_mass_kg_config (calibration '
            'mass fallback), and model-based SDF CL(alpha). Use as a '
            'convenience diagnostic; formal eta_L validation remains F3.'
        ),
        'eta_l_proxy_reportable': False,
        'eta_c_note': (
            'eta_C is defined in ca_lsc_td3.physics.capabilities, but F1 '
            'telemetry does not yet contain shadow FW pitch moment increment; '
            'do not report eta_C values from this F1 aggregator.'
        ),
        'power_sanity_note': (
            'pusher_power_w is a static motor reaction-torque proxy. Cells '
            'with T_p*V_axial/P_p > 1.05 fail the mechanical energy lower '
            'bound and must not support paper energy claims.'
        ),
        'power_model_reportable': False,
        'derived_by_airspeed': derived,
        'points': points,
    }


def aggregate_roots(
    roots: list[Path], calibration_path: Path | None = None,
    duplicate_policy: str = 'error',
) -> dict[str, object]:
    """Merge disjoint F1 batches without copying or symlinking raw data."""
    if duplicate_policy not in {'error', 'last'}:
        raise ValueError('duplicate_policy must be error or last')
    if len(roots) == 1:
        return aggregate(roots[0], calibration_path)
    batch_results = [aggregate(root, calibration_path) for root in roots]
    points: list[dict[str, object]] = []
    seen: dict[tuple[float, float], int] = {}
    for root, result in zip(roots, batch_results):
        for original in result['points']:  # type: ignore[union-attr]
            point = dict(original)
            key = (
                _finite(point.get('va_target_mps')),
                _finite(point.get('lambda_target')),
            )
            if key in seen and duplicate_policy == 'error':
                raise ValueError(
                    f'duplicate F1 point Va={key[0]:g}, lambda={key[1]:g} '
                    f'across input batches'
                )
            point['point_dir'] = f'{root.name}/{point["point_dir"]}'
            if key in seen:
                points[seen[key]] = point
            else:
                seen[key] = len(points)
                points.append(point)
    points.sort(key=lambda item: (
        _finite(item.get('va_target_mps')),
        _finite(item.get('lambda_target')),
    ))
    schema_versions = sorted({
        _finite(value)
        for result in batch_results
        for value in result.get('telemetry_schema_versions', [])
        if math.isfinite(_finite(value))
    })
    maximum_schema = max(schema_versions, default=1.0)
    protocol_keys = (
        PROTOCOL_KEYS + V9_PROTOCOL_KEYS + V13_PROTOCOL_KEYS
        if maximum_schema >= 13.0 else
        PROTOCOL_KEYS + V9_PROTOCOL_KEYS
        if maximum_schema >= 9.0 else PROTOCOL_KEYS
    )
    protocol_tuples = sorted({
        configuration
        for result in batch_results
        for config in result.get('protocol_configurations', [])
        if isinstance(config, dict)
        for configuration in [_protocol_configuration(config, protocol_keys)]
        if configuration is not None
    })
    return {
        'experiment': 'F1_Va_lambda_capability_map',
        'roots': [str(root.expanduser().resolve()) for root in roots],
        'point_count': len(points),
        'telemetry_schema_versions': schema_versions,
        'protocol_keys': list(protocol_keys),
        'protocol_configurations': [
            dict(zip(protocol_keys, values)) for values in protocol_tuples
        ],
        'protocol_consistent': bool(
            len(schema_versions) == 1 and len(protocol_tuples) == 1
        ),
        'classification_counts': {
            label: sum(item['classification'] == label for item in points)
            for label in OUTCOME_CLASSES
        },
        'outcome_class_counts': {
            label: sum(item['outcome_class'] == label for item in points)
            for label in OUTCOME_CLASSES
        },
        'physical_class_counts': {
            label: sum(item['physical_class'] == label for item in points)
            for label in PHYSICAL_CLASSES
        },
        'data_quality_counts': {
            label: sum(item['data_quality'] == label for item in points)
            for label in DATA_QUALITY_CLASSES
        },
        'power_sanity_counts': {
            label: sum(item['pusher_power_sanity_status'] == label
                       for item in points)
            for label in (
                'pass', 'model_inconsistent_at_forward_speed',
                'insufficient_data'
            )
        },
        'run_statistic_definition': batch_results[0][
            'run_statistic_definition'
        ],
        'eta_l_proxy_note': batch_results[0]['eta_l_proxy_note'],
        'eta_l_proxy_reportable': False,
        'eta_c_note': batch_results[0]['eta_c_note'],
        'power_sanity_note': batch_results[0]['power_sanity_note'],
        'power_model_reportable': False,
        'derived_by_airspeed': _derived_by_airspeed(points),
        'points': points,
    }


def write_csv(points: list[dict[str, object]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, extrasaction='ignore')
        writer.writeheader()
        for point in points:
            writer.writerow({key: _json_safe(point.get(key)) for key in CSV_FIELDS})


def write_boundary_csv(rows: list[dict[str, object]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=BOUNDARY_CSV_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({
                key: _json_safe(row.get(key)) for key in BOUNDARY_CSV_FIELDS
            })


def write_quality_csv(points: list[dict[str, object]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=QUALITY_CSV_FIELDS)
        writer.writeheader()
        for point in points:
            if (
                point.get('data_quality') == 'valid'
                and not point.get('data_quality_reasons')
            ):
                continue
            writer.writerow({
                key: _json_safe(point.get(key)) for key in QUALITY_CSV_FIELDS
            })


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('roots', type=Path, nargs='+')
    parser.add_argument(
        '--calibration',
        type=Path,
        default=Path('data/calibration/model_based/airframe_calibration.json'),
    )
    parser.add_argument('--output-json', type=Path)
    parser.add_argument('--output-csv', type=Path)
    parser.add_argument('--output-boundary-csv', type=Path)
    parser.add_argument('--output-quality-csv', type=Path)
    parser.add_argument(
        '--duplicate-policy', choices=('error', 'last'), default='error',
        help='Use last only for an explicit independently rerun replacement.',
    )
    args = parser.parse_args()

    result = aggregate_roots(
        args.roots, args.calibration, args.duplicate_policy
    )
    default_root = args.roots[0]
    output_json = args.output_json or default_root / 'f1_grid_summary.json'
    output_csv = args.output_csv or default_root / 'f1_grid_points.csv'
    output_boundary_csv = (
        args.output_boundary_csv or default_root / 'f1_boundaries.csv'
    )
    output_quality_csv = (
        args.output_quality_csv
        or output_json.parent / 'f1_data_quality_cases.csv'
    )
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(
        json.dumps(_json_safe(result), indent=2, allow_nan=False) + '\n',
        encoding='utf-8',
    )
    write_csv(result['points'], output_csv)  # type: ignore[arg-type]
    write_boundary_csv(  # type: ignore[arg-type]
        result['derived_by_airspeed'], output_boundary_csv
    )
    write_quality_csv(  # type: ignore[arg-type]
        result['points'], output_quality_csv
    )
    print(json.dumps(_json_safe(result), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
