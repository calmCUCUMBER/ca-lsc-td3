"""Evaluate one nominal transition CSV without requiring ROS."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import fmean


FRESHNESS_LIMIT_S = 0.5
LAMBDA_START_AIRSPEED_MPS = 8.0
STABILITY_ALTITUDE_TOLERANCE_M = 1.0
STABILITY_VERTICAL_SPEED_TOLERANCE_MPS = 0.2
STABILITY_GROUNDSPEED_TOLERANCE_MPS = 0.2
STABILITY_DWELL_S = 2.0
PUSHER_MINIMUM_PEAK_SETPOINT = 0.4
PUSHER_MAXIMUM_TIME_TO_8_MPS_S = 4.0
LIFT_ROTOR_SPIKE_THRESHOLD_RAD_S = 1400.0
LIFT_ROTOR_SATURATION_THRESHOLD_RAD_S = 0.95 * 1500.0
PUSHER_ROTOR_SATURATION_THRESHOLD_RAD_S = 0.95 * 3500.0
PUSHER_THROTTLE_UPPER_SATURATION = 0.449


def _number(row: dict[str, str], key: str) -> float:
    try:
        value = float(row.get(key, 'nan'))
    except (TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def _first_number(row: dict[str, str], keys: tuple[str, ...]) -> float:
    for key in keys:
        value = _number(row, key)
        if math.isfinite(value):
            return value
    return math.nan


def _values(rows: list[dict[str, str]], key: str) -> list[float]:
    return [
        value for row in rows for value in [_number(row, key)]
        if math.isfinite(value)
    ]


def _airspeed_value(row: dict[str, str]) -> float:
    """Return the airspeed used by the active experiment protocol.

    Schema 14 wind-qualification runs can select the reconstructed
    ground-minus-wind relative speed for Va hold.  Older F1/F0 logs have no
    selected-airpeed column, so they naturally fall back to the original PX4
    `airspeed_mps` field.
    """
    selected = _number(row, 'selected_airspeed_mps')
    return selected if math.isfinite(selected) else _number(row, 'airspeed_mps')


def _airspeed_values(rows: list[dict[str, str]]) -> list[float]:
    return [
        value for row in rows for value in [_airspeed_value(row)]
        if math.isfinite(value)
    ]


def _mean(values: list[float]) -> float:
    return fmean(values) if values else math.nan


def _std(values: list[float]) -> float:
    if not values:
        return math.nan
    mean = fmean(values)
    return math.sqrt(fmean((value - mean) ** 2 for value in values))


def _rmse(values: list[float]) -> float:
    return math.sqrt(fmean(value * value for value in values)) if values else math.nan


def _percentile(values: list[float], probability: float) -> float:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return math.nan
    position = (len(finite) - 1) * probability
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    fraction = position - lower
    return finite[lower] * (1.0 - fraction) + finite[upper] * fraction


def _trapezoid(rows: list[dict[str, str]], key: str) -> float:
    """Integrate a telemetry field over time, skipping invalid intervals."""
    total = 0.0
    valid_intervals = 0
    for left, right in zip(rows, rows[1:]):
        t0 = _number(left, 'time_s')
        t1 = _number(right, 'time_s')
        y0 = _number(left, key)
        y1 = _number(right, key)
        if not all(math.isfinite(value) for value in (t0, t1, y0, y1)):
            continue
        dt = t1 - t0
        if dt <= 0.0:
            continue
        total += 0.5 * (y0 + y1) * dt
        valid_intervals += 1
    return total if valid_intervals else math.nan


def _energy(rows: list[dict[str, str]]) -> tuple[float, float, float]:
    lift = _trapezoid(rows, 'lift_power_w')
    pusher = _trapezoid(rows, 'pusher_power_w')
    total = lift + pusher if math.isfinite(lift) and math.isfinite(pusher) else math.nan
    return lift, pusher, total


def _va_measurement_segments(
    rows: list[dict[str, str]],
) -> list[list[dict[str, str]]]:
    """Split Phase-0.75 fixed-Va measurement rows into contiguous segments.

    A recorder may briefly leave and then re-enter the measurement phase before
    declaring the run complete.  The experiment protocol treats the final
    uninterrupted measurement dwell as the usable sample window; earlier
    partial windows are transients and must not be merged with the final dwell.
    """
    segments: list[list[dict[str, str]]] = []
    current: list[dict[str, str]] = []
    for row in rows:
        active = (
            _number(row, 'va_hold_measurement_active') > 0.5
            and _number(row, 'va_hold_measurement_complete') <= 0.5
        )
        if active:
            current.append(row)
        elif current:
            segments.append(current)
            current = []
    if current:
        segments.append(current)
    return segments


def _contains_in_order(sequence: list[object], required: tuple[object, ...]) -> bool:
    required_index = 0
    for value in sequence:
        if value == required[required_index]:
            required_index += 1
            if required_index == len(required):
                return True
    return False


def _json_safe(value: object) -> object:
    """Convert non-finite floats to JSON null for strict output."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value


def _stability_flags(
    row: dict[str, str],
    target_altitude_m: float,
) -> tuple[bool, bool, bool, bool]:
    altitude = _first_number(row, ('altitude_relative_m', 'altitude_m'))
    vertical_speed = _number(row, 'vz_up_mps')
    groundspeed = _number(row, 'groundspeed_mps')
    height_ok = (
        math.isfinite(altitude)
        and math.isfinite(target_altitude_m)
        and abs(altitude - target_altitude_m)
        <= STABILITY_ALTITUDE_TOLERANCE_M
    )
    vertical_speed_ok = (
        math.isfinite(vertical_speed)
        and abs(vertical_speed) <= STABILITY_VERTICAL_SPEED_TOLERANCE_MPS
    )
    groundspeed_ok = (
        math.isfinite(groundspeed)
        and groundspeed <= STABILITY_GROUNDSPEED_TOLERANCE_MPS
    )
    return (
        height_ok,
        vertical_speed_ok,
        groundspeed_ok,
        height_ok and vertical_speed_ok and groundspeed_ok,
    )


def _stable_row(row: dict[str, str], target_altitude_m: float) -> bool:
    return _stability_flags(row, target_altitude_m)[3]


def _stability_statistics(
    rows: list[dict[str, str]],
    target_altitude_m: float,
) -> dict[str, float]:
    if not rows:
        return {
            'height_gate_fraction': 0.0,
            'vz_gate_fraction': 0.0,
            'groundspeed_gate_fraction': 0.0,
            'stability_gate_fraction': 0.0,
            'max_stability_dwell_s': 0.0,
        }

    counts = [0, 0, 0, 0]
    stable_since = math.nan
    max_dwell = 0.0
    for row in rows:
        flags = _stability_flags(row, target_altitude_m)
        for index, flag in enumerate(flags):
            counts[index] += int(flag)
        time_s = _number(row, 'time_s')
        if flags[3] and math.isfinite(time_s):
            if not math.isfinite(stable_since):
                stable_since = time_s
            max_dwell = max(max_dwell, time_s - stable_since)
        else:
            stable_since = math.nan
    total = len(rows)
    return {
        'height_gate_fraction': counts[0] / total,
        'vz_gate_fraction': counts[1] / total,
        'groundspeed_gate_fraction': counts[2] / total,
        'stability_gate_fraction': counts[3] / total,
        'max_stability_dwell_s': max_dwell,
    }


def _final_stability_dwell(
    hold_rows: list[dict[str, str]],
    transition_row: dict[str, str],
    target_altitude_m: float,
) -> float:
    stable_since = math.nan
    for row in hold_rows:
        time_s = _number(row, 'time_s')
        if _stable_row(row, target_altitude_m) and math.isfinite(time_s):
            if not math.isfinite(stable_since):
                stable_since = time_s
        else:
            stable_since = math.nan
    transition_time = _number(transition_row, 'time_s')
    if (
        math.isfinite(stable_since)
        and math.isfinite(transition_time)
        and _stable_row(transition_row, target_altitude_m)
    ):
        return max(0.0, transition_time - stable_since)
    return 0.0


def evaluate(path: Path) -> dict[str, object]:
    with path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f'no telemetry rows in {path}')

    states = [row.get('command_state', '').split('|', 1)[0] for row in rows]
    required_states = ('HOLD_MC', 'TRANSITION_FW', 'HOLD_FW')
    state_sequence_ok = _contains_in_order(states, required_states)
    vtol_states = [
        int(value)
        for row in rows
        for value in [_number(row, 'vtol_state')]
        if math.isfinite(value)
    ]
    # MAV_VTOL_STATE: 3=MC, 1=transition-to-FW, 4=FW.
    vtol_state_sequence_ok = _contains_in_order(vtol_states, (3, 1, 4))
    transition_indices = [
        index for index, state in enumerate(states)
        if state == 'TRANSITION_FW'
    ]
    fw_indices = [
        index for index, state in enumerate(states) if state == 'HOLD_FW'
    ]

    segment: list[dict[str, str]] = []
    hold_rows = [row for row, state in zip(rows, states) if state == 'HOLD_MC']
    hold_segment: list[dict[str, str]] = []
    transition_time = math.nan
    fw_hold_time = 0.0
    transition_start_index: int | None = None
    if transition_indices:
        transition_start_index = transition_indices[0]
        stop = (
            fw_indices[0]
            if fw_indices and fw_indices[0] >= transition_start_index
            else transition_indices[-1]
        )
        segment = (
            rows[transition_start_index:stop]
            if stop > transition_start_index
            else rows[transition_start_index:transition_start_index + 1]
        )
        transition_time = (
            _number(rows[stop], 'time_s')
            - _number(rows[transition_start_index], 'time_s')
        )
        hold_start = transition_start_index
        while hold_start > 0 and states[hold_start - 1] == 'HOLD_MC':
            hold_start -= 1
        hold_segment = rows[hold_start:transition_start_index]
    if fw_indices:
        fw_hold_time = (
            _number(rows[fw_indices[-1]], 'time_s')
            - _number(rows[fw_indices[0]], 'time_s')
        )

    altitude_errors = _values(segment, 'altitude_error_m')
    altitudes = [
        _first_number(row, ('altitude_relative_m', 'altitude_m'))
        for row in segment
        if math.isfinite(_first_number(row, ('altitude_relative_m', 'altitude_m')))
    ]
    rmse = (
        math.sqrt(fmean(value * value for value in altitude_errors))
        if altitude_errors else math.nan
    )
    max_abs_error = max((abs(value) for value in altitude_errors), default=math.nan)
    min_altitude = min(altitudes, default=math.nan)
    target_altitudes = [
        altitude - error
        for row in segment
        for altitude, error in [
            (
                _first_number(row, ('altitude_relative_m', 'altitude_m')),
                _number(row, 'altitude_error_m'),
            )
        ]
        if math.isfinite(altitude) and math.isfinite(error)
    ]
    target_altitude = _mean(target_altitudes)
    if not math.isfinite(target_altitude):
        target_altitude_candidates = _values(
            rows, 'target_altitude_relative_m'
        )
        target_altitude = (
            _mean(target_altitude_candidates)
            if target_altitude_candidates else 50.0
        )
    start_row = segment[0] if segment else {}
    transition_start_altitude = _first_number(
        start_row, ('altitude_relative_m', 'altitude_m')
    )
    transition_start_altitude_error = (
        transition_start_altitude - target_altitude
        if math.isfinite(transition_start_altitude)
        and math.isfinite(target_altitude)
        else math.nan
    )
    pretransition_altitude_deficit = (
        target_altitude - transition_start_altitude
        if math.isfinite(transition_start_altitude)
        and math.isfinite(target_altitude)
        else math.nan
    )
    target_referenced_drop = (
        max(0.0, target_altitude - min_altitude)
        if math.isfinite(target_altitude) and math.isfinite(min_altitude)
        else math.nan
    )
    additional_transition_drop = (
        max(0.0, transition_start_altitude - min_altitude)
        if math.isfinite(transition_start_altitude)
        and math.isfinite(min_altitude)
        else math.nan
    )
    stability_dwell = (
        _final_stability_dwell(hold_segment, start_row, target_altitude)
        if segment else 0.0
    )
    hold_stability_stats = _stability_statistics(hold_rows, target_altitude)
    if not segment:
        stability_dwell = hold_stability_stats['max_stability_dwell_s']
    pretransition_stability_pass = stability_dwell >= STABILITY_DWELL_S
    hold_times = _values(hold_rows, 'time_s')
    pretransition_hold_duration = (
        _number(start_row, 'time_s') - hold_times[0]
        if segment and hold_times else
        hold_times[-1] - hold_times[0] if len(hold_times) >= 2 else 0.0
    )
    hold_altitudes = [
        _first_number(row, ('altitude_relative_m', 'altitude_m'))
        for row in hold_rows
        if math.isfinite(_first_number(row, ('altitude_relative_m', 'altitude_m')))
    ]
    hold_altitude_errors = _values(hold_rows, 'altitude_error_m')
    hold_trajectory_sp = _values(hold_rows, 'trajectory_sp_altitude_relative_m')

    finite_airspeeds = _airspeed_values(segment)
    max_airspeed = max(finite_airspeeds, default=math.nan)
    any_failsafe = any(row.get('failsafe') == '1' for row in rows)

    final_state = states[-1] if states else ''
    command_error_messages = [
        token.split('=', 1)[1]
        for row in rows
        for token in row.get('command_state', '').split('|')[1:]
        if token.startswith('error=')
    ]
    joined_errors = ' '.join(command_error_messages)
    if state_sequence_ok and vtol_state_sequence_ok and not any_failsafe:
        primary_failure_cause = 'none'
    elif 'timed out waiting for nominal transition stability' in joined_errors:
        primary_failure_cause = 'pretransition_stability_timeout'
    elif (
        final_state == 'ERROR'
        and not transition_indices
        and hold_rows
        and hold_stability_stats['max_stability_dwell_s'] < STABILITY_DWELL_S
    ):
        primary_failure_cause = 'pretransition_stability_timeout'
    elif final_state == 'ERROR' and not transition_indices:
        primary_failure_cause = 'pretransition_abort_before_transition'
    elif any_failsafe:
        primary_failure_cause = 'px4_failsafe'
    elif transition_indices and not fw_indices:
        primary_failure_cause = 'transition_did_not_reach_fw'
    else:
        primary_failure_cause = 'sequence_or_telemetry_incomplete'

    schema_version = max(_values(rows, 'schema_version'), default=1.0)
    completeness_sources = {
        'altitude_m': 'position_age_s',
        'airspeed_mps': 'airspeed_age_s',
        'pitch_rad': 'attitude_age_s',
        'q_rad_s': 'angular_velocity_age_s',
        'lambda_exec': 'lambda_status_age_s',
        'servo_0': 'servos_age_s',
        'servo_1': 'servos_age_s',
        'servo_2': 'servos_age_s',
        'omega_0_rad_s': 'esc_age_s',
        'omega_1_rad_s': 'esc_age_s',
        'omega_2_rad_s': 'esc_age_s',
        'omega_3_rad_s': 'esc_age_s',
        'omega_4_rad_s': 'esc_age_s',
        'lift_power_w': 'esc_age_s',
        'pusher_power_w': 'esc_age_s',
        'pusher_throttle_status': 'pusher_throttle_status_age_s',
        'pusher_throttle_external_active': 'pusher_throttle_active_age_s',
        'motor_control_0': 'actuator_motors_age_s',
        'motor_control_1': 'actuator_motors_age_s',
        'motor_control_2': 'actuator_motors_age_s',
        'motor_control_3': 'actuator_motors_age_s',
        'motor_control_4': 'actuator_motors_age_s',
        'pusher_thrust_setpoint': 'thrust_setpoint_age_s',
        'lift_collective_thrust_setpoint': 'thrust_setpoint_age_s',
    }
    if schema_version >= 12.0:
        completeness_sources.update({
            'mc_pitch_weight_actual': 'mc_pitch_weight_age_s',
            'fw_pitch_weight_actual': 'fw_pitch_weight_age_s',
            'mc_pitch_torque_demand_normalized': (
                'mc_torque_setpoint_age_s'
            ),
            'fw_pitch_torque_demand_normalized': (
                'fw_torque_setpoint_age_s'
            ),
        })
    completeness = {
        key: sum(
            math.isfinite(_number(row, key))
            and math.isfinite(_number(row, age_key))
            and 0.0 <= _number(row, age_key) <= FRESHNESS_LIMIT_S
            for row in rows
        ) / len(rows)
        for key, age_key in completeness_sources.items()
    }
    transition_completeness = {
        key: (
            sum(
                math.isfinite(_number(row, key))
                and math.isfinite(_number(row, age_key))
                and 0.0 <= _number(row, age_key) <= FRESHNESS_LIMIT_S
                for row in segment
            ) / len(segment)
            if segment else 0.0
        )
        for key, age_key in completeness_sources.items()
    }

    schedule_modes = {
        row.get('schedule_mode', '') for row in rows
        if row.get('schedule_mode', '')
    }
    airspeed_sources = [
        row.get('airspeed_source_config', '') for row in rows
        if row.get('airspeed_source_config', '')
    ]
    airspeed_source_config = (
        list(dict.fromkeys(airspeed_sources))
        if airspeed_sources else ['native']
    )
    manual_target_mode = 'manual_target' in schedule_modes
    va_hold_target_mode = 'va_hold_target' in schedule_modes
    fixed_target_mode = manual_target_mode or va_hold_target_mode
    published_rows = [
        row for row in segment
        if math.isfinite(_number(row, 'lambda_command'))
        and (
            _number(row, 'lambda_command') >= 0.0
            if fixed_target_mode
            else _number(row, 'lambda_command') > 0.0
        )
    ]
    commanded_rows = [
        row for row in published_rows
        if _number(row, 'lambda_external_active') > 0.5
        and 0.0 <= _number(row, 'lambda_active_age_s') <= FRESHNESS_LIMIT_S
    ]
    external_active_fraction = (
        len(commanded_rows) / len(published_rows) if published_rows else 0.0
    )
    lambda_command = _values(commanded_rows, 'lambda_command')
    lambda_exec = _values(commanded_rows, 'lambda_exec')
    max_lambda_command = max(lambda_command, default=math.nan)
    max_lambda_exec = max(lambda_exec, default=math.nan)
    tracking_errors = [
        abs(command - executed)
        for row in commanded_rows
        for command, executed in [
            (_number(row, 'lambda_command'), _number(row, 'lambda_exec'))
        ]
        if math.isfinite(command) and math.isfinite(executed)
    ]
    mean_tracking_error = _mean(tracking_errors)
    max_tracking_error = max(tracking_errors, default=math.nan)

    first_publish_time = (
        _number(published_rows[0], 'time_s') if published_rows else math.nan
    )
    first_active_time = (
        _number(commanded_rows[0], 'time_s') if commanded_rows else math.nan
    )
    command_to_active_delay = (
        first_active_time - first_publish_time
        if math.isfinite(first_publish_time) and math.isfinite(first_active_time)
        else math.nan
    )
    target_values = _values(rows, 'lambda_target_config')
    tolerance_values = _values(rows, 'lambda_target_tolerance')
    lambda_target = target_values[0] if target_values else math.nan
    lambda_target_tolerance = (
        tolerance_values[0] if tolerance_values else math.nan
    )
    target_rows = [
        row for row in commanded_rows
        if math.isfinite(lambda_target)
        and math.isfinite(lambda_target_tolerance)
        and math.isfinite(_number(row, 'lambda_exec'))
        and abs(_number(row, 'lambda_exec') - lambda_target)
        <= lambda_target_tolerance
    ]
    lambda_target_settling_time = (
        _number(target_rows[0], 'time_s') - first_publish_time
        if target_rows and math.isfinite(first_publish_time)
        else math.nan
    )
    response_rates = []
    for previous, current in zip(commanded_rows, commanded_rows[1:]):
        previous_time = _number(previous, 'time_s')
        current_time = _number(current, 'time_s')
        previous_exec = _number(previous, 'lambda_exec')
        current_exec = _number(current, 'lambda_exec')
        dt = current_time - previous_time
        if (
            math.isfinite(dt) and dt > 0.0
            and math.isfinite(previous_exec) and math.isfinite(current_exec)
        ):
            response_rates.append((current_exec - previous_exec) / dt)
    positive_response_rates = [rate for rate in response_rates if rate > 0.0]
    steady_state_signed_errors = [
        _number(row, 'lambda_exec') - lambda_target
        for row in target_rows
        if math.isfinite(_number(row, 'lambda_exec'))
    ]
    steady_state_abs_errors = [abs(value) for value in steady_state_signed_errors]
    steady_state_bias = _mean(steady_state_signed_errors)
    steady_state_mean_abs_error = _mean(steady_state_abs_errors)
    steady_state_max_abs_error = max(steady_state_abs_errors, default=math.nan)
    lambda_overshoot = (
        max((value - lambda_target for value in lambda_exec), default=math.nan)
        if math.isfinite(lambda_target) else math.nan
    )
    lambda_characterization_complete = any(
        _number(row, 'lambda_characterization_complete') > 0.5
        for row in rows
    )

    lift_energy, pusher_energy, total_energy = _energy(segment)
    split_index = next(
        (
            index for index, row in enumerate(segment)
            if _number(row, 'airspeed_mps') >= LAMBDA_START_AIRSPEED_MPS
        ),
        None,
    )
    if split_index is None:
        pre_lambda_rows = segment
        lambda_window_rows: list[dict[str, str]] = []
    else:
        # Share the threshold sample so no full logging interval disappears.
        pre_lambda_rows = segment[:split_index + 1]
        lambda_window_rows = segment[split_index:]
    pre_lift, pre_pusher, pre_total = _energy(pre_lambda_rows)
    post_lift, post_pusher, post_total = _energy(lambda_window_rows)
    pre_energy_fraction = (
        pre_total / total_energy
        if math.isfinite(pre_total) and math.isfinite(total_energy)
        and total_energy > 0.0
        else math.nan
    )

    lift_power_by_airspeed_bin = {}
    for lower in range(8, 13):
        values = [
            _number(row, 'lift_power_w') for row in segment
            if lower <= _number(row, 'airspeed_mps') < lower + 1
            and math.isfinite(_number(row, 'lift_power_w'))
        ]
        lift_power_by_airspeed_bin[f'{lower}_{lower + 1}_mps'] = _mean(values)

    transition_start_time = _number(start_row, 'time_s')
    time_to_8_mps = (
        _number(segment[split_index], 'time_s') - transition_start_time
        if split_index is not None and math.isfinite(transition_start_time)
        else math.nan
    )
    pusher_setpoints = _values(segment, 'pusher_thrust_setpoint')
    pusher_motor_controls = _values(segment, 'motor_control_4')
    pusher_omega = _values(segment, 'omega_4_rad_s')
    pusher_setpoint_peak = max(pusher_setpoints, default=math.nan)
    pusher_motor_control_peak = max(pusher_motor_controls, default=math.nan)
    pusher_omega_peak = max(pusher_omega, default=math.nan)
    if (
        transition_completeness.get('pusher_thrust_setpoint', 0.0) < 0.95
        or transition_completeness.get('omega_4_rad_s', 0.0) < 0.95
    ):
        pusher_anomaly_class = 'telemetry_missing_or_stale'
    elif (
        not math.isfinite(pusher_setpoint_peak)
        or pusher_setpoint_peak < PUSHER_MINIMUM_PEAK_SETPOINT
    ):
        pusher_anomaly_class = 'controller_undercommand'
    elif (
        not math.isfinite(pusher_motor_control_peak)
        or pusher_motor_control_peak < pusher_setpoint_peak - 0.05
    ):
        pusher_anomaly_class = 'allocator_or_actuator_command_mismatch'
    elif not math.isfinite(time_to_8_mps) or time_to_8_mps > PUSHER_MAXIMUM_TIME_TO_8_MPS_S:
        pusher_anomaly_class = 'propulsion_response_or_acceleration_slow'
    else:
        pusher_anomaly_class = 'none'
    pusher_diagnostic_pass = bool(
        pusher_anomaly_class == 'none'
    )

    lift_omega = [
        _number(row, f'omega_{index}_rad_s')
        for row in segment for index in range(4)
        if math.isfinite(_number(row, f'omega_{index}_rad_s'))
    ]
    lift_spike_samples = sum(
        any(
            _number(row, f'omega_{index}_rad_s')
            >= LIFT_ROTOR_SPIKE_THRESHOLD_RAD_S
            for index in range(4)
        )
        for row in segment
    )
    lift_spike_rows = [
        row for row in segment
        if any(
            _number(row, f'omega_{index}_rad_s')
            >= LIFT_ROTOR_SPIKE_THRESHOLD_RAD_S
            for index in range(4)
        )
    ]
    lift_spike_rotors = sorted({
        index for row in lift_spike_rows for index in range(4)
        if _number(row, f'omega_{index}_rad_s')
        >= LIFT_ROTOR_SPIKE_THRESHOLD_RAD_S
    })
    first_lift_spike_time = (
        _number(lift_spike_rows[0], 'time_s') - transition_start_time
        if lift_spike_rows and math.isfinite(transition_start_time)
        else math.nan
    )
    if not lift_spike_rows:
        lift_spike_source = 'none'
    elif any(
        _number(row, 'lift_collective_thrust_setpoint') >= 0.98
        and all(
            _number(row, f'motor_control_{index}') >= 0.95
            for index in range(4)
        )
        for row in lift_spike_rows
    ):
        # A simultaneous setpoint, allocator output and measured-speed peak is
        # a real commanded transient, not an ESC-status-only mapping glitch.
        lift_spike_source = 'commanded_collective_saturation'
    else:
        lift_spike_source = 'mixed_or_unresolved'
    early_lift_power = [
        _number(row, 'lift_power_w') for row in segment
        if math.isfinite(transition_start_time)
        and _number(row, 'time_s') - transition_start_time <= 0.5
        and math.isfinite(_number(row, 'lift_power_w'))
    ]

    q_values = _values(segment, 'q_rad_s')
    abs_q = [abs(value) for value in q_values]
    q_rmse = (
        math.sqrt(fmean(value * value for value in q_values))
        if q_values else math.nan
    )
    pitch_values = _values(segment, 'pitch_rad')
    alpha_valid_fraction = (
        sum(_number(row, 'alpha_valid') > 0.5 for row in segment)
        / len(segment)
        if segment else 0.0
    )

    va_measurement_segments = _va_measurement_segments(segment)
    va_measurement_rows = (
        va_measurement_segments[-1] if va_measurement_segments else []
    )
    va_measurement_complete = any(
        _number(row, 'va_hold_measurement_complete') > 0.5 for row in rows
    )
    va_measurement_interrupted = any(
        _number(row, 'va_hold_measurement_interrupted') > 0.5
        for row in rows
    )
    # Schema <= 7 permitted measurement timer resets and later re-entry.  A
    # fixed-condition characterization must instead be one uninterrupted
    # interval, so archived multi-segment runs remain diagnostic only.
    va_measurement_protocol_valid = bool(
        va_measurement_complete
        and not va_measurement_interrupted
        and len(va_measurement_segments) == 1
    )
    va_abort_reasons = [
        row.get('va_hold_abort_reason', '')
        for row in rows
        if row.get('va_hold_abort_reason', '')
    ]
    va_abort_reason = va_abort_reasons[-1] if va_abort_reasons else 'none'
    va_hold_phases = [
        row.get('va_hold_phase', '') for row in segment
        if row.get('va_hold_phase', '')
    ]
    va_pre_measurement_altitude_transient = bool(
        va_hold_target_mode
        and not va_measurement_complete
        and va_abort_reason in {
            'va_hold_altitude_runaway',
            'pre_measurement_altitude_transient',
        }
        and va_hold_phases
        and all(phase == 'airspeed_settle' for phase in va_hold_phases)
        and (
            not math.isfinite(max_airspeed)
            or max_airspeed < 5.0
        )
    )
    va_low_speed_vertical_transient_samples = sum(
        _number(row, 'va_low_speed_vertical_transient') > 0.5
        for row in segment
    )
    va_vertical_abort_enabled_flags = [
        _number(row, 'va_vertical_abort_enabled')
        for row in segment
        if math.isfinite(_number(row, 'va_vertical_abort_enabled'))
    ]
    va_vertical_abort_enabled_fraction = (
        sum(value > 0.5 for value in va_vertical_abort_enabled_flags)
        / len(va_vertical_abort_enabled_flags)
        if va_vertical_abort_enabled_flags else math.nan
    )
    va_measurement_duration = (
        _number(va_measurement_rows[-1], 'time_s')
        - _number(va_measurement_rows[0], 'time_s')
        if len(va_measurement_rows) >= 2 else 0.0
    )
    va_target_values = _values(rows, 'va_target_config')
    va_target = va_target_values[0] if va_target_values else math.nan
    va_tolerance_values = _values(rows, 'va_target_tolerance')
    va_tolerance = (
        va_tolerance_values[0] if va_tolerance_values else math.nan
    )
    va_min_band_fraction_values = _values(
        rows, 'va_measurement_min_band_fraction_config'
    )
    va_min_band_fraction = (
        va_min_band_fraction_values[0]
        if va_min_band_fraction_values else 0.9
    )
    va_max_error_guard_values = _values(
        rows, 'va_measurement_max_abs_airspeed_error_mps_config'
    )
    va_max_error_guard = (
        va_max_error_guard_values[0]
        if va_max_error_guard_values else 0.6
    )
    va_target_dwell_values = _values(rows, 'va_target_dwell_s_config')
    va_target_dwell_config = (
        va_target_dwell_values[0] if va_target_dwell_values else math.nan
    )
    lambda_target_dwell_values = _values(
        rows, 'lambda_target_dwell_s_config'
    )
    lambda_target_dwell_config = (
        lambda_target_dwell_values[0]
        if lambda_target_dwell_values else math.nan
    )
    va_measurement_duration_values = _values(
        rows, 'va_measurement_duration_s_config'
    )
    va_measurement_duration_config = (
        va_measurement_duration_values[0]
        if va_measurement_duration_values else 3.0
    )
    va_measurement_values = _airspeed_values(va_measurement_rows)
    va_measurement_errors = [
        value - va_target for value in va_measurement_values
        if math.isfinite(va_target)
    ]
    va_measurement_abs_errors = [abs(value) for value in va_measurement_errors]
    va_measurement_in_band_fraction = (
        sum(error <= va_tolerance for error in va_measurement_abs_errors)
        / len(va_measurement_abs_errors)
        if va_measurement_abs_errors and math.isfinite(va_tolerance)
        else 0.0
    )
    lambda_measurement_values = _values(va_measurement_rows, 'lambda_exec')
    lambda_measurement_errors = [
        value - lambda_target for value in lambda_measurement_values
        if math.isfinite(lambda_target)
    ]
    lambda_measurement_abs_errors = [
        abs(value) for value in lambda_measurement_errors
    ]
    lambda_measurement_in_band_fraction = (
        sum(error <= lambda_target_tolerance for error in lambda_measurement_abs_errors)
        / len(lambda_measurement_abs_errors)
        if lambda_measurement_abs_errors and math.isfinite(lambda_target_tolerance)
        else 0.0
    )
    va_measurement_max_abs_error = max(
        va_measurement_abs_errors, default=math.nan
    )
    va_measurement_band_occupancy_pass = bool(
        math.isfinite(va_min_band_fraction)
        and va_measurement_in_band_fraction >= va_min_band_fraction
    )
    va_measurement_max_error_guard_pass = bool(
        math.isfinite(va_max_error_guard)
        and math.isfinite(va_measurement_max_abs_error)
        and va_measurement_max_abs_error <= va_max_error_guard
    )
    lambda_measurement_band_occupancy_pass = bool(
        math.isfinite(va_min_band_fraction)
        and lambda_measurement_in_band_fraction >= va_min_band_fraction
    )
    measurement_altitude_errors = _values(
        va_measurement_rows, 'altitude_error_m'
    )
    measurement_vz = _values(va_measurement_rows, 'vz_up_mps')
    measurement_q = _values(va_measurement_rows, 'q_rad_s')
    measurement_pitch = _values(va_measurement_rows, 'pitch_rad')
    measurement_pitch_setpoint = _values(
        va_measurement_rows, 'pitch_setpoint_rad'
    )
    measurement_pitch_tracking_error = [
        _number(row, 'pitch_rad') - _number(row, 'pitch_setpoint_rad')
        for row in va_measurement_rows
        if math.isfinite(_number(row, 'pitch_rad'))
        and math.isfinite(_number(row, 'pitch_setpoint_rad'))
    ]
    measurement_alpha = [
        _number(row, 'alpha_est_rad') for row in va_measurement_rows
        if _number(row, 'alpha_valid') > 0.5
        and math.isfinite(_number(row, 'alpha_est_rad'))
    ]
    measurement_lift_omega = [
        _number(row, f'omega_{index}_rad_s')
        for row in va_measurement_rows for index in range(4)
        if math.isfinite(_number(row, f'omega_{index}_rad_s'))
    ]
    measurement_lift_spike_samples = sum(
        any(
            _number(row, f'omega_{index}_rad_s')
            >= LIFT_ROTOR_SPIKE_THRESHOLD_RAD_S
            for index in range(4)
        )
        for row in va_measurement_rows
    )
    measurement_lift_rotor_saturation_fraction = (
        sum(
            any(
                _number(row, f'omega_{index}_rad_s')
                >= LIFT_ROTOR_SATURATION_THRESHOLD_RAD_S
                for index in range(4)
            )
            for row in va_measurement_rows
        ) / len(va_measurement_rows)
        if va_measurement_rows else math.nan
    )
    measurement_lift_collective_saturation_values = _values(
        va_measurement_rows, 'lift_collective_thrust_setpoint'
    )
    measurement_lift_collective_saturation_fraction = (
        sum(value >= 0.98 for value in measurement_lift_collective_saturation_values)
        / len(measurement_lift_collective_saturation_values)
        if measurement_lift_collective_saturation_values else math.nan
    )
    measurement_total_power = [
        lift + pusher
        for row in va_measurement_rows
        for lift, pusher in [
            (_number(row, 'lift_power_w'), _number(row, 'pusher_power_w'))
        ]
        if math.isfinite(lift) and math.isfinite(pusher)
    ]
    measurement_servo_values = [
        _number(row, f'servo_{index}')
        for row in va_measurement_rows for index in range(3)
        if math.isfinite(_number(row, f'servo_{index}'))
    ]
    measurement_servo_saturation_fraction = (
        sum(abs(value) >= 0.95 for value in measurement_servo_values)
        / len(measurement_servo_values)
        if measurement_servo_values else math.nan
    )
    measurement_elevator_joint_limit_values = _values(
        va_measurement_rows, 'elevator_joint_limit_commanded'
    )
    measurement_elevator_joint_limit_command_fraction = (
        sum(value > 0.5 for value in measurement_elevator_joint_limit_values)
        / len(measurement_elevator_joint_limit_values)
        if measurement_elevator_joint_limit_values else math.nan
    )
    measurement_mc_pitch_weight = _values(
        va_measurement_rows, 'mc_pitch_weight_actual'
    )
    measurement_fw_pitch_weight = _values(
        va_measurement_rows, 'fw_pitch_weight_actual'
    )
    measurement_mc_pitch_weight_proxy = _values(
        va_measurement_rows, 'mc_pitch_weight_proxy'
    )
    measurement_mc_pitch_weight_errors = [
        actual - proxy
        for row in va_measurement_rows
        for actual, proxy in [(
            _number(row, 'mc_pitch_weight_actual'),
            _number(row, 'mc_pitch_weight_proxy'),
        )]
        if math.isfinite(actual) and math.isfinite(proxy)
    ]
    measurement_mc_lambda_expected = _values(
        va_measurement_rows, 'mc_pitch_weight_lambda_expected'
    )
    measurement_fw_lambda_expected = _values(
        va_measurement_rows, 'fw_pitch_weight_lambda_expected'
    )
    measurement_mc_lambda_sync_errors = [
        actual - expected
        for row in va_measurement_rows
        for actual, expected in [(
            _number(row, 'mc_pitch_weight_actual'),
            _number(row, 'mc_pitch_weight_lambda_expected'),
        )]
        if math.isfinite(actual) and math.isfinite(expected)
    ]
    measurement_fw_lambda_sync_errors = [
        actual - expected
        for row in va_measurement_rows
        for actual, expected in [(
            _number(row, 'fw_pitch_weight_actual'),
            _number(row, 'fw_pitch_weight_lambda_expected'),
        )]
        if math.isfinite(actual) and math.isfinite(expected)
    ]
    lambda_attitude_sync_pass = bool(
        schema_version < 13.0
        or (
            len(measurement_mc_lambda_sync_errors) >= 30
            and len(measurement_fw_lambda_sync_errors) >= 30
            and _rmse(measurement_mc_lambda_sync_errors) <= 0.02
            and _rmse(measurement_fw_lambda_sync_errors) <= 0.02
        )
    )
    transition_lambda_active_rows = [
        row for row in segment
        if _number(row, 'lambda_external_active') > 0.5
    ]
    transition_mc_lambda_sync_errors = [
        actual - expected
        for row in transition_lambda_active_rows
        for actual, expected in [(
            _number(row, 'mc_pitch_weight_actual'),
            _number(row, 'mc_pitch_weight_lambda_expected'),
        )]
        if math.isfinite(actual) and math.isfinite(expected)
    ]
    transition_fw_lambda_sync_errors = [
        actual - expected
        for row in transition_lambda_active_rows
        for actual, expected in [(
            _number(row, 'fw_pitch_weight_actual'),
            _number(row, 'fw_pitch_weight_lambda_expected'),
        )]
        if math.isfinite(actual) and math.isfinite(expected)
    ]
    # Architecture verification must be evaluated after the requested test
    # point has actually been reached.  Looking at every externally-active
    # row can otherwise produce a false pass when a run aborts while lambda is
    # still zero: the stock (MC=1, FW=0) endpoint is internally consistent,
    # but it provides no evidence for the requested non-zero allocation.
    target_mc_lambda_sync_errors = [
        actual - expected
        for row in target_rows
        for actual, expected in [(
            _number(row, 'mc_pitch_weight_actual'),
            _number(row, 'mc_pitch_weight_lambda_expected'),
        )]
        if math.isfinite(actual) and math.isfinite(expected)
    ]
    target_fw_lambda_sync_errors = [
        actual - expected
        for row in target_rows
        for actual, expected in [(
            _number(row, 'fw_pitch_weight_actual'),
            _number(row, 'fw_pitch_weight_lambda_expected'),
        )]
        if math.isfinite(actual) and math.isfinite(expected)
    ]
    transition_lambda_attitude_sync_pass = bool(
        schema_version < 13.0
        or (
            len(transition_mc_lambda_sync_errors) >= 10
            and len(transition_fw_lambda_sync_errors) >= 10
            and _rmse(transition_mc_lambda_sync_errors) <= 0.02
            and _rmse(transition_fw_lambda_sync_errors) <= 0.02
        )
    )
    target_lambda_attitude_sync_pass = bool(
        schema_version < 13.0
        or (
            len(target_mc_lambda_sync_errors) >= 10
            and len(target_fw_lambda_sync_errors) >= 10
            and _rmse(target_mc_lambda_sync_errors) <= 0.02
            and _rmse(target_fw_lambda_sync_errors) <= 0.02
        )
    )
    measurement_mc_pitch_demand = _values(
        va_measurement_rows, 'mc_pitch_torque_demand_normalized'
    )
    measurement_fw_pitch_demand = _values(
        va_measurement_rows, 'fw_pitch_torque_demand_normalized'
    )
    measurement_mc_pitch_contribution = _values(
        va_measurement_rows, 'mc_pitch_torque_contribution_normalized'
    )
    measurement_fw_pitch_contribution = _values(
        va_measurement_rows, 'fw_pitch_torque_contribution_normalized'
    )
    measurement_pusher_active_flags = [
        _number(row, 'pusher_throttle_external_active')
        for row in va_measurement_rows
        if math.isfinite(_number(row, 'pusher_throttle_external_active'))
    ]
    measurement_pusher_active_fraction = (
        sum(value > 0.5 for value in measurement_pusher_active_flags)
        / len(measurement_pusher_active_flags)
        if measurement_pusher_active_flags else 0.0
    )
    measurement_pusher_throttle_command_values = _values(
        va_measurement_rows, 'pusher_throttle_command'
    )
    measurement_down_velocity_commands = _values(
        va_measurement_rows, 'va_hold_down_velocity_command_mps'
    )
    transition_down_velocity_commands = _values(
        segment, 'va_hold_down_velocity_command_mps'
    )
    measurement_pusher_throttle_upper_saturation_fraction = (
        sum(value >= PUSHER_THROTTLE_UPPER_SATURATION
            for value in measurement_pusher_throttle_command_values)
        / len(measurement_pusher_throttle_command_values)
        if measurement_pusher_throttle_command_values else math.nan
    )
    measurement_pusher_omega_values = _values(
        va_measurement_rows, 'omega_4_rad_s'
    )
    measurement_pusher_rotor_saturation_fraction = (
        sum(value >= PUSHER_ROTOR_SATURATION_THRESHOLD_RAD_S
            for value in measurement_pusher_omega_values)
        / len(measurement_pusher_omega_values)
        if measurement_pusher_omega_values else math.nan
    )
    measurement_lift_energy, measurement_pusher_energy, measurement_total_energy = (
        _energy(va_measurement_rows)
    )
    transition_pusher_active_flags = [
        _number(row, 'pusher_throttle_external_active')
        for row in segment
        if math.isfinite(_number(row, 'pusher_throttle_external_active'))
    ]
    transition_pusher_active_fraction = (
        sum(value > 0.5 for value in transition_pusher_active_flags)
        / len(transition_pusher_active_flags)
        if transition_pusher_active_flags else 0.0
    )
    transition_pusher_command_values = _values(segment, 'pusher_throttle_command')
    transition_pusher_status_values = _values(segment, 'pusher_throttle_status')

    pusher_regulation_pass = True
    pusher_regulation_failure = 'none'
    if va_hold_target_mode:
        if not va_measurement_complete:
            pusher_regulation_pass = False
            if (
                math.isfinite(va_target)
                and math.isfinite(max_airspeed)
                and max_airspeed > va_target + 3.0
                and transition_pusher_active_fraction < 0.5
                and _mean(pusher_setpoints) > 0.4
            ):
                pusher_regulation_failure = 'pusher_not_regulating_airspeed'
            elif (
                math.isfinite(va_target)
                and math.isfinite(max_airspeed)
                and max_airspeed > va_target + 3.0
                and transition_pusher_active_fraction >= 0.5
                and _mean(transition_pusher_status_values) > 0.4
            ):
                pusher_regulation_failure = 'pusher_airspeed_control_ineffective'
            else:
                pusher_regulation_failure = 'va_hold_not_measured'
        elif measurement_pusher_active_fraction < 0.9:
            pusher_regulation_pass = False
            pusher_regulation_failure = 'pusher_external_control_not_active'

    secondary_failure_cause = 'none'
    if (
        primary_failure_cause == 'transition_did_not_reach_fw'
        and pusher_anomaly_class == 'controller_undercommand'
    ):
        primary_failure_cause = 'transition_pusher_undercommand'
    if (
        primary_failure_cause != 'none'
        and manual_target_mode
        and len(published_rows) == 0
        and external_active_fraction == 0.0
    ):
        secondary_failure_cause = 'lambda_never_activated'
    if va_hold_target_mode and not va_measurement_protocol_valid:
        primary_failure_cause = 'va_hold_settle_timeout'
        if va_measurement_interrupted:
            primary_failure_cause = 'measurement_window_interrupted'
            secondary_failure_cause = 'fixed_condition_not_maintained'
        elif len(va_measurement_segments) > 1:
            primary_failure_cause = 'measurement_window_fragmented'
            secondary_failure_cause = 'fixed_condition_not_maintained'
        elif va_abort_reason != 'none':
            primary_failure_cause = va_abort_reason
        if va_pre_measurement_altitude_transient:
            primary_failure_cause = 'pre_measurement_altitude_transient'
            secondary_failure_cause = 'pre_measurement_not_f1_boundary'
        elif secondary_failure_cause == 'none':
            secondary_failure_cause = pusher_regulation_failure

    baseline_pass = bool(
        state_sequence_ok
        and vtol_state_sequence_ok
        and not any_failsafe
        and math.isfinite(transition_time)
        and transition_time <= 25.0
        and fw_hold_time >= 5.0
        and math.isfinite(min_altitude)
        and min_altitude >= 45.0
    )
    constant_altitude_pass = bool(
        baseline_pass
        and pretransition_stability_pass
        and math.isfinite(target_altitude)
        and abs(target_altitude - 50.0) <= 0.1
        and math.isfinite(transition_start_altitude_error)
        and abs(transition_start_altitude_error)
        <= STABILITY_ALTITUDE_TOLERANCE_M
        and math.isfinite(rmse) and rmse <= 3.0
        and math.isfinite(max_abs_error) and max_abs_error <= 5.0
    )
    f1_fixed_window_altitude_pass = bool(
        pretransition_stability_pass
        and math.isfinite(target_altitude)
        and abs(target_altitude - 50.0) <= 0.1
        and math.isfinite(transition_start_altitude_error)
        and abs(transition_start_altitude_error)
        <= STABILITY_ALTITUDE_TOLERANCE_M
        and math.isfinite(_rmse(measurement_altitude_errors))
        and _rmse(measurement_altitude_errors) <= 3.0
        and math.isfinite(max(
            (abs(value) for value in measurement_altitude_errors),
            default=math.nan,
        ))
        and max(
            (abs(value) for value in measurement_altitude_errors),
            default=math.nan,
        ) <= 5.0
    )
    # Startup rows necessarily precede several PX4 publishers, so whole-file
    # completeness is diagnostic only.  The scientific requirement applies
    # to the transition window.  The pusher actuator-control slot is NaN until
    # its allocator becomes active; its direct thrust setpoint and measured
    # omega remain the authoritative continuous diagnostics.  We therefore
    # require 80% for that auxiliary slot and 95% for every other field.
    transition_required = {
        key: value for key, value in transition_completeness.items()
        if key not in {
            'motor_control_4',
            'pusher_throttle_status',
            'pusher_throttle_external_active',
        }
    }
    telemetry_pass = bool(
        schema_version >= 2.0
        and all(value >= 0.95 for value in transition_required.values())
        and transition_completeness.get('motor_control_4', 0.0) >= 0.8
    )
    manual_airspeed_lambda_pass = bool(
        not manual_target_mode
        and constant_altitude_pass
        and len(tracking_errors) >= 5
        and external_active_fraction >= 0.8
        and math.isfinite(max_lambda_command) and max_lambda_command >= 0.05
        and math.isfinite(max_lambda_exec) and max_lambda_exec >= 0.02
        and max_lambda_exec <= max_lambda_command + 0.05
        and math.isfinite(mean_tracking_error) and mean_tracking_error <= 0.1
        and math.isfinite(max_tracking_error) and max_tracking_error <= 0.2
    )
    manual_target_lambda_pass = bool(
        manual_target_mode
        and constant_altitude_pass
        and len(tracking_errors) >= 5
        and external_active_fraction >= 0.8
        and bool(target_rows)
        and lambda_characterization_complete
        and math.isfinite(command_to_active_delay)
        and command_to_active_delay <= FRESHNESS_LIMIT_S
        and math.isfinite(lambda_target_settling_time)
        and math.isfinite(steady_state_mean_abs_error)
        and steady_state_mean_abs_error <= lambda_target_tolerance
        and math.isfinite(steady_state_max_abs_error)
        and steady_state_max_abs_error <= lambda_target_tolerance
        and (
            not math.isfinite(lambda_overshoot)
            or lambda_overshoot <= lambda_target_tolerance
        )
    )
    manual_lambda_pass = bool(
        manual_airspeed_lambda_pass or manual_target_lambda_pass
    )
    characterization_pass = manual_target_lambda_pass
    # Phase-0.75 retains the historical acceleration diagnostic because that
    # stage validated the complete transition engineering chain at Va=12 m/s.
    # F1, however, deliberately includes targets below 8 m/s.  Its scientific
    # pass criterion must describe the fixed-(Va, lambda) measurement window,
    # not require the vehicle to satisfy the Phase-0.5 "reach 8 m/s quickly"
    # diagnostic.  Keep the two gates separate so low-speed F1 cells are not
    # rejected solely for respecting their commanded airspeed.
    fixed_window_quality_failures = []
    if not f1_fixed_window_altitude_pass:
        fixed_window_quality_failures.append('fixed_window_altitude')
    if not va_measurement_band_occupancy_pass:
        fixed_window_quality_failures.append('airspeed_band_occupancy')
    if not va_measurement_max_error_guard_pass:
        fixed_window_quality_failures.append('airspeed_max_error_guard')
    if not lambda_measurement_band_occupancy_pass:
        fixed_window_quality_failures.append('lambda_band_occupancy')
    if (
        not math.isfinite(_mean(lambda_measurement_abs_errors))
        or _mean(lambda_measurement_abs_errors) > lambda_target_tolerance
    ):
        fixed_window_quality_failures.append('lambda_mean_tracking')
    if not lambda_attitude_sync_pass:
        fixed_window_quality_failures.append('lambda_attitude_sync')
    f1_measurement_pass = bool(
        va_hold_target_mode
        and f1_fixed_window_altitude_pass
        and telemetry_pass
        and pusher_regulation_pass
        and va_measurement_protocol_valid
        and va_measurement_duration >= 0.9 * va_measurement_duration_config
        and len(va_measurement_rows) >= 30
        and va_measurement_band_occupancy_pass
        and va_measurement_max_error_guard_pass
        and lambda_measurement_band_occupancy_pass
        and _mean(lambda_measurement_abs_errors) <= lambda_target_tolerance
        and measurement_pusher_active_fraction >= 0.9
        and lambda_attitude_sync_pass
    )
    hard_physical_failure_causes = {
        'va_hold_altitude_runaway',
        'va_hold_vertical_speed_runaway',
        'airspeed_hold_runaway',
        'px4_failsafe',
        'altitude_runaway',
        'aoa_limit',
        'descent_rate_limit',
        'pitch_rate_limit',
    }
    lambda_progress_threshold = math.nan
    if math.isfinite(lambda_target):
        if lambda_target <= max(lambda_target_tolerance, 1e-6):
            lambda_progress_threshold = 0.0
        else:
            lambda_progress_threshold = min(
                max(0.05, 0.5 * lambda_target),
                max(0.05, lambda_target - 2.0 * lambda_target_tolerance),
            )
    target_directed_ramp_established = bool(
        va_hold_target_mode
        and primary_failure_cause in hard_physical_failure_causes
        and not va_measurement_complete
        and not va_pre_measurement_altitude_transient
        and math.isfinite(va_target)
        and math.isfinite(va_tolerance)
        and math.isfinite(max_airspeed)
        and max_airspeed >= va_target - va_tolerance
        and math.isfinite(lambda_progress_threshold)
        and math.isfinite(max_lambda_exec)
        and (
            (lambda_progress_threshold == 0.0
             and max_lambda_exec <= max(lambda_target_tolerance, 0.03))
            or max_lambda_exec >= lambda_progress_threshold
        )
        and external_active_fraction > 0.0
        and transition_lambda_attitude_sync_pass
    )
    target_condition_established = bool(
        va_measurement_complete
        or target_directed_ramp_established
        or (
            bool(target_rows)
            and math.isfinite(va_target)
            and math.isfinite(va_tolerance)
            and math.isfinite(max_airspeed)
            and max_airspeed >= va_target - va_tolerance
        )
    )
    protocol_failure_causes = {
        'sequence_or_telemetry_incomplete',
        'pretransition_stability_timeout',
        'pretransition_abort_before_transition',
        'transition_did_not_reach_fw',
        'transition_pusher_undercommand',
    }
    f1_attempt_quality_reason = 'none'
    if va_pre_measurement_altitude_transient:
        f1_attempt_quality_reason = 'pre_measurement_altitude_transient'
    elif primary_failure_cause in protocol_failure_causes and not f1_measurement_pass:
        f1_attempt_quality_reason = primary_failure_cause
    elif not telemetry_pass:
        f1_attempt_quality_reason = 'telemetry_incomplete'
    elif (
        primary_failure_cause not in {'', 'none'}
        and not va_measurement_complete
        and not target_condition_established
    ):
        f1_attempt_quality_reason = 'target_condition_not_established'
    f1_attempt_quality = (
        'valid' if f1_attempt_quality_reason == 'none'
        else 'protocol_invalid'
    )
    va_hold_pass = bool(
        f1_measurement_pass
        and pusher_diagnostic_pass
    )

    return {
        'source_csv': str(path.resolve()),
        'telemetry_schema_version': schema_version,
        'schedule_modes_seen': sorted(schedule_modes),
        'airspeed_source_config': airspeed_source_config,
        'condition_id': list(dict.fromkeys(
            row.get('condition_id', '') for row in rows
            if row.get('condition_id', '')
        )),
        'wind_model_config': list(dict.fromkeys(
            row.get('wind_model_config', '') for row in rows
            if row.get('wind_model_config', '')
        )),
        'mass_scale_config': _mean(_values(rows, 'mass_scale_config')),
        'nominal_model_mass_kg_config': _mean(
            _values(rows, 'nominal_model_mass_kg_config')
        ),
        'actual_model_mass_kg_config': _mean(
            _values(rows, 'actual_model_mass_kg_config')
        ),
        'payload_mass_kg_config': _mean(_values(rows, 'payload_mass_kg_config')),
        'row_count': len(rows),
        'states_seen': list(dict.fromkeys(states)),
        'state_sequence_ok': state_sequence_ok,
        'vtol_states_seen': list(dict.fromkeys(vtol_states)),
        'vtol_state_sequence_ok': vtol_state_sequence_ok,
        'any_failsafe': any_failsafe,
        'primary_failure_cause': primary_failure_cause,
        'secondary_failure_cause': secondary_failure_cause,
        'f1_attempt_quality': f1_attempt_quality,
        'f1_attempt_quality_reason': f1_attempt_quality_reason,
        'f1_target_condition_established': target_condition_established,
        'f1_target_directed_ramp_established': (
            target_directed_ramp_established
        ),
        'transition_time_s': transition_time,
        'fw_hold_time_s': fw_hold_time,
        'altitude_datum_local_m': _mean(_values(rows, 'altitude_datum_local_m')),
        'target_altitude_local_m': _mean(_values(rows, 'target_altitude_local_m')),
        'target_altitude_relative_m': target_altitude,
        'transition_start_altitude_m': transition_start_altitude,
        'transition_start_altitude_error_m': transition_start_altitude_error,
        'pretransition_altitude_deficit_m': pretransition_altitude_deficit,
        'pretransition_stability_dwell_s': stability_dwell,
        'pretransition_stability_pass': pretransition_stability_pass,
        'pretransition_hold_duration_s': pretransition_hold_duration,
        'pretransition_hold_mean_altitude_m': _mean(hold_altitudes),
        'pretransition_hold_mean_altitude_error_m': _mean(hold_altitude_errors),
        'pretransition_hold_mean_trajectory_sp_altitude_m': _mean(
            hold_trajectory_sp
        ),
        'pretransition_height_gate_fraction': hold_stability_stats[
            'height_gate_fraction'
        ],
        'pretransition_vz_gate_fraction': hold_stability_stats[
            'vz_gate_fraction'
        ],
        'pretransition_groundspeed_gate_fraction': hold_stability_stats[
            'groundspeed_gate_fraction'
        ],
        'pretransition_stability_gate_fraction': hold_stability_stats[
            'stability_gate_fraction'
        ],
        'pretransition_max_stability_dwell_s': hold_stability_stats[
            'max_stability_dwell_s'
        ],
        'altitude_rmse_m': rmse,
        'max_abs_altitude_error_m': max_abs_error,
        'minimum_transition_altitude_m': min_altitude,
        'target_altitude_m': target_altitude,
        # Kept for compatibility; this is target-referenced, not incremental.
        'maximum_transition_drop_m': target_referenced_drop,
        'maximum_target_referenced_drop_m': target_referenced_drop,
        'additional_transition_drop_m': additional_transition_drop,
        'maximum_transition_airspeed_mps': max_airspeed,
        'time_to_8_mps_s': time_to_8_mps,
        'pusher_thrust_setpoint_mean': _mean(pusher_setpoints),
        'pusher_thrust_setpoint_peak': pusher_setpoint_peak,
        'pusher_motor_control_mean': _mean(pusher_motor_controls),
        'pusher_motor_control_peak': pusher_motor_control_peak,
        'pusher_motor_omega_peak_rad_s': pusher_omega_peak,
        'pusher_anomaly_class': pusher_anomaly_class,
        'pusher_acceleration_diagnostic_pass': pusher_diagnostic_pass,
        'max_lift_rotor_omega_rad_s': max(lift_omega, default=math.nan),
        'lift_rotor_spike_threshold_rad_s': LIFT_ROTOR_SPIKE_THRESHOLD_RAD_S,
        'lift_rotor_spike_samples': lift_spike_samples,
        'lift_rotor_spike_rotor_indices': lift_spike_rotors,
        'first_lift_rotor_spike_after_transition_s': first_lift_spike_time,
        'lift_rotor_spike_source_class': lift_spike_source,
        'max_lift_power_proxy_w': max(_values(segment, 'lift_power_w'), default=math.nan),
        'initial_0p5s_mean_lift_power_proxy_w': _mean(early_lift_power),
        'pitch_rate_rmse_rad_s': q_rmse,
        'pitch_rate_p95_rad_s': _percentile(abs_q, 0.95),
        'max_abs_pitch_rate_rad_s': max(abs_q, default=math.nan),
        'max_abs_pitch_rad': max((abs(value) for value in pitch_values), default=math.nan),
        'alpha_valid_fraction_in_transition': alpha_valid_fraction,
        'va_target_config': va_target,
        'va_target_tolerance': va_tolerance,
        'vt_unload_altitude_pitch_kp_config': _mean(_values(
            rows, 'vt_unload_altitude_pitch_kp_config'
        )),
        'vt_unload_vertical_speed_pitch_kd_config': _mean(_values(
            rows, 'vt_unload_vertical_speed_pitch_kd_config'
        )),
        'vt_unload_pitch_min_deg_config': _mean(_values(
            rows, 'vt_unload_pitch_min_deg_config'
        )),
        'vt_unload_pitch_max_deg_config': _mean(_values(
            rows, 'vt_unload_pitch_max_deg_config'
        )),
        'vt_airspeed_blend_mps_config': _mean(_values(
            rows, 'vt_airspeed_blend_mps_config'
        )),
        'vt_transition_airspeed_mps_config': _mean(_values(
            rows, 'vt_transition_airspeed_mps_config'
        )),
        'lambda_attitude_blend_start_config': _mean(_values(
            rows, 'lambda_attitude_blend_start_config'
        )),
        'lambda_attitude_blend_full_config': _mean(_values(
            rows, 'lambda_attitude_blend_full_config'
        )),
        'va_hold_abort_reason': va_abort_reason,
        'va_hold_pre_measurement_altitude_transient': (
            va_pre_measurement_altitude_transient
        ),
        'va_hold_measurement_segment_count': len(va_measurement_segments),
        'va_hold_selected_measurement_segment_index': (
            len(va_measurement_segments) - 1
            if va_measurement_segments else None
        ),
        'va_low_speed_vertical_transient_samples': (
            va_low_speed_vertical_transient_samples
        ),
        'va_vertical_abort_enabled_fraction': (
            va_vertical_abort_enabled_fraction
        ),
        'phase_0_75_pusher_airspeed_regulation_pass': pusher_regulation_pass,
        'phase_0_75_pusher_airspeed_regulation_failure': pusher_regulation_failure,
        'va_hold_transition_pusher_external_active_fraction': (
            transition_pusher_active_fraction
        ),
        'va_hold_transition_mean_pusher_throttle_command': _mean(
            transition_pusher_command_values
        ),
        'va_hold_transition_mean_pusher_throttle_status': _mean(
            transition_pusher_status_values
        ),
        'va_hold_measurement_complete': va_measurement_complete,
        'va_hold_measurement_interrupted': va_measurement_interrupted,
        'va_hold_measurement_protocol_valid': va_measurement_protocol_valid,
        'va_hold_measurement_samples': len(va_measurement_rows),
        'va_hold_measurement_duration_s': va_measurement_duration,
        'va_hold_measurement_min_band_fraction_config': (
            va_min_band_fraction
        ),
        'va_hold_measurement_max_abs_airspeed_error_mps_config': (
            va_max_error_guard
        ),
        'va_target_dwell_s_config': va_target_dwell_config,
        'lambda_target_dwell_s_config': lambda_target_dwell_config,
        'va_measurement_duration_s_config': (
            va_measurement_duration_config
        ),
        'va_hold_mean_airspeed_mps': _mean(va_measurement_values),
        'va_hold_std_airspeed_mps': _std(va_measurement_values),
        'va_hold_mean_airspeed_error_mps': _mean(va_measurement_errors),
        'va_hold_max_abs_airspeed_error_mps': (
            va_measurement_max_abs_error
        ),
        'va_hold_airspeed_in_band_fraction': va_measurement_in_band_fraction,
        'va_hold_airspeed_band_occupancy_pass': (
            va_measurement_band_occupancy_pass
        ),
        'va_hold_airspeed_max_error_guard_pass': (
            va_measurement_max_error_guard_pass
        ),
        'va_hold_mean_lambda_exec': _mean(lambda_measurement_values),
        'va_hold_mean_lambda_error': _mean(lambda_measurement_errors),
        'va_hold_max_abs_lambda_error': max(
            lambda_measurement_abs_errors, default=math.nan
        ),
        'va_hold_lambda_in_band_fraction': lambda_measurement_in_band_fraction,
        'va_hold_lambda_band_occupancy_pass': (
            lambda_measurement_band_occupancy_pass
        ),
        'va_hold_fixed_window_altitude_pass': (
            f1_fixed_window_altitude_pass
        ),
        'va_hold_fixed_window_quality_pass': not fixed_window_quality_failures,
        'va_hold_fixed_window_quality_failures': (
            fixed_window_quality_failures
        ),
        'va_hold_measurement_pusher_external_active_fraction': (
            measurement_pusher_active_fraction
        ),
        'va_hold_mean_pusher_throttle_command': _mean(
            _values(va_measurement_rows, 'pusher_throttle_command')
        ),
        'va_hold_mean_down_velocity_command_mps': _mean(
            measurement_down_velocity_commands
        ),
        'va_hold_max_abs_down_velocity_command_mps': max(
            (abs(value) for value in measurement_down_velocity_commands),
            default=math.nan,
        ),
        'va_hold_transition_max_abs_down_velocity_command_mps': max(
            (abs(value) for value in transition_down_velocity_commands),
            default=math.nan,
        ),
        'va_hold_mean_pusher_throttle_status': _mean(
            _values(va_measurement_rows, 'pusher_throttle_status')
        ),
        'va_hold_mean_airspeed_filtered_mps': _mean(
            _values(va_measurement_rows, 'airspeed_filtered_mps')
        ),
        'va_hold_mean_pusher_airspeed_error_filtered_mps': _mean(
            _values(
                va_measurement_rows,
                'pusher_airspeed_error_filtered_mps',
            )
        ),
        'va_hold_max_abs_pusher_airspeed_error_filtered_mps': max(
            (
                abs(value) for value in _values(
                    va_measurement_rows,
                    'pusher_airspeed_error_filtered_mps',
                )
            ),
            default=math.nan,
        ),
        'va_hold_mean_pusher_pi_integral_mps_s': _mean(
            _values(va_measurement_rows, 'pusher_pi_integral_mps_s')
        ),
        'va_hold_max_abs_pusher_pi_integral_mps_s': max(
            (
                abs(value) for value in _values(
                    va_measurement_rows, 'pusher_pi_integral_mps_s'
                )
            ),
            default=math.nan,
        ),
        'va_hold_mean_pusher_throttle_unsaturated': _mean(
            _values(va_measurement_rows, 'pusher_throttle_unsaturated')
        ),
        'va_hold_mean_lift_thrust_n': _mean(
            _values(va_measurement_rows, 'lift_thrust_n')
        ),
        'va_hold_mean_lift_collective_thrust_setpoint': _mean(
            _values(va_measurement_rows, 'lift_collective_thrust_setpoint')
        ),
        'va_hold_mean_lift_rotor_omega_rad_s': _mean(measurement_lift_omega),
        'va_hold_max_lift_rotor_omega_rad_s': max(
            measurement_lift_omega, default=math.nan
        ),
        'va_hold_lift_rotor_spike_samples': measurement_lift_spike_samples,
        'va_hold_lift_rotor_saturation_fraction': (
            measurement_lift_rotor_saturation_fraction
        ),
        'va_hold_lift_collective_saturation_fraction': (
            measurement_lift_collective_saturation_fraction
        ),
        'va_hold_mean_lift_power_w': _mean(
            _values(va_measurement_rows, 'lift_power_w')
        ),
        'va_hold_mean_pusher_power_w': _mean(
            _values(va_measurement_rows, 'pusher_power_w')
        ),
        'va_hold_mean_total_power_w': _mean(measurement_total_power),
        'va_hold_lift_energy_proxy_j': measurement_lift_energy,
        'va_hold_pusher_energy_proxy_j': measurement_pusher_energy,
        'va_hold_total_energy_proxy_j': measurement_total_energy,
        'va_hold_altitude_rmse_m': _rmse(measurement_altitude_errors),
        'va_hold_max_abs_altitude_error_m': max(
            (abs(value) for value in measurement_altitude_errors),
            default=math.nan,
        ),
        'va_hold_vertical_speed_rms_mps': _rmse(measurement_vz),
        'va_hold_max_abs_vertical_speed_mps': max(
            (abs(value) for value in measurement_vz), default=math.nan
        ),
        'va_hold_mean_alpha_rad': _mean(measurement_alpha),
        'va_hold_max_abs_alpha_rad': max(
            (abs(value) for value in measurement_alpha), default=math.nan
        ),
        'va_hold_mean_pitch_rad': _mean(measurement_pitch),
        'va_hold_mean_pitch_setpoint_rad': _mean(
            measurement_pitch_setpoint
        ),
        'va_hold_mean_mc_pitch_weight_proxy': _mean(_values(
            va_measurement_rows, 'mc_pitch_weight_proxy'
        )),
        'va_hold_min_mc_pitch_weight_proxy': min(
            _values(va_measurement_rows, 'mc_pitch_weight_proxy'),
            default=math.nan,
        ),
        'va_hold_max_mc_pitch_weight_proxy': max(
            _values(va_measurement_rows, 'mc_pitch_weight_proxy'),
            default=math.nan,
        ),
        'va_hold_mean_mc_pitch_weight_actual': _mean(
            measurement_mc_pitch_weight
        ),
        'va_hold_min_mc_pitch_weight_actual': min(
            measurement_mc_pitch_weight, default=math.nan
        ),
        'va_hold_max_mc_pitch_weight_actual': max(
            measurement_mc_pitch_weight, default=math.nan
        ),
        'va_hold_mean_fw_pitch_weight_actual': _mean(
            measurement_fw_pitch_weight
        ),
        'va_hold_mean_mc_pitch_weight_lambda_expected': _mean(
            measurement_mc_lambda_expected
        ),
        'va_hold_mean_fw_pitch_weight_lambda_expected': _mean(
            measurement_fw_lambda_expected
        ),
        'va_hold_mc_pitch_weight_lambda_sync_rmse': _rmse(
            measurement_mc_lambda_sync_errors
        ),
        'va_hold_fw_pitch_weight_lambda_sync_rmse': _rmse(
            measurement_fw_lambda_sync_errors
        ),
        'lambda_attitude_sync_pass': lambda_attitude_sync_pass,
        'transition_lambda_attitude_sync_samples': min(
            len(transition_mc_lambda_sync_errors),
            len(transition_fw_lambda_sync_errors),
        ),
        'transition_mc_pitch_weight_lambda_sync_rmse': _rmse(
            transition_mc_lambda_sync_errors
        ),
        'transition_fw_pitch_weight_lambda_sync_rmse': _rmse(
            transition_fw_lambda_sync_errors
        ),
        'transition_lambda_attitude_sync_pass': (
            transition_lambda_attitude_sync_pass
        ),
        'target_lambda_attitude_sync_samples': min(
            len(target_mc_lambda_sync_errors),
            len(target_fw_lambda_sync_errors),
        ),
        'target_mc_pitch_weight_lambda_sync_rmse': _rmse(
            target_mc_lambda_sync_errors
        ),
        'target_fw_pitch_weight_lambda_sync_rmse': _rmse(
            target_fw_lambda_sync_errors
        ),
        'target_lambda_attitude_sync_pass': (
            target_lambda_attitude_sync_pass
        ),
        'va_hold_mc_pitch_weight_proxy_rmse': _rmse(
            measurement_mc_pitch_weight_errors
        ),
        # PX4 VehicleTorqueSetpoint.xyz is normalized.  These fields are
        # controller-demand diagnostics and must not be reported as N m.
        'va_hold_mean_mc_pitch_torque_demand_normalized': _mean(
            measurement_mc_pitch_demand
        ),
        'va_hold_max_abs_mc_pitch_torque_demand_normalized': max(
            (abs(value) for value in measurement_mc_pitch_demand),
            default=math.nan,
        ),
        'va_hold_mean_fw_pitch_torque_demand_normalized': _mean(
            measurement_fw_pitch_demand
        ),
        'va_hold_max_abs_fw_pitch_torque_demand_normalized': max(
            (abs(value) for value in measurement_fw_pitch_demand),
            default=math.nan,
        ),
        'va_hold_mean_mc_pitch_torque_contribution_normalized': _mean(
            measurement_mc_pitch_contribution
        ),
        'va_hold_mean_fw_pitch_torque_contribution_normalized': _mean(
            measurement_fw_pitch_contribution
        ),
        'va_hold_pitch_tracking_rmse_rad': _rmse(
            measurement_pitch_tracking_error
        ),
        'va_hold_pitch_rate_rmse_rad_s': _rmse(measurement_q),
        'va_hold_pitch_rate_p95_rad_s': _percentile(
            [abs(value) for value in measurement_q], 0.95
        ),
        'va_hold_servo_saturation_fraction': (
            measurement_servo_saturation_fraction
        ),
        # `servo_2` is a GZ bridge radian command; standard_vtol clips its
        # elevator joint at +/-0.53 rad.  This is a command-clipping proxy,
        # not a direct joint-position measurement.
        'va_hold_elevator_joint_limit_command_fraction': (
            measurement_elevator_joint_limit_command_fraction
        ),
        'va_hold_pusher_throttle_upper_saturation_fraction': (
            measurement_pusher_throttle_upper_saturation_fraction
        ),
        'va_hold_pusher_rotor_saturation_fraction': (
            measurement_pusher_rotor_saturation_fraction
        ),
        'max_lambda_command': max_lambda_command,
        'max_lambda_exec': max_lambda_exec,
        'lambda_published_samples': len(published_rows),
        'lambda_tracking_samples': len(tracking_errors),
        'lambda_external_active_fraction': external_active_fraction,
        'lambda_command_to_active_delay_s': command_to_active_delay,
        'mean_lambda_tracking_error': mean_tracking_error,
        'max_lambda_tracking_error': max_tracking_error,
        'lambda_target_config': lambda_target,
        'lambda_target_tolerance': lambda_target_tolerance,
        'lambda_target_settling_time_s': lambda_target_settling_time,
        'lambda_response_rate_mean_per_s': _mean(positive_response_rates),
        'lambda_response_rate_peak_per_s': max(
            positive_response_rates, default=math.nan
        ),
        'lambda_steady_state_bias': steady_state_bias,
        'lambda_steady_state_mean_abs_error': steady_state_mean_abs_error,
        'lambda_steady_state_max_abs_error': steady_state_max_abs_error,
        'lambda_overshoot': lambda_overshoot,
        'lambda_target_reached': bool(target_rows),
        'lambda_characterization_complete': lambda_characterization_complete,
        'energy_measurement_kind': 'static_motor_model_effort_proxy',
        # The stock Gazebo motor model has no advance-ratio dependence and
        # violates P >= T*V in forward flight.  Do not expose its integral as
        # physical propulsion energy; retain it only in the explicit proxy
        # fields below.
        'transition_lift_energy_j': math.nan,
        'transition_pusher_energy_j': math.nan,
        'transition_total_propulsion_energy_j': math.nan,
        'transition_lift_energy_proxy_j': lift_energy,
        'transition_pusher_energy_proxy_j': pusher_energy,
        'transition_total_propulsion_energy_proxy_j': total_energy,
        'pre_8mps_lift_energy_proxy_j': pre_lift,
        'pre_8mps_pusher_energy_proxy_j': pre_pusher,
        'pre_8mps_total_energy_proxy_j': pre_total,
        'from_8mps_lift_energy_proxy_j': post_lift,
        'from_8mps_pusher_energy_proxy_j': post_pusher,
        'from_8mps_total_energy_proxy_j': post_total,
        'pre_8mps_energy_fraction': pre_energy_fraction,
        'lift_power_proxy_by_airspeed_bin_w': lift_power_by_airspeed_bin,
        'telemetry_completeness': completeness,
        'transition_telemetry_completeness': transition_completeness,
        'telemetry_freshness_limit_s': FRESHNESS_LIMIT_S,
        'test_1_native_transition_pass': baseline_pass,
        'test_2_nominal_50m_pass': constant_altitude_pass,
        'test_3_telemetry_pass': telemetry_pass,
        'test_4_manual_lambda_pass': manual_lambda_pass,
        'phase_0_5_pusher_diagnostic_pass': pusher_diagnostic_pass,
        'phase_0_5_lambda_characterization_pass': characterization_pass,
        'phase_0_75_va_hold_pass': va_hold_pass,
        'f1_measurement_pass': f1_measurement_pass,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('csv', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument(
        '--require',
        choices=('native', 'manual', 'characterization', 'va_hold', 'f1'),
    )
    arguments = parser.parse_args()
    result = evaluate(arguments.csv)
    payload = json.dumps(_json_safe(result), indent=2, allow_nan=False)
    if arguments.output:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(payload + '\n', encoding='utf-8')
    print(payload)
    common = (
        'test_1_native_transition_pass',
        'test_2_nominal_50m_pass',
        'test_3_telemetry_pass',
        'phase_0_5_pusher_diagnostic_pass',
    )
    required = (
        common
        if arguments.require == 'native'
        else common + ('test_4_manual_lambda_pass',)
        if arguments.require == 'manual'
        else common + ('phase_0_5_lambda_characterization_pass',)
        if arguments.require == 'characterization'
        else common + ('phase_0_75_va_hold_pass',)
        if arguments.require == 'va_hold'
        else ('f1_measurement_pass',)
        if arguments.require == 'f1'
        else ()
    )
    if required and not all(bool(result[key]) for key in required):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
