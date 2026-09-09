"""Deterministic hand-written unloading schedules used before RL."""

import math


def smoothstep(value: float, lower: float, upper: float) -> float:
    """Return a clamped cubic smoothstep between two finite bounds."""
    if not all(math.isfinite(item) for item in (value, lower, upper)):
        raise ValueError('smoothstep inputs must be finite')
    if upper <= lower:
        raise ValueError('upper must be greater than lower')
    ratio = min(max((value - lower) / (upper - lower), 0.0), 1.0)
    return ratio * ratio * (3.0 - 2.0 * ratio)


def airspeed_unloading(
    airspeed_mps: float,
    start_mps: float,
    full_mps: float,
    maximum: float,
) -> float:
    """Map airspeed to a bounded manual lift-rotor unloading command."""
    if not math.isfinite(maximum) or not 0.0 <= maximum <= 1.0:
        raise ValueError('maximum must be within [0, 1]')
    if not math.isfinite(airspeed_mps):
        return 0.0
    return maximum * smoothstep(airspeed_mps, start_mps, full_mps)


def transition_attitude_weights(
    lambda_exec: float,
    lambda_start: float = 0.1,
    lambda_full: float = 0.9,
) -> tuple[float, float]:
    """Return the MC/FW weights synchronized to executed transition lambda.

    This mirrors the PX4 implementation.  It is deliberately piecewise
    linear: a small initial unloading interval retains full MC attitude
    authority, and full FW authority is reached only near complete nominal
    unloading.  The pusher remains outside this allocation.
    """
    if not all(math.isfinite(value) for value in (
        lambda_exec, lambda_start, lambda_full
    )):
        raise ValueError('transition-allocation inputs must be finite')
    if not 0.0 <= lambda_exec <= 1.0:
        raise ValueError('lambda_exec must lie in [0, 1]')
    if not 0.0 <= lambda_start < lambda_full <= 1.0:
        raise ValueError(
            'require 0 <= lambda_start < lambda_full <= 1'
        )
    fw_weight = min(max(
        (lambda_exec - lambda_start) / (lambda_full - lambda_start),
        0.0,
    ), 1.0)
    return 1.0 - fw_weight, fw_weight


def altitude_hold_down_velocity(
    altitude_m: float,
    target_altitude_m: float,
    vertical_speed_up_mps: float,
    proportional_gain: float,
    vertical_speed_gain: float,
    maximum_speed_mps: float,
) -> float:
    """Return a bounded NED/down-positive vertical-speed correction.

    A positive altitude error or positive climb rate produces a positive
    (downward) command.  The correction is intentionally one-sided: PX4's
    native height loop remains responsible for climbing back from an altitude
    deficit, while this outer loop only suppresses the high-speed excess-lift
    climb.  This also avoids a small upward feed-forward command winding the
    transition height controller to maximum collective at mode handover.
    Missing state feedback falls back to zero.
    """
    gains = (proportional_gain, vertical_speed_gain, maximum_speed_mps)
    if not all(math.isfinite(value) for value in gains):
        raise ValueError('altitude-hold gains and limit must be finite')
    if proportional_gain < 0.0 or vertical_speed_gain < 0.0:
        raise ValueError('altitude-hold gains must be non-negative')
    if maximum_speed_mps <= 0.0:
        raise ValueError('maximum_speed_mps must be positive')
    if not (
        math.isfinite(altitude_m)
        and math.isfinite(target_altitude_m)
    ):
        return 0.0
    vertical_speed = (
        vertical_speed_up_mps
        if math.isfinite(vertical_speed_up_mps) else 0.0
    )
    command = (
        proportional_gain * (altitude_m - target_altitude_m)
        + vertical_speed_gain * vertical_speed
    )
    return min(max(command, 0.0), maximum_speed_mps)
