"""Pure helpers shared by transition telemetry and its unit tests."""

from __future__ import annotations

import math
from typing import Mapping


def a0_transition_terminal_reason(
    previous_state: str,
    current_state: str,
    command_error: str,
    existing_reason: str = '',
) -> str:
    """Classify an A0 terminal from the authoritative command-node state.

    A capability dwell only releases the external transition hold; it is not
    success.  Success is emitted exclusively when the command node confirms
    ``TRANSITION_FW -> HOLD_FW``.  An already-latched physical terminal always
    wins over later state/error messages.
    """
    if existing_reason:
        return existing_reason
    if previous_state != 'TRANSITION_FW' or current_state == 'TRANSITION_FW':
        return ''
    if current_state == 'HOLD_FW':
        return 'success'
    normalized_error = command_error.strip().lower().replace(' ', '_')
    if normalized_error == 'forward_transition_timed_out':
        return 'forward_transition_timeout'
    return normalized_error


def sustained_sim_time(
    active: bool,
    started_at_s: float | None,
    current_time_s: float,
    dwell_s: float,
) -> tuple[float | None, bool]:
    """Evaluate a sustained condition using simulation time.

    Clock rollback starts a new dwell.  This keeps task termination invariant
    under x1/x4 simulation speed while wall time remains available solely for
    process watchdogs and transport diagnostics.
    """
    if not math.isfinite(dwell_s) or dwell_s <= 0.0:
        raise ValueError('dwell_s must be finite and positive')
    if not active or not math.isfinite(current_time_s):
        return None, False
    if (
        started_at_s is None
        or not math.isfinite(started_at_s)
        or current_time_s < started_at_s
    ):
        return float(current_time_s), False
    return started_at_s, current_time_s - started_at_s >= dwell_s


def gated_angle_of_attack(
    raw_alpha_rad: float,
    airspeed_mps: float,
    minimum_airspeed_mps: float,
) -> tuple[float, bool]:
    """Return a usable AoA only when the wind-axis state is observable.

    The nominal recorder currently derives body-relative direction from local
    velocity, so this gate is valid only for the no-wind Phase-0/0.5 world.
    Wind experiments must replace that estimate with air-relative velocity.
    """
    if not math.isfinite(minimum_airspeed_mps) or minimum_airspeed_mps <= 0.0:
        raise ValueError('minimum AoA airspeed must be positive')
    valid = (
        math.isfinite(raw_alpha_rad)
        and math.isfinite(airspeed_mps)
        and airspeed_mps >= minimum_airspeed_mps
    )
    return (float(raw_alpha_rad), True) if valid else (math.nan, False)


def within_lambda_target(
    executed: float,
    target: float,
    tolerance: float,
) -> bool:
    """Return whether an executed unloading command is in its dwell band."""
    if not all(math.isfinite(value) for value in (target, tolerance)):
        raise ValueError('lambda target and tolerance must be finite')
    if not 0.0 <= target <= 1.0:
        raise ValueError('lambda target must lie in [0, 1]')
    if tolerance <= 0.0 or tolerance > 1.0:
        raise ValueError('lambda tolerance must lie in (0, 1]')
    return math.isfinite(executed) and abs(executed - target) <= tolerance


def transition_mc_weight_proxy(
    airspeed_mps: float,
    blend_airspeed_mps: float,
    transition_airspeed_mps: float,
) -> float:
    """Reconstruct PX4 Standard VTOL's airspeed-based MC control weight.

    This is deliberately named a proxy: PX4 also gates blending on transition
    time and airspeed validity.  During the controlled-unloading measurement
    phase those gates have already been satisfied, so the proxy exposes the
    control-authority handover that is otherwise absent from DDS telemetry.
    """
    if not all(math.isfinite(value) for value in (
        blend_airspeed_mps, transition_airspeed_mps,
    )):
        raise ValueError('VTOL blend and transition airspeeds must be finite')
    if blend_airspeed_mps < 0.0:
        raise ValueError('VTOL blend airspeed must be non-negative')
    if transition_airspeed_mps < blend_airspeed_mps:
        raise ValueError(
            'VTOL transition airspeed must be >= blend airspeed'
        )
    if not math.isfinite(airspeed_mps):
        return math.nan
    margin = transition_airspeed_mps - blend_airspeed_mps
    if margin <= 0.0 or airspeed_mps < blend_airspeed_mps:
        return 1.0
    return min(max(
        1.0 - abs(airspeed_mps - blend_airspeed_mps) / margin,
        0.0,
    ), 1.0)


def wind_relative_velocity_enu(
    north_mps: float,
    east_mps: float,
    up_mps: float,
    wind_east_mps: float,
    wind_north_mps: float,
    wind_up_mps: float,
) -> tuple[float, float, float, float]:
    """Return aircraft velocity relative to air in world ENU coordinates.

    PX4 local position velocities are logged as NED-like components
    ``north/east/down``; the recorder converts the vertical component to
    ``up_mps``.  Gazebo world wind is configured in ENU.  The physical
    relative-air velocity used for wind qualification is therefore

    ``V_rel = V_ground_enu - V_wind_enu``.
    """
    values = (
        north_mps, east_mps, up_mps,
        wind_east_mps, wind_north_mps, wind_up_mps,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return (math.nan, math.nan, math.nan, math.nan)
    relative_east = float(east_mps) - float(wind_east_mps)
    relative_north = float(north_mps) - float(wind_north_mps)
    relative_up = float(up_mps) - float(wind_up_mps)
    norm = math.sqrt(
        relative_east * relative_east
        + relative_north * relative_north
        + relative_up * relative_up
    )
    return relative_east, relative_north, relative_up, norm
def transition_start_readiness(
    metadata: Mapping[str, float],
) -> tuple[bool, bool, bool, bool]:
    """Return the command-node readiness snapshot at transition entry.

    These values must be latched from the ``HOLD_MC -> TRANSITION_FW`` state
    message.  Re-evaluating the gates after transition entry is invalid:
    groundspeed and vertical speed are expected to change as acceleration
    begins, and an x4 subscriber may legitimately receive a later sample.
    """
    keys = ("height_gate_ok", "vz_gate_ok", "groundspeed_gate_ok")
    available = all(key in metadata for key in keys)
    if not available:
        return False, False, False, False
    height_ok, vz_ok, groundspeed_ok = (
        bool(float(metadata[key]) >= 0.5) for key in keys
    )
    return (
        height_ok,
        vz_ok,
        groundspeed_ok,
        height_ok and vz_ok and groundspeed_ok,
    )
