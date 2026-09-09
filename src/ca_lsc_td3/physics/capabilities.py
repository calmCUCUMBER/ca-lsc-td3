"""Paper-level definitions for transition capability estimation."""

from __future__ import annotations

from dataclasses import dataclass
import math


_EPSILON = 1.0e-8


@dataclass(frozen=True)
class WingSupportEstimate:
    """Auditable intermediate quantities used to form :math:`eta_L`."""

    dynamic_pressure_pa: float
    wing_lift_n: float
    vertical_support_n: float
    required_support_n: float
    eta_l: float


@dataclass(frozen=True)
class ShadowPitchMomentEstimate:
    """Physical shadow-controller demand reconstructed at the elevator."""

    target_elevator_angle_rad: float
    target_pitch_moment_nm: float
    current_pitch_moment_nm: float
    increment_pitch_moment_nm: float


@dataclass(frozen=True)
class ControlAuthorityEstimate:
    """Auditable intermediate quantities used to form :math:`eta_C`."""

    pressure_gate: float
    remaining_elevator_angle_rad: float
    available_increment_moment_nm: float
    requested_increment_moment_nm: float
    moment_margin: float
    eta_c: float


def _unit_interval(value: float) -> float:
    """Clamp a finite scalar to [0, 1]."""
    if not math.isfinite(value):
        raise ValueError("capability values must be finite")
    return min(1.0, max(0.0, float(value)))


def smoothstep(value: float, lower: float, upper: float) -> float:
    """Return the cubic smooth gate S(value; lower, upper)."""
    if not all(math.isfinite(item) for item in (value, lower, upper)):
        raise ValueError("smoothstep inputs must be finite")
    if upper <= lower:
        raise ValueError("smoothstep upper bound must exceed lower bound")
    if value <= lower:
        return 0.0
    if value >= upper:
        return 1.0
    xi = (value - lower) / (upper - lower)
    return float(3.0 * xi**2 - 2.0 * xi**3)


def piecewise_lift_coefficient(
    *,
    geometric_alpha_rad: float,
    alpha_offset_rad: float,
    lift_slope_per_rad: float,
    stall_angle_rad: float,
    post_stall_slope_per_rad: float,
    control_lift_increment: float = 0.0,
) -> float:
    """Evaluate the Gazebo ``LiftDrag`` piecewise :math:`C_L` law.

    ``geometric_alpha_rad`` deliberately excludes the SDF ``a0`` offset.
    The post-stall sign clamp matches Gazebo Sim 8's implementation.
    """
    values = (
        geometric_alpha_rad,
        alpha_offset_rad,
        lift_slope_per_rad,
        stall_angle_rad,
        post_stall_slope_per_rad,
        control_lift_increment,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("lift-coefficient inputs must be finite")
    if stall_angle_rad <= 0.0:
        raise ValueError("stall angle must be positive")
    alpha = geometric_alpha_rad + alpha_offset_rad
    if alpha > stall_angle_rad:
        coefficient = max(
            0.0,
            lift_slope_per_rad * stall_angle_rad
            + post_stall_slope_per_rad * (alpha - stall_angle_rad),
        )
    elif alpha < -stall_angle_rad:
        coefficient = min(
            0.0,
            -lift_slope_per_rad * stall_angle_rad
            + post_stall_slope_per_rad * (alpha + stall_angle_rad),
        )
    else:
        coefficient = lift_slope_per_rad * alpha
    return float(coefficient + control_lift_increment)


def estimate_wing_vertical_support(
    *,
    air_density_kg_m3: float,
    airspeed_mps: float,
    wing_area_m2: float,
    lift_coefficient: float,
    flight_path_angle_rad: float,
    roll_angle_rad: float,
    estimated_mass_kg: float,
    desired_vertical_acceleration_mps2: float = 0.0,
    gravity_mps2: float = 9.80665,
    minimum_valid_airspeed_mps: float = 5.0,
    epsilon: float = _EPSILON,
) -> WingSupportEstimate:
    """Return :math:`eta_L` and every dimensional term used to form it."""
    scalars = (
        air_density_kg_m3,
        airspeed_mps,
        wing_area_m2,
        lift_coefficient,
        flight_path_angle_rad,
        roll_angle_rad,
        estimated_mass_kg,
        desired_vertical_acceleration_mps2,
        gravity_mps2,
        minimum_valid_airspeed_mps,
        epsilon,
    )
    if not all(math.isfinite(item) for item in scalars):
        raise ValueError("eta_L inputs must be finite")
    if air_density_kg_m3 < 0.0 or airspeed_mps < 0.0:
        raise ValueError("air density and airspeed must be non-negative")
    if minimum_valid_airspeed_mps <= 0.0:
        raise ValueError("minimum valid airspeed must be positive")
    if wing_area_m2 <= 0.0 or estimated_mass_kg <= 0.0:
        raise ValueError("wing area and estimated mass must be positive")
    if gravity_mps2 <= 0.0 or epsilon <= 0.0:
        raise ValueError("gravity and epsilon must be positive")

    dynamic_pressure_pa = 0.5 * air_density_kg_m3 * airspeed_mps**2
    required_support_n = estimated_mass_kg * (
        gravity_mps2 + desired_vertical_acceleration_mps2
    )
    if required_support_n <= 0.0:
        raise ValueError("required vertical support must be positive")
    if airspeed_mps < minimum_valid_airspeed_mps:
        return WingSupportEstimate(
            dynamic_pressure_pa=dynamic_pressure_pa,
            wing_lift_n=0.0,
            vertical_support_n=0.0,
            required_support_n=required_support_n,
            eta_l=0.0,
        )

    wing_lift_n = dynamic_pressure_pa * wing_area_m2 * lift_coefficient
    vertical_support_n = (
        wing_lift_n
        * math.cos(flight_path_angle_rad)
        * math.cos(roll_angle_rad)
    )
    return WingSupportEstimate(
        dynamic_pressure_pa=dynamic_pressure_pa,
        wing_lift_n=wing_lift_n,
        vertical_support_n=vertical_support_n,
        required_support_n=required_support_n,
        eta_l=_unit_interval(
            vertical_support_n / (required_support_n + epsilon)
        ),
    )


def wing_vertical_support_capability(
    *,
    air_density_kg_m3: float,
    airspeed_mps: float,
    wing_area_m2: float,
    lift_coefficient: float,
    flight_path_angle_rad: float,
    roll_angle_rad: float,
    estimated_mass_kg: float,
    desired_vertical_acceleration_mps2: float = 0.0,
    gravity_mps2: float = 9.80665,
    minimum_valid_airspeed_mps: float = 5.0,
    epsilon: float = _EPSILON,
) -> float:
    """Compute eta_L, the estimated wing vertical-support ratio.

    `estimated_mass_kg` is intentionally required. Passing the simulator's
    hidden true mass would invalidate the intended robustness experiment.
    Flight-path angle and desired vertical acceleration use an up-positive
    convention; convert PX4 NED telemetry before calling this function.
    """
    return estimate_wing_vertical_support(
        air_density_kg_m3=air_density_kg_m3,
        airspeed_mps=airspeed_mps,
        wing_area_m2=wing_area_m2,
        lift_coefficient=lift_coefficient,
        flight_path_angle_rad=flight_path_angle_rad,
        roll_angle_rad=roll_angle_rad,
        estimated_mass_kg=estimated_mass_kg,
        desired_vertical_acceleration_mps2=(
            desired_vertical_acceleration_mps2
        ),
        gravity_mps2=gravity_mps2,
        minimum_valid_airspeed_mps=minimum_valid_airspeed_mps,
        epsilon=epsilon,
    ).eta_l


def shadow_pitch_moment_increment(
    *,
    dynamic_pressure_pa: float,
    dimensional_moment_derivative_per_q_m3: float,
    normalized_shadow_pitch_demand: float,
    current_elevator_angle_rad: float,
    elevator_min_rad: float,
    elevator_max_rad: float,
    command_radians_per_normalized: float = 1.0,
) -> ShadowPitchMomentEstimate:
    """Map the unweighted PX4 FW demand into a physical moment increment.

    The current PX4 Gazebo servo bridge sends normalized control directly as
    a radian joint-position command.  The explicit scale argument keeps that
    simulator-specific mapping visible and independently testable.
    """
    values = (
        dynamic_pressure_pa,
        dimensional_moment_derivative_per_q_m3,
        normalized_shadow_pitch_demand,
        current_elevator_angle_rad,
        elevator_min_rad,
        elevator_max_rad,
        command_radians_per_normalized,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("shadow pitch-moment inputs must be finite")
    if dynamic_pressure_pa < 0.0:
        raise ValueError("dynamic pressure must be non-negative")
    if elevator_max_rad <= elevator_min_rad:
        raise ValueError("elevator limits are invalid")
    if not elevator_min_rad <= current_elevator_angle_rad <= elevator_max_rad:
        raise ValueError("current elevator angle lies outside its limits")
    if command_radians_per_normalized <= 0.0:
        raise ValueError("command scale must be positive")
    normalized = min(1.0, max(-1.0, normalized_shadow_pitch_demand))
    target_angle = min(
        elevator_max_rad,
        max(elevator_min_rad, normalized * command_radians_per_normalized),
    )
    derivative_nm_per_rad = (
        dynamic_pressure_pa * dimensional_moment_derivative_per_q_m3
    )
    target_moment = derivative_nm_per_rad * target_angle
    current_moment = derivative_nm_per_rad * current_elevator_angle_rad
    return ShadowPitchMomentEstimate(
        target_elevator_angle_rad=target_angle,
        target_pitch_moment_nm=target_moment,
        current_pitch_moment_nm=current_moment,
        increment_pitch_moment_nm=target_moment - current_moment,
    )


def _control_authority_from_moment_derivative(
    *,
    dynamic_pressure_pa: float,
    moment_derivative_nm_per_rad: float,
    elevator_angle_rad: float,
    elevator_min_rad: float,
    elevator_max_rad: float,
    shadow_pitch_moment_increment_nm: float,
    pressure_gate_lower_pa: float,
    pressure_gate_upper_pa: float,
    epsilon: float,
) -> ControlAuthorityEstimate:
    if abs(moment_derivative_nm_per_rad) <= epsilon:
        return ControlAuthorityEstimate(
            pressure_gate=smoothstep(
                dynamic_pressure_pa,
                pressure_gate_lower_pa,
                pressure_gate_upper_pa,
            ),
            remaining_elevator_angle_rad=0.0,
            available_increment_moment_nm=0.0,
            requested_increment_moment_nm=shadow_pitch_moment_increment_nm,
            moment_margin=0.0,
            eta_c=0.0,
        )

    elevator_increment_direction = (
        shadow_pitch_moment_increment_nm / moment_derivative_nm_per_rad
    )
    if elevator_increment_direction > 0.0:
        remaining_angle_rad = elevator_max_rad - elevator_angle_rad
    elif elevator_increment_direction < 0.0:
        remaining_angle_rad = elevator_angle_rad - elevator_min_rad
    else:
        remaining_angle_rad = min(
            elevator_max_rad - elevator_angle_rad,
            elevator_angle_rad - elevator_min_rad,
        )
    available_moment_nm = (
        abs(moment_derivative_nm_per_rad) * max(0.0, remaining_angle_rad)
    )
    pressure_gate = smoothstep(
        dynamic_pressure_pa,
        pressure_gate_lower_pa,
        pressure_gate_upper_pa,
    )
    if available_moment_nm <= epsilon:
        margin = 0.0
    else:
        margin = _unit_interval(
            1.0
            - abs(shadow_pitch_moment_increment_nm)
            / (available_moment_nm + epsilon)
        )
    return ControlAuthorityEstimate(
        pressure_gate=pressure_gate,
        remaining_elevator_angle_rad=max(0.0, remaining_angle_rad),
        available_increment_moment_nm=available_moment_nm,
        requested_increment_moment_nm=shadow_pitch_moment_increment_nm,
        moment_margin=margin,
        eta_c=_unit_interval(pressure_gate * margin),
    )


def aerodynamic_control_authority_from_dimensional_derivative(
    *,
    dynamic_pressure_pa: float,
    dimensional_moment_derivative_per_q_m3: float,
    elevator_angle_rad: float,
    elevator_min_rad: float,
    elevator_max_rad: float,
    shadow_pitch_moment_increment_nm: float,
    pressure_gate_lower_pa: float,
    pressure_gate_upper_pa: float,
    epsilon: float = _EPSILON,
) -> ControlAuthorityEstimate:
    """Compute :math:`eta_C` from the SDF dimensional elevator derivative."""
    scalars = (
        dynamic_pressure_pa,
        dimensional_moment_derivative_per_q_m3,
        elevator_angle_rad,
        elevator_min_rad,
        elevator_max_rad,
        shadow_pitch_moment_increment_nm,
        pressure_gate_lower_pa,
        pressure_gate_upper_pa,
        epsilon,
    )
    if not all(math.isfinite(item) for item in scalars):
        raise ValueError("eta_C inputs must be finite")
    if dynamic_pressure_pa < 0.0:
        raise ValueError("dynamic pressure must be non-negative")
    if pressure_gate_lower_pa < 0.0:
        raise ValueError("dynamic-pressure gate cannot start below zero")
    if pressure_gate_upper_pa <= pressure_gate_lower_pa:
        raise ValueError("dynamic-pressure gate bounds are invalid")
    if elevator_max_rad <= elevator_min_rad:
        raise ValueError("elevator limits are invalid")
    if not elevator_min_rad <= elevator_angle_rad <= elevator_max_rad:
        raise ValueError("elevator angle must lie inside its limits")
    if epsilon <= 0.0:
        raise ValueError("epsilon must be positive")
    return _control_authority_from_moment_derivative(
        dynamic_pressure_pa=dynamic_pressure_pa,
        moment_derivative_nm_per_rad=(
            dynamic_pressure_pa * dimensional_moment_derivative_per_q_m3
        ),
        elevator_angle_rad=elevator_angle_rad,
        elevator_min_rad=elevator_min_rad,
        elevator_max_rad=elevator_max_rad,
        shadow_pitch_moment_increment_nm=shadow_pitch_moment_increment_nm,
        pressure_gate_lower_pa=pressure_gate_lower_pa,
        pressure_gate_upper_pa=pressure_gate_upper_pa,
        epsilon=epsilon,
    )


def aerodynamic_control_authority(
    *,
    dynamic_pressure_pa: float,
    wing_area_m2: float,
    mean_aerodynamic_chord_m: float,
    pitch_moment_derivative_per_rad: float,
    elevator_angle_rad: float,
    elevator_min_rad: float,
    elevator_max_rad: float,
    shadow_pitch_moment_increment_nm: float,
    pressure_gate_lower_pa: float,
    pressure_gate_upper_pa: float,
    epsilon: float = _EPSILON,
) -> float:
    """Compute eta_C using direction-aware remaining elevator authority.

    `shadow_pitch_moment_increment_nm` is the additional aerodynamic pitch
    moment requested relative to the moment produced at the current elevator
    angle. A controller that outputs total unsaturated moment demand must
    subtract the current estimated elevator moment before calling this
    function.
    """
    scalars = (
        dynamic_pressure_pa,
        wing_area_m2,
        mean_aerodynamic_chord_m,
        pitch_moment_derivative_per_rad,
        elevator_angle_rad,
        elevator_min_rad,
        elevator_max_rad,
        shadow_pitch_moment_increment_nm,
        pressure_gate_lower_pa,
        pressure_gate_upper_pa,
        epsilon,
    )
    if not all(math.isfinite(item) for item in scalars):
        raise ValueError("eta_C inputs must be finite")
    if dynamic_pressure_pa < 0.0:
        raise ValueError("dynamic pressure must be non-negative")
    if pressure_gate_lower_pa < 0.0:
        raise ValueError("dynamic-pressure gate cannot start below zero")
    if pressure_gate_upper_pa <= pressure_gate_lower_pa:
        raise ValueError("dynamic-pressure gate bounds are invalid")
    if wing_area_m2 <= 0.0 or mean_aerodynamic_chord_m <= 0.0:
        raise ValueError("wing area and mean chord must be positive")
    if elevator_max_rad <= elevator_min_rad:
        raise ValueError("elevator limits are invalid")
    if not elevator_min_rad <= elevator_angle_rad <= elevator_max_rad:
        raise ValueError("elevator angle must lie inside its limits")
    if epsilon <= 0.0:
        raise ValueError("epsilon must be positive")

    return _control_authority_from_moment_derivative(
        dynamic_pressure_pa=dynamic_pressure_pa,
        moment_derivative_nm_per_rad=(
            dynamic_pressure_pa
            * wing_area_m2
            * mean_aerodynamic_chord_m
            * pitch_moment_derivative_per_rad
        ),
        elevator_angle_rad=elevator_angle_rad,
        elevator_min_rad=elevator_min_rad,
        elevator_max_rad=elevator_max_rad,
        shadow_pitch_moment_increment_nm=shadow_pitch_moment_increment_nm,
        pressure_gate_lower_pa=pressure_gate_lower_pa,
        pressure_gate_upper_pa=pressure_gate_upper_pa,
        epsilon=epsilon,
    ).eta_c


def physics_prior(
    eta_l: float,
    eta_c: float,
    *,
    eta_l_lower: float,
    eta_l_upper: float,
    eta_c_lower: float,
    eta_c_upper: float,
) -> float:
    """Compute lambda_phy from smoothly gated eta_L and eta_C."""
    thresholds = (
        eta_l_lower,
        eta_l_upper,
        eta_c_lower,
        eta_c_upper,
    )
    if not all(math.isfinite(value) for value in thresholds):
        raise ValueError("physics-prior thresholds must be finite")
    if not all(0.0 <= value <= 1.0 for value in thresholds):
        raise ValueError("physics-prior thresholds must lie in [0, 1]")
    eta_l = _unit_interval(eta_l)
    eta_c = _unit_interval(eta_c)
    g_l = smoothstep(eta_l, eta_l_lower, eta_l_upper)
    g_c = smoothstep(eta_c, eta_c_lower, eta_c_upper)
    return _unit_interval(g_l * g_c)
