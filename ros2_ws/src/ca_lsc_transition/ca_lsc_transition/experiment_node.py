"""Publish a manual lambda schedule and record synchronized test telemetry."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import time
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped, WrenchStamped
from px4_msgs.msg import (
    ActuatorMotors,
    ActuatorServos,
    AirspeedValidated,
    BatteryStatus,
    EscStatus,
    NormalizedUnsignedSetpoint,
    TrajectorySetpoint,
    VehicleAngularVelocity,
    VehicleAttitude,
    VehicleAttitudeSetpoint,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleThrustSetpoint,
    VehicleTorqueSetpoint,
    VtolVehicleStatus,
)
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, String

from ca_lsc_td3.physics.capabilities import (
    aerodynamic_control_authority_from_dimensional_derivative,
    estimate_wing_vertical_support,
    piecewise_lift_coefficient,
    physics_prior,
    smoothstep,
    shadow_pitch_moment_increment,
)
from ca_lsc_td3.rl.observations import (
    A0_OBSERVATION_FIELDS,
    pack_a0_observation,
)
from ca_lsc_td3.safety.shield import (
    AsymmetricRateLimiter,
    apply_unloading_shield,
)

from .schedule import (
    airspeed_unloading,
    altitude_hold_down_velocity,
    transition_attitude_weights,
)
from .diagnostics import (
    a0_transition_terminal_reason,
    gated_angle_of_attack,
    sustained_sim_time,
    transition_start_readiness,
    transition_mc_weight_proxy,
    within_lambda_target,
    wind_relative_velocity_enu,
)


FIELDS = (
    'time_s', 'schema_version', 'condition_id', 'wind_model_config',
    'sim_speed_factor_config', 'a0_observation_sim_dt_s',
    'a0_observation_wall_dt_s', 'schedule_mode', 'command_state',
    'a0_rl_action_seq', 'a0_rl_action', 'a0_rl_lambda_d',
    'a0_rl_action_source', 'a0_rl_action_age_s', 'a0_rl_action_stale',
    'a0_rl_control_phase', 'a0_rl_handover_ready',
    'a0_rl_handover_ready_dwell_s', 'a0_rl_actual_fw_confirmed',
    'a0_rl_initial_readiness_available',
    'a0_rl_initial_readiness_latched',
    'a0_rl_initial_height_gate_ok', 'a0_rl_initial_vz_gate_ok',
    'a0_rl_initial_groundspeed_gate_ok',
    'a0_rl_terminal_reason',
    'f3b2_test_mode', 'f3b2_shadow_demand_offset_nm',
    'f3b2_shadow_demand_for_eta_c_nm',
    'f3b2_elevator_bias_rad', 'f3b2_stage_index',
    'vtol_state', 'nav_state', 'armed',
    'failsafe', 'north_m', 'east_m',
    'altitude_local_m', 'altitude_datum_local_m',
    'altitude_relative_m', 'target_altitude_local_m',
    'target_altitude_relative_m', 'altitude_m', 'altitude_error_m',
    'trajectory_sp_altitude_local_m', 'trajectory_sp_altitude_relative_m',
    'trajectory_sp_vz_up_mps',
    'height_gate_ok', 'vz_gate_ok', 'groundspeed_gate_ok',
    'stability_gate_ok', 'stability_dwell_s',
    'va_target_config', 'va_target_tolerance', 'va_error_mps',
    'va_measurement_min_band_fraction_config',
    'va_measurement_max_abs_airspeed_error_mps_config',
    'va_target_dwell_s_config', 'lambda_target_dwell_s_config',
    'va_measurement_duration_s_config',
    'vt_unload_altitude_pitch_kp_config',
    'vt_unload_vertical_speed_pitch_kd_config',
    'vt_unload_pitch_min_deg_config', 'vt_unload_pitch_max_deg_config',
    'vt_airspeed_blend_mps_config', 'vt_transition_airspeed_mps_config',
    'lambda_attitude_blend_start_config',
    'lambda_attitude_blend_full_config',
    'mc_pitch_weight_proxy', 'mc_pitch_weight_actual',
    'fw_pitch_weight_actual', 'mc_pitch_weight_lambda_expected',
    'fw_pitch_weight_lambda_expected',
    'mc_pitch_torque_demand_normalized',
    'fw_pitch_torque_demand_normalized',
    'mc_pitch_torque_contribution_normalized',
    'fw_pitch_torque_contribution_normalized',
    'va_hold_velocity_command_mps',
    'va_hold_down_velocity_command_mps', 'va_hold_phase',
    'va_hold_measurement_active', 'va_hold_measurement_complete',
    'va_hold_measurement_interrupted',
    'va_hold_abort_reason', 'va_vertical_abort_enabled',
    'va_low_speed_vertical_transient',
    'vn_mps', 've_mps', 'vz_up_mps', 'groundspeed_mps', 'airspeed_mps',
    'airspeed_source_config', 'selected_airspeed_mps',
    'selected_airspeed_error_mps',
    'wind_cmd_e_enu_mps', 'wind_cmd_n_enu_mps', 'wind_cmd_u_enu_mps',
    'wind_actual_e_enu_mps', 'wind_actual_n_enu_mps',
    'wind_actual_u_enu_mps', 'wind_heading_parallel_mps',
    'wind_heading_cross_mps', 'wind_actual_source',
    'gust_amplitude_mps_config', 'gust_delay_s_config',
    'gust_duration_s_config', 'gust_recovery_s_config',
    'gust_phase', 'gust_phase_elapsed_s', 'gust_profile_scale',
    'gust_active', 'gust_bridge_ready', 'gust_status_age_s',
    'vg_e_enu_mps', 'vg_n_enu_mps', 'vg_u_enu_mps',
    'vrel_e_enu_mps', 'vrel_n_enu_mps', 'vrel_u_enu_mps',
    'vrel_norm_mps', 'vrel_body_u_mps', 'vrel_body_v_mps',
    'vrel_body_w_mps', 'beta_est_rad', 'beta_valid',
    'airspeed_relative_error_mps',
    'mass_scale_config', 'nominal_model_mass_kg_config',
    'actual_model_mass_kg_config', 'payload_mass_kg_config',
    'wing_lift_gt_enabled', 'wing_lift_gt_valid',
    'wing_lift_left_fx_enu_n', 'wing_lift_left_fy_enu_n',
    'wing_lift_left_fz_enu_n', 'wing_lift_right_fx_enu_n',
    'wing_lift_right_fy_enu_n', 'wing_lift_right_fz_enu_n',
    'wing_lift_total_fx_enu_n', 'wing_lift_total_fy_enu_n',
    'wing_lift_total_fz_enu_n', 'wing_lift_gt_vertical_enu_n',
    'wing_lift_gt_age_s',
    'elevator_aero_wrench_gt_valid',
    'elevator_aero_fx_enu_n', 'elevator_aero_fy_enu_n',
    'elevator_aero_fz_enu_n',
    'elevator_aero_mx_enu_nm', 'elevator_aero_my_enu_nm',
    'elevator_aero_mz_enu_nm',
    'elevator_pitch_moment_gt_nm',
    'elevator_aero_wrench_gt_age_s',
    'roll_rad', 'pitch_rad', 'yaw_rad',
    'roll_setpoint_rad', 'pitch_setpoint_rad', 'yaw_setpoint_rad',
    'p_rad_s', 'q_rad_s', 'r_rad_s',
    'alpha_ground_est_raw_rad', 'alpha_est_rad', 'alpha_valid',
    'gamma_est_rad',
    'eta_l_valid', 'eta_l_available', 'eta_l_force_model_valid',
    'eta_l_force_model_invalid_reason',
    'eta_l_min_body_forward_airspeed_mps_config',
    'eta_l_max_abs_beta_rad_config', 'eta_l_max_abs_alpha_rad_config',
    'eta_l_mass_source', 'eta_l_estimated_mass_kg',
    'eta_l_desired_vertical_acceleration_mps2',
    'eta_l_dynamic_pressure_pa', 'eta_l_cl_est',
    'wing_lift_est_n', 'wing_vertical_support_est_n',
    'wing_required_support_est_n', 'eta_l', 'eta_l_gt',
    'eta_c_valid', 'eta_c_mapping_source',
    'eta_c_dynamic_pressure_pa',
    'eta_c_pressure_gate_lower_pa_config',
    'eta_c_pressure_gate_upper_pa_config',
    'qbar_selected_pa', 'cm_delta_e_est',
    'elevator_dmoment_ddelta_per_q_m3_config',
    'shadow_fw_pitch_demand_normalized',
    'shadow_fw_pitch_moment_req_nm',
    'shadow_fw_pitch_moment_req_raw_nm',
    'shadow_fw_pitch_moment_req_for_eta_c_nm',
    'shadow_target_elevator_angle_rad',
    'elevator_actual_angle_rad',
    'elevator_actual_angle_gt_rad', 'elevator_actual_angle_gt_valid',
    'elevator_actual_angle_gt_age_s',
    'elevator_effective_angle_gt_rad',
    'elevator_effective_angle_gt_valid',
    'elevator_effective_angle_gt_age_s',
    'elevator_control_angle_for_eta_c_rad',
    'elevator_id_dither_scale_command',
    'elevator_min_rad_config', 'elevator_max_rad_config',
    'elevator_moment_derivative_nm_per_rad',
    'shadow_target_pitch_moment_nm', 'current_elevator_pitch_moment_nm',
    'shadow_pitch_moment_increment_nm',
    'elevator_remaining_directional_rad',
    'eta_c_remaining_elevator_angle_rad',
    'moment_available_est_nm',
    'moment_available_gt_nm',
    'eta_c_available_increment_moment_nm', 'eta_c_raw',
    'eta_c_q_gate', 'eta_c_pressure_gate',
    'eta_c_gt',
    'eta_c_moment_margin', 'eta_c', 'eta_c_modifies_px4_blending',
    'physics_prior_active',
    'physics_prior_eta_l_lower_config',
    'physics_prior_eta_l_upper_config',
    'physics_prior_eta_c_lower_config',
    'physics_prior_eta_c_upper_config',
    'physics_prior_release_lambda_config',
    'physics_prior_release_dwell_s_config',
    'physics_prior_g_l',
    'physics_prior_g_c',
    'lambda_phy',
    'lambda_max_hard',
    'lambda_projected',
    'lambda_shield_projection_intervention',
    'lambda_shield_total_intervention',
    'lambda_shield_emergency_recovery',
    'lambda_command', 'lambda_exec',
    'lambda_external_active', 'lambda_target_config',
    'lambda_target_tolerance', 'lambda_characterization_complete',
    'pusher_throttle_command', 'pusher_throttle_status',
    'pusher_throttle_external_active',
    'airspeed_filtered_mps', 'pusher_airspeed_error_filtered_mps',
    'pusher_pi_integral_mps_s', 'pusher_throttle_unsaturated',
    'servo_0', 'servo_1', 'servo_2',
    'elevator_angle_proxy_rad', 'elevator_joint_limit_commanded',
    'motor_control_0', 'motor_control_1', 'motor_control_2',
    'motor_control_3', 'motor_control_4',
    'pusher_thrust_setpoint', 'lift_collective_thrust_setpoint',
    'omega_0_rad_s', 'omega_1_rad_s', 'omega_2_rad_s',
    'omega_3_rad_s', 'omega_4_rad_s', 'lift_thrust_n',
    'pusher_thrust_n', 'lift_power_w', 'pusher_power_w',
    'battery_voltage_v', 'battery_current_a', 'battery_power_w',
    'position_age_s', 'attitude_age_s', 'attitude_setpoint_age_s',
    'angular_velocity_age_s',
    'airspeed_age_s', 'vehicle_status_age_s', 'vtol_status_age_s',
    'esc_age_s', 'servos_age_s', 'lambda_status_age_s',
    'lambda_active_age_s', 'pusher_throttle_status_age_s',
    'pusher_throttle_active_age_s', 'actuator_motors_age_s',
    'mc_pitch_weight_age_s', 'fw_pitch_weight_age_s',
    'mc_torque_setpoint_age_s', 'fw_torque_setpoint_age_s',
    'thrust_setpoint_age_s', 'trajectory_setpoint_age_s',
    'wing_lift_left_gt_age_s', 'wing_lift_right_gt_age_s',
)


def _finite(value: Any, default: float = math.nan) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _parse_state_metadata(message: str) -> dict[str, float]:
    metadata: dict[str, float] = {}
    for token in message.split('|')[1:]:
        if '=' not in token:
            continue
        key, value = token.split('=', 1)
        parsed = _finite(value)
        if math.isfinite(parsed):
            metadata[key] = parsed
    return metadata


def _parse_state_error(message: str) -> str:
    for token in message.split('|')[1:]:
        if '=' not in token:
            continue
        key, value = token.split('=', 1)
        if key == 'error':
            return value.strip()
    return ''


def _json_safe(value: object) -> object:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value


def _euler_and_body_velocity(
    attitude: Optional[VehicleAttitude],
    position: Optional[VehicleLocalPosition],
    *,
    wind_east_mps: float = 0.0,
    wind_north_mps: float = 0.0,
    wind_up_mps: float = 0.0,
) -> tuple[float, float, float, float, float, float, float, float, float]:
    if attitude is None or position is None:
        return (math.nan,) * 9
    w, x, y, z = (_finite(item) for item in attitude.q)
    if not all(math.isfinite(item) for item in (w, x, y, z)):
        return (math.nan,) * 9

    roll = math.atan2(2.0 * (w * x + y * z),
                      1.0 - 2.0 * (x * x + y * y))
    pitch_term = min(max(2.0 * (w * y - z * x), -1.0), 1.0)
    pitch = math.asin(pitch_term)
    yaw = math.atan2(2.0 * (w * z + x * y),
                     1.0 - 2.0 * (y * y + z * z))

    vn, ve, vd = (_finite(position.vx), _finite(position.vy),
                  _finite(position.vz))
    if not all(math.isfinite(item) for item in (vn, ve, vd)):
        return (
            roll, pitch, yaw, math.nan, math.nan,
            math.nan, math.nan, math.nan, math.nan,
        )
    if all(math.isfinite(item) for item in (
        wind_east_mps, wind_north_mps, wind_up_mps,
    )):
        vn = vn - wind_north_mps
        ve = ve - wind_east_mps
        vd = vd + wind_up_mps

    r00 = 1.0 - 2.0 * (y * y + z * z)
    r01 = 2.0 * (x * y - w * z)
    r02 = 2.0 * (x * z + w * y)
    r10 = 2.0 * (x * y + w * z)
    r11 = 1.0 - 2.0 * (x * x + z * z)
    r12 = 2.0 * (y * z - w * x)
    r20 = 2.0 * (x * z - w * y)
    r21 = 2.0 * (y * z + w * x)
    r22 = 1.0 - 2.0 * (x * x + y * y)
    body_u = r00 * vn + r10 * ve + r20 * vd
    body_v = r01 * vn + r11 * ve + r21 * vd
    body_w = r02 * vn + r12 * ve + r22 * vd
    alpha = math.atan2(body_w, body_u) if abs(body_u) + abs(body_w) > 1e-6 else math.nan
    body_speed = math.sqrt(body_u * body_u + body_v * body_v + body_w * body_w)
    beta = (
        math.asin(min(max(body_v / body_speed, -1.0), 1.0))
        if body_speed > 1.0e-6 else math.nan
    )
    horizontal_speed = math.hypot(vn, ve)
    gamma = math.atan2(-vd, horizontal_speed) if horizontal_speed > 1e-6 else math.nan
    return roll, pitch, yaw, alpha, gamma, body_u, body_v, body_w, beta


class TransitionExperiment(Node):
    """Collect one deterministic transition run at a fixed sample rate."""

    def __init__(self) -> None:
        super().__init__('ca_lsc_transition_experiment')
        self.declare_parameter('output_csv', '/tmp/ca_lsc_transition.csv')
        self.declare_parameter('condition_id', '')
        self.declare_parameter('wind_model', 'none')
        self.declare_parameter('sim_speed_factor', 1.0)
        self.declare_parameter('wind_cmd_e_enu_mps', 0.0)
        self.declare_parameter('wind_cmd_n_enu_mps', 0.0)
        self.declare_parameter('wind_cmd_u_enu_mps', 0.0)
        self.declare_parameter('gust_amplitude_mps', 0.0)
        self.declare_parameter('gust_delay_s', 0.5)
        self.declare_parameter('gust_duration_s', 2.0)
        self.declare_parameter('gust_recovery_s', 2.0)
        self.declare_parameter('gust_status_timeout_s', 0.25)
        self.declare_parameter('airspeed_source', 'native')
        self.declare_parameter('mass_scale', 1.0)
        self.declare_parameter('nominal_model_mass_kg', math.nan)
        self.declare_parameter('actual_model_mass_kg', math.nan)
        self.declare_parameter('payload_mass_kg', 0.0)
        self.declare_parameter('wing_force_gt_enabled', False)
        self.declare_parameter('wing_force_gt_timeout_s', 0.25)
        self.declare_parameter('eta_l_estimated_mass_kg', math.nan)
        self.declare_parameter(
            'eta_l_desired_vertical_acceleration_mps2', 0.0
        )
        self.declare_parameter('eta_l_air_density_kg_m3', 1.2041)
        self.declare_parameter('eta_l_wing_area_m2', 1.0)
        self.declare_parameter('eta_l_alpha_offset_rad', 0.05984281113)
        self.declare_parameter('eta_l_lift_slope_per_rad', 4.752798721)
        self.declare_parameter('eta_l_stall_angle_rad', 0.3391428111)
        self.declare_parameter('eta_l_post_stall_slope_per_rad', -3.85)
        self.declare_parameter('eta_l_minimum_valid_airspeed_mps', 5.0)
        self.declare_parameter('eta_l_min_body_forward_airspeed_mps', 1.0)
        self.declare_parameter('eta_l_max_abs_beta_rad', 0.35)
        self.declare_parameter('eta_l_max_abs_alpha_rad', 0.60)
        self.declare_parameter('gravity_mps2', 9.80665)
        self.declare_parameter(
            'elevator_dmoment_ddelta_per_q_m3', -0.06
        )
        self.declare_parameter('eta_c_mean_aerodynamic_chord_m', math.nan)
        self.declare_parameter('shadow_command_radians_per_normalized', 1.0)
        self.declare_parameter('eta_c_pressure_gate_lower_pa', 15.05125)
        self.declare_parameter('eta_c_pressure_gate_upper_pa', 60.205)
        self.declare_parameter('f3b2_test_mode', '')
        self.declare_parameter('f3b2_shadow_demand_offset_nm', 0.0)
        self.declare_parameter('f3b2_shadow_demand_for_eta_c_nm', math.nan)
        self.declare_parameter('f3b2_elevator_bias_rad', 0.0)
        self.declare_parameter('f3b2_stage_index', -1.0)
        self.declare_parameter('f3b2_gt_dmoment_ddelta_per_q_m3', math.nan)
        self.declare_parameter('elevator_id_dither_enabled', False)
        self.declare_parameter('target_altitude_m', 50.0)
        self.declare_parameter('sample_rate_hz', 20.0)
        self.declare_parameter('schedule_mode', 'none')
        self.declare_parameter('a0_rl_command_timeout_s', 0.35)
        self.declare_parameter('a0_rl_handover_lambda', 0.95)
        self.declare_parameter('a0_rl_handover_dwell_s', 1.0)
        self.declare_parameter('lambda_start_airspeed_mps', 8.0)
        self.declare_parameter('lambda_full_airspeed_mps', 16.0)
        self.declare_parameter('lambda_maximum', 0.6)
        self.declare_parameter('lambda_target', 0.5)
        self.declare_parameter('lambda_target_tolerance', 0.03)
        self.declare_parameter('lambda_target_dwell_s', 1.0)
        self.declare_parameter('va_target_mps', 12.0)
        self.declare_parameter('va_target_tolerance_mps', 0.3)
        self.declare_parameter('va_target_dwell_s', 1.0)
        self.declare_parameter('va_measurement_s', 3.0)
        self.declare_parameter('va_measurement_min_band_fraction', 0.9)
        self.declare_parameter(
            'va_measurement_max_abs_airspeed_error_mps', 0.6
        )
        self.declare_parameter('va_velocity_kp', 0.8)
        self.declare_parameter('va_velocity_min_mps', 8.0)
        self.declare_parameter('va_velocity_max_mps', 15.0)
        self.declare_parameter('va_altitude_kp', 0.35)
        self.declare_parameter('va_altitude_vertical_speed_kd', 0.5)
        self.declare_parameter('va_altitude_velocity_limit_mps', 1.5)
        self.declare_parameter('vt_unload_altitude_pitch_kp', 2.0)
        self.declare_parameter('vt_unload_vertical_speed_pitch_kd', 2.0)
        self.declare_parameter('vt_unload_pitch_min_deg', -12.0)
        self.declare_parameter('vt_unload_pitch_max_deg', 10.0)
        self.declare_parameter('vt_airspeed_blend_mps', 8.0)
        self.declare_parameter('vt_transition_airspeed_mps', 13.0)
        self.declare_parameter('lambda_attitude_blend_start', 0.1)
        self.declare_parameter('lambda_attitude_blend_full', 0.9)
        self.declare_parameter('physics_prior_eta_l_lower', 0.15)
        self.declare_parameter('physics_prior_eta_l_upper', 0.85)
        self.declare_parameter('physics_prior_eta_c_lower', 0.20)
        self.declare_parameter('physics_prior_eta_c_upper', 0.80)
        self.declare_parameter('physics_prior_hard_eta_l_lower', 0.05)
        self.declare_parameter('physics_prior_hard_eta_l_upper', 0.35)
        self.declare_parameter('physics_prior_hard_eta_c_lower', 0.05)
        self.declare_parameter('physics_prior_hard_eta_c_upper', 0.35)
        self.declare_parameter('physics_prior_altitude_drop_soft_m', 1.0)
        self.declare_parameter('physics_prior_altitude_drop_hard_m', 4.0)
        self.declare_parameter('physics_prior_descent_soft_mps', 0.5)
        self.declare_parameter('physics_prior_descent_hard_mps', 1.5)
        self.declare_parameter('physics_prior_alpha_soft_rad', 0.28)
        self.declare_parameter('physics_prior_alpha_hard_rad', 0.42)
        self.declare_parameter('physics_prior_unloading_rate_per_s', 0.25)
        self.declare_parameter('physics_prior_recovery_rate_per_s', 1.00)
        self.declare_parameter(
            'physics_prior_emergency_recovery_rate_per_s', 2.50
        )
        self.declare_parameter('physics_prior_release_lambda', 0.95)
        self.declare_parameter('physics_prior_release_dwell_s', 2.0)
        self.declare_parameter('pusher_airspeed_ff', 0.25)
        self.declare_parameter('pusher_airspeed_kp', 0.08)
        self.declare_parameter('pusher_airspeed_ki', 0.03)
        self.declare_parameter('pusher_throttle_min', 0.0)
        self.declare_parameter('pusher_throttle_max', 0.45)
        self.declare_parameter('pusher_handover_below_target_mps', 1.0)
        self.declare_parameter('pusher_throttle_slew_up_per_s', 0.5)
        self.declare_parameter('pusher_throttle_slew_down_per_s', 1.5)
        self.declare_parameter('va_filter_alpha', 0.35)
        self.declare_parameter('va_runaway_margin_mps', 3.0)
        self.declare_parameter('va_runaway_dwell_s', 0.5)
        self.declare_parameter('va_altitude_abort_error_m', 5.0)
        self.declare_parameter('va_vertical_speed_abort_mps', 2.0)
        self.declare_parameter('va_vertical_abort_min_airspeed_mps', 6.0)
        self.declare_parameter('transition_stability_dwell_s', 2.0)
        self.declare_parameter('transition_altitude_tolerance_m', 1.0)
        self.declare_parameter('transition_vertical_speed_tolerance_mps', 0.2)
        self.declare_parameter('transition_groundspeed_tolerance_mps', 0.2)
        self.declare_parameter('alpha_min_airspeed_mps', 5.0)
        self.declare_parameter('exit_on_terminal', True)
        self.declare_parameter('stop_after_fw_hold_s', 6.0)
        self.declare_parameter('lift_motor_constant', 2.0e-5)
        self.declare_parameter('lift_moment_constant', 0.06)
        self.declare_parameter('pusher_motor_constant', 8.54858e-6)
        self.declare_parameter('pusher_moment_constant', 0.01)
        # GZ standard_vtol servo_2 joint limits.  ActuatorServos is passed by
        # the PX4 GZ bridge as a radian command, so values outside this range
        # are clipped by the simulator joint even though the PX4 channel is
        # still below its normalized +/-1 limit.
        self.declare_parameter('elevator_joint_min_rad', -0.53)
        self.declare_parameter('elevator_joint_max_rad', 0.53)

        output = Path(str(self.get_parameter('output_csv').value)).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        self._stream = output.open('w', newline='', encoding='utf-8')
        self._writer = csv.DictWriter(self._stream, fieldnames=FIELDS)
        self._writer.writeheader()
        self._rows = 0

        self._target_altitude = float(self.get_parameter('target_altitude_m').value)
        self._condition_id = str(self.get_parameter('condition_id').value)
        self._wind_model = str(self.get_parameter('wind_model').value)
        self._sim_speed_factor = float(
            self.get_parameter('sim_speed_factor').value
        )
        self._wind_cmd_e = float(
            self.get_parameter('wind_cmd_e_enu_mps').value
        )
        self._wind_cmd_n = float(
            self.get_parameter('wind_cmd_n_enu_mps').value
        )
        self._wind_cmd_u = float(
            self.get_parameter('wind_cmd_u_enu_mps').value
        )
        self._gust_amplitude = float(
            self.get_parameter('gust_amplitude_mps').value
        )
        self._gust_delay_s = float(self.get_parameter('gust_delay_s').value)
        self._gust_duration_s = float(
            self.get_parameter('gust_duration_s').value
        )
        self._gust_recovery_s = float(
            self.get_parameter('gust_recovery_s').value
        )
        self._gust_status_timeout_s = float(
            self.get_parameter('gust_status_timeout_s').value
        )
        self._airspeed_source = str(self.get_parameter('airspeed_source').value)
        if self._airspeed_source not in {'native', 'relative_wind'}:
            raise ValueError('airspeed_source must be native or relative_wind')
        self._mass_scale = float(self.get_parameter('mass_scale').value)
        self._nominal_model_mass_kg = float(
            self.get_parameter('nominal_model_mass_kg').value
        )
        self._actual_model_mass_kg = float(
            self.get_parameter('actual_model_mass_kg').value
        )
        self._payload_mass_kg = float(
            self.get_parameter('payload_mass_kg').value
        )
        self._wing_force_gt_enabled = bool(
            self.get_parameter('wing_force_gt_enabled').value
        )
        self._wing_force_gt_timeout_s = float(
            self.get_parameter('wing_force_gt_timeout_s').value
        )
        configured_eta_l_mass = float(
            self.get_parameter('eta_l_estimated_mass_kg').value
        )
        if math.isfinite(configured_eta_l_mass) and configured_eta_l_mass > 0.0:
            self._eta_l_estimated_mass_kg = configured_eta_l_mass
            self._eta_l_mass_source = 'configured_estimate'
        elif math.isfinite(self._actual_model_mass_kg) and self._actual_model_mass_kg > 0.0:
            self._eta_l_estimated_mass_kg = self._actual_model_mass_kg
            self._eta_l_mass_source = 'actual_model_mass_for_validation'
        elif math.isfinite(self._nominal_model_mass_kg) and self._nominal_model_mass_kg > 0.0:
            self._eta_l_estimated_mass_kg = self._nominal_model_mass_kg
            self._eta_l_mass_source = 'nominal_model_mass_fallback'
        else:
            self._eta_l_estimated_mass_kg = math.nan
            self._eta_l_mass_source = 'unavailable'
        self._eta_l_desired_vertical_acceleration = float(
            self.get_parameter(
                'eta_l_desired_vertical_acceleration_mps2'
            ).value
        )
        self._eta_l_density = float(
            self.get_parameter('eta_l_air_density_kg_m3').value
        )
        self._eta_l_wing_area = float(
            self.get_parameter('eta_l_wing_area_m2').value
        )
        self._eta_l_alpha_offset = float(
            self.get_parameter('eta_l_alpha_offset_rad').value
        )
        self._eta_l_lift_slope = float(
            self.get_parameter('eta_l_lift_slope_per_rad').value
        )
        self._eta_l_stall_angle = float(
            self.get_parameter('eta_l_stall_angle_rad').value
        )
        self._eta_l_post_stall_slope = float(
            self.get_parameter('eta_l_post_stall_slope_per_rad').value
        )
        self._eta_l_min_airspeed = float(
            self.get_parameter('eta_l_minimum_valid_airspeed_mps').value
        )
        self._eta_l_min_body_forward_airspeed = float(
            self.get_parameter('eta_l_min_body_forward_airspeed_mps').value
        )
        self._eta_l_max_abs_beta = float(
            self.get_parameter('eta_l_max_abs_beta_rad').value
        )
        self._eta_l_max_abs_alpha = float(
            self.get_parameter('eta_l_max_abs_alpha_rad').value
        )
        self._gravity = float(self.get_parameter('gravity_mps2').value)
        self._elevator_dmoment_ddelta_per_q = float(
            self.get_parameter('elevator_dmoment_ddelta_per_q_m3').value
        )
        self._eta_c_mean_chord = float(
            self.get_parameter('eta_c_mean_aerodynamic_chord_m').value
        )
        self._shadow_command_radians_per_normalized = float(
            self.get_parameter('shadow_command_radians_per_normalized').value
        )
        self._eta_c_pressure_gate_lower = float(
            self.get_parameter('eta_c_pressure_gate_lower_pa').value
        )
        self._eta_c_pressure_gate_upper = float(
            self.get_parameter('eta_c_pressure_gate_upper_pa').value
        )
        self._f3b2_test_mode = str(
            self.get_parameter('f3b2_test_mode').value
        ).strip()
        self._f3b2_shadow_demand_offset = float(
            self.get_parameter('f3b2_shadow_demand_offset_nm').value
        )
        self._f3b2_shadow_demand_for_eta_c = float(
            self.get_parameter('f3b2_shadow_demand_for_eta_c_nm').value
        )
        self._f3b2_elevator_bias = float(
            self.get_parameter('f3b2_elevator_bias_rad').value
        )
        self._f3b2_stage_index = float(
            self.get_parameter('f3b2_stage_index').value
        )
        self._f3b2_gt_dmoment_ddelta_per_q = float(
            self.get_parameter('f3b2_gt_dmoment_ddelta_per_q_m3').value
        )
        self._elevator_id_dither_enabled = bool(
            self.get_parameter('elevator_id_dither_enabled').value
        )
        capability_parameters = (
            self._wing_force_gt_timeout_s,
            self._eta_l_desired_vertical_acceleration,
            self._eta_l_density,
            self._eta_l_wing_area,
            self._eta_l_alpha_offset,
            self._eta_l_lift_slope,
            self._eta_l_stall_angle,
            self._eta_l_post_stall_slope,
            self._eta_l_min_airspeed,
            self._eta_l_min_body_forward_airspeed,
            self._eta_l_max_abs_beta,
            self._eta_l_max_abs_alpha,
            self._gravity,
            self._elevator_dmoment_ddelta_per_q,
            self._eta_c_pressure_gate_lower,
            self._eta_c_pressure_gate_upper,
        )
        if not all(math.isfinite(value) for value in capability_parameters):
            raise ValueError('F3 capability parameters must be finite')
        if not math.isfinite(self._shadow_command_radians_per_normalized):
            raise ValueError(
                'shadow_command_radians_per_normalized must be finite'
            )
        if (
            self._wing_force_gt_timeout_s <= 0.0
            or self._eta_l_density <= 0.0
            or self._eta_l_wing_area <= 0.0
            or self._eta_l_stall_angle <= 0.0
            or self._eta_l_min_airspeed <= 0.0
            or self._eta_l_min_body_forward_airspeed < 0.0
            or self._eta_l_max_abs_beta <= 0.0
            or self._eta_l_max_abs_alpha <= 0.0
            or self._gravity <= 0.0
            or self._shadow_command_radians_per_normalized <= 0.0
            or self._eta_c_pressure_gate_lower < 0.0
            or self._eta_c_pressure_gate_upper
            <= self._eta_c_pressure_gate_lower
        ):
            raise ValueError('F3 capability parameter bounds are invalid')
        if self._f3b2_test_mode not in {
            '', 'q_gate', 'demand', 'remaining_travel',
        }:
            raise ValueError('unsupported f3b2_test_mode')
        if not all(math.isfinite(value) for value in (
            self._f3b2_shadow_demand_offset,
            self._f3b2_elevator_bias,
            self._f3b2_stage_index,
        )):
            raise ValueError('F3-B2 diagnostic parameters must be finite')
        if self._f3b2_elevator_bias < 0.0:
            raise ValueError('f3b2_elevator_bias_rad must be non-negative')
        self._schedule_mode = str(self.get_parameter('schedule_mode').value)
        if self._schedule_mode not in {
            'none', 'manual_airspeed', 'manual_target', 'va_hold_target',
            'gust_target', 'physics_prior_only', 'a0_rl_external',
        }:
            raise ValueError(
                'schedule_mode must be none, manual_airspeed, manual_target, '
                'va_hold_target, gust_target, physics_prior_only, or '
                'a0_rl_external'
            )
        self._a0_rl_command_timeout_s = float(
            self.get_parameter('a0_rl_command_timeout_s').value
        )
        self._a0_rl_handover_lambda = float(
            self.get_parameter('a0_rl_handover_lambda').value
        )
        self._a0_rl_handover_dwell_s = float(
            self.get_parameter('a0_rl_handover_dwell_s').value
        )
        self._lambda_start = float(self.get_parameter('lambda_start_airspeed_mps').value)
        self._lambda_full = float(self.get_parameter('lambda_full_airspeed_mps').value)
        self._lambda_max = float(self.get_parameter('lambda_maximum').value)
        self._lambda_target = float(self.get_parameter('lambda_target').value)
        self._lambda_target_tolerance = float(
            self.get_parameter('lambda_target_tolerance').value
        )
        self._lambda_target_dwell_s = float(
            self.get_parameter('lambda_target_dwell_s').value
        )
        self._va_target = float(self.get_parameter('va_target_mps').value)
        self._va_target_tolerance = float(
            self.get_parameter('va_target_tolerance_mps').value
        )
        self._va_target_dwell_s = float(
            self.get_parameter('va_target_dwell_s').value
        )
        self._va_measurement_s = float(
            self.get_parameter('va_measurement_s').value
        )
        self._va_measurement_min_band_fraction = float(
            self.get_parameter('va_measurement_min_band_fraction').value
        )
        self._va_measurement_max_abs_airspeed_error = float(
            self.get_parameter(
                'va_measurement_max_abs_airspeed_error_mps'
            ).value
        )
        self._va_velocity_kp = float(self.get_parameter('va_velocity_kp').value)
        self._va_velocity_min = float(
            self.get_parameter('va_velocity_min_mps').value
        )
        self._va_velocity_max = float(
            self.get_parameter('va_velocity_max_mps').value
        )
        self._va_altitude_kp = float(
            self.get_parameter('va_altitude_kp').value
        )
        self._va_altitude_vertical_speed_kd = float(
            self.get_parameter('va_altitude_vertical_speed_kd').value
        )
        self._va_altitude_velocity_limit = float(
            self.get_parameter('va_altitude_velocity_limit_mps').value
        )
        self._vt_unload_altitude_pitch_kp = float(
            self.get_parameter('vt_unload_altitude_pitch_kp').value
        )
        self._vt_unload_vertical_speed_pitch_kd = float(
            self.get_parameter('vt_unload_vertical_speed_pitch_kd').value
        )
        self._vt_unload_pitch_min_deg = float(
            self.get_parameter('vt_unload_pitch_min_deg').value
        )
        self._vt_unload_pitch_max_deg = float(
            self.get_parameter('vt_unload_pitch_max_deg').value
        )
        self._vt_airspeed_blend = float(
            self.get_parameter('vt_airspeed_blend_mps').value
        )
        self._vt_transition_airspeed = float(
            self.get_parameter('vt_transition_airspeed_mps').value
        )
        self._lambda_attitude_blend_start = float(
            self.get_parameter('lambda_attitude_blend_start').value
        )
        self._lambda_attitude_blend_full = float(
            self.get_parameter('lambda_attitude_blend_full').value
        )
        self._physics_prior_eta_l_lower = float(
            self.get_parameter('physics_prior_eta_l_lower').value
        )
        self._physics_prior_eta_l_upper = float(
            self.get_parameter('physics_prior_eta_l_upper').value
        )
        self._physics_prior_eta_c_lower = float(
            self.get_parameter('physics_prior_eta_c_lower').value
        )
        self._physics_prior_eta_c_upper = float(
            self.get_parameter('physics_prior_eta_c_upper').value
        )
        self._physics_prior_hard_eta_l_lower = float(
            self.get_parameter('physics_prior_hard_eta_l_lower').value
        )
        self._physics_prior_hard_eta_l_upper = float(
            self.get_parameter('physics_prior_hard_eta_l_upper').value
        )
        self._physics_prior_hard_eta_c_lower = float(
            self.get_parameter('physics_prior_hard_eta_c_lower').value
        )
        self._physics_prior_hard_eta_c_upper = float(
            self.get_parameter('physics_prior_hard_eta_c_upper').value
        )
        self._physics_prior_altitude_drop_soft = float(
            self.get_parameter('physics_prior_altitude_drop_soft_m').value
        )
        self._physics_prior_altitude_drop_hard = float(
            self.get_parameter('physics_prior_altitude_drop_hard_m').value
        )
        self._physics_prior_descent_soft = float(
            self.get_parameter('physics_prior_descent_soft_mps').value
        )
        self._physics_prior_descent_hard = float(
            self.get_parameter('physics_prior_descent_hard_mps').value
        )
        self._physics_prior_alpha_soft = float(
            self.get_parameter('physics_prior_alpha_soft_rad').value
        )
        self._physics_prior_alpha_hard = float(
            self.get_parameter('physics_prior_alpha_hard_rad').value
        )
        self._physics_prior_rate_limiter = AsymmetricRateLimiter(
            unloading_rate_per_s=float(
                self.get_parameter(
                    'physics_prior_unloading_rate_per_s'
                ).value
            ),
            recovery_rate_per_s=float(
                self.get_parameter(
                    'physics_prior_recovery_rate_per_s'
                ).value
            ),
            emergency_recovery_rate_per_s=float(
                self.get_parameter(
                    'physics_prior_emergency_recovery_rate_per_s'
                ).value
            ),
        )
        self._physics_prior_release_lambda = float(
            self.get_parameter('physics_prior_release_lambda').value
        )
        self._physics_prior_release_dwell_s = float(
            self.get_parameter('physics_prior_release_dwell_s').value
        )
        self._pusher_airspeed_ff = float(
            self.get_parameter('pusher_airspeed_ff').value
        )
        self._pusher_airspeed_kp = float(
            self.get_parameter('pusher_airspeed_kp').value
        )
        self._pusher_airspeed_ki = float(
            self.get_parameter('pusher_airspeed_ki').value
        )
        self._pusher_throttle_min = float(
            self.get_parameter('pusher_throttle_min').value
        )
        self._pusher_throttle_max = float(
            self.get_parameter('pusher_throttle_max').value
        )
        self._pusher_handover_below_target = float(
            self.get_parameter('pusher_handover_below_target_mps').value
        )
        self._pusher_throttle_slew_up = float(
            self.get_parameter('pusher_throttle_slew_up_per_s').value
        )
        self._pusher_throttle_slew_down = float(
            self.get_parameter('pusher_throttle_slew_down_per_s').value
        )
        self._va_filter_alpha = float(self.get_parameter('va_filter_alpha').value)
        self._va_runaway_margin = float(
            self.get_parameter('va_runaway_margin_mps').value
        )
        self._va_runaway_dwell_s = float(
            self.get_parameter('va_runaway_dwell_s').value
        )
        self._va_altitude_abort_error = float(
            self.get_parameter('va_altitude_abort_error_m').value
        )
        self._va_vertical_speed_abort = float(
            self.get_parameter('va_vertical_speed_abort_mps').value
        )
        self._va_vertical_abort_min_airspeed = float(
            self.get_parameter('va_vertical_abort_min_airspeed_mps').value
        )
        self._transition_stability_dwell_s = float(
            self.get_parameter('transition_stability_dwell_s').value
        )
        self._transition_altitude_tolerance_m = float(
            self.get_parameter('transition_altitude_tolerance_m').value
        )
        self._transition_vertical_speed_tolerance_mps = float(
            self.get_parameter('transition_vertical_speed_tolerance_mps').value
        )
        self._transition_groundspeed_tolerance_mps = float(
            self.get_parameter('transition_groundspeed_tolerance_mps').value
        )
        self._alpha_min_airspeed = float(
            self.get_parameter('alpha_min_airspeed_mps').value
        )
        if not all(math.isfinite(value) for value in (
            self._wind_cmd_e, self._wind_cmd_n, self._wind_cmd_u,
        )):
            raise ValueError('configured wind vector must be finite')
        if (
            not math.isfinite(self._gust_amplitude)
            or self._gust_amplitude < 0.0
            or not math.isfinite(self._gust_delay_s)
            or self._gust_delay_s < 0.0
            or not math.isfinite(self._gust_duration_s)
            or self._gust_duration_s <= 0.0
            or not math.isfinite(self._gust_recovery_s)
            or self._gust_recovery_s <= 0.0
            or not math.isfinite(self._gust_status_timeout_s)
            or self._gust_status_timeout_s <= 0.0
        ):
            raise ValueError('gust timing/amplitude parameters are invalid')
        if (
            not math.isfinite(self._a0_rl_command_timeout_s)
            or self._a0_rl_command_timeout_s <= 0.0
            or not 0.0 <= self._a0_rl_handover_lambda <= 1.0
            or not math.isfinite(self._a0_rl_handover_dwell_s)
            or self._a0_rl_handover_dwell_s <= 0.0
        ):
            raise ValueError('A0 RL external timing parameters are invalid')
        if not 0.0 <= self._lambda_target <= 1.0:
            raise ValueError('lambda_target must lie in [0, 1]')
        if not 0.0 < self._lambda_target_tolerance <= 1.0:
            raise ValueError('lambda_target_tolerance must lie in (0, 1]')
        if self._lambda_target_dwell_s <= 0.0:
            raise ValueError('lambda_target_dwell_s must be positive')
        if not math.isfinite(self._va_target) or self._va_target <= 0.0:
            raise ValueError('va_target_mps must be positive')
        if (
            not math.isfinite(self._va_target_tolerance)
            or self._va_target_tolerance <= 0.0
        ):
            raise ValueError('va_target_tolerance_mps must be positive')
        if self._va_target_dwell_s <= 0.0:
            raise ValueError('va_target_dwell_s must be positive')
        if self._va_measurement_s <= 0.0:
            raise ValueError('va_measurement_s must be positive')
        if not 0.0 < self._va_measurement_min_band_fraction <= 1.0:
            raise ValueError(
                'va_measurement_min_band_fraction must lie in (0, 1]'
            )
        if (
            not math.isfinite(self._va_measurement_max_abs_airspeed_error)
            or self._va_measurement_max_abs_airspeed_error
            < self._va_target_tolerance
        ):
            raise ValueError(
                'va_measurement_max_abs_airspeed_error_mps must be finite '
                'and no smaller than va_target_tolerance_mps'
            )
        if not math.isfinite(self._va_velocity_kp) or self._va_velocity_kp < 0.0:
            raise ValueError('va_velocity_kp must be finite and non-negative')
        if (
            not math.isfinite(self._va_velocity_min)
            or not math.isfinite(self._va_velocity_max)
            or self._va_velocity_min <= 0.0
            or self._va_velocity_max <= self._va_velocity_min
        ):
            raise ValueError('va velocity command bounds are invalid')
        if (
            not math.isfinite(self._va_altitude_kp)
            or not math.isfinite(self._va_altitude_vertical_speed_kd)
            or not math.isfinite(self._va_altitude_velocity_limit)
            or self._va_altitude_kp < 0.0
            or self._va_altitude_vertical_speed_kd < 0.0
            or self._va_altitude_velocity_limit <= 0.0
        ):
            raise ValueError('Va-hold altitude controller parameters are invalid')
        if (
            not math.isfinite(self._vt_unload_altitude_pitch_kp)
            or not math.isfinite(self._vt_unload_vertical_speed_pitch_kd)
            or not math.isfinite(self._vt_unload_pitch_min_deg)
            or not math.isfinite(self._vt_unload_pitch_max_deg)
            or self._vt_unload_altitude_pitch_kp < 0.0
            or self._vt_unload_vertical_speed_pitch_kd < 0.0
            or self._vt_unload_pitch_min_deg >= self._vt_unload_pitch_max_deg
        ):
            raise ValueError('PX4 unloading pitch controller parameters are invalid')
        if (
            not math.isfinite(self._vt_airspeed_blend)
            or not math.isfinite(self._vt_transition_airspeed)
            or self._vt_airspeed_blend < 0.0
            or self._vt_transition_airspeed < self._vt_airspeed_blend
        ):
            raise ValueError('PX4 VTOL airspeed blend parameters are invalid')
        if not (
            0.0 <= self._lambda_attitude_blend_start
            < self._lambda_attitude_blend_full <= 1.0
        ):
            raise ValueError('lambda attitude-allocation bounds are invalid')
        if not (
            0.0 <= self._physics_prior_eta_l_lower
            < self._physics_prior_eta_l_upper <= 1.0
            and 0.0 <= self._physics_prior_eta_c_lower
            < self._physics_prior_eta_c_upper <= 1.0
            and 0.0 <= self._physics_prior_hard_eta_l_lower
            < self._physics_prior_hard_eta_l_upper <= 1.0
            and 0.0 <= self._physics_prior_hard_eta_c_lower
            < self._physics_prior_hard_eta_c_upper <= 1.0
        ):
            raise ValueError('physics-prior capability gates are invalid')
        if not (
            0.0 <= self._physics_prior_altitude_drop_soft
            < self._physics_prior_altitude_drop_hard
            and 0.0 <= self._physics_prior_descent_soft
            < self._physics_prior_descent_hard
            and 0.0 <= self._physics_prior_alpha_soft
            < self._physics_prior_alpha_hard
        ):
            raise ValueError('physics-prior hard-envelope bounds are invalid')
        if not (
            0.0 < self._physics_prior_release_lambda <= 1.0
            and self._physics_prior_release_dwell_s > 0.0
        ):
            raise ValueError('physics-prior release gate is invalid')
        if (
            not math.isfinite(self._pusher_airspeed_ff)
            or not math.isfinite(self._pusher_airspeed_kp)
            or not math.isfinite(self._pusher_airspeed_ki)
            or self._pusher_airspeed_kp < 0.0
            or self._pusher_airspeed_ki < 0.0
        ):
            raise ValueError('pusher airspeed controller gains are invalid')
        if (
            not math.isfinite(self._pusher_throttle_min)
            or not math.isfinite(self._pusher_throttle_max)
            or self._pusher_throttle_min < 0.0
            or self._pusher_throttle_max > 1.0
            or self._pusher_throttle_max <= self._pusher_throttle_min
        ):
            raise ValueError('pusher throttle bounds are invalid')
        if (
            not math.isfinite(self._pusher_handover_below_target)
            or self._pusher_handover_below_target < 0.0
            or self._pusher_throttle_slew_up <= 0.0
            or self._pusher_throttle_slew_down <= 0.0
            or not 0.0 < self._va_filter_alpha <= 1.0
            or self._va_runaway_margin <= 0.0
            or self._va_runaway_dwell_s <= 0.0
            or self._va_altitude_abort_error <= 0.0
            or self._va_vertical_speed_abort <= 0.0
            or self._va_vertical_abort_min_airspeed < 0.0
        ):
            raise ValueError('Va-hold safety/controller parameters are invalid')
        if self._transition_stability_dwell_s <= 0.0:
            raise ValueError('transition_stability_dwell_s must be positive')
        if self._transition_altitude_tolerance_m <= 0.0:
            raise ValueError('transition_altitude_tolerance_m must be positive')
        if self._transition_vertical_speed_tolerance_mps <= 0.0:
            raise ValueError(
                'transition_vertical_speed_tolerance_mps must be positive'
            )
        if self._transition_groundspeed_tolerance_mps <= 0.0:
            raise ValueError(
                'transition_groundspeed_tolerance_mps must be positive'
            )
        if self._alpha_min_airspeed <= 0.0:
            raise ValueError('alpha_min_airspeed_mps must be positive')
        self._exit_on_terminal = bool(self.get_parameter('exit_on_terminal').value)
        self._stop_after_fw_hold_s = float(
            self.get_parameter('stop_after_fw_hold_s').value
        )
        self._lift_kt = float(self.get_parameter('lift_motor_constant').value)
        self._lift_km = float(self.get_parameter('lift_moment_constant').value)
        self._pusher_kt = float(self.get_parameter('pusher_motor_constant').value)
        self._pusher_km = float(self.get_parameter('pusher_moment_constant').value)
        self._elevator_joint_min = float(
            self.get_parameter('elevator_joint_min_rad').value
        )
        self._elevator_joint_max = float(
            self.get_parameter('elevator_joint_max_rad').value
        )
        if (
            not math.isfinite(self._elevator_joint_min)
            or not math.isfinite(self._elevator_joint_max)
            or self._elevator_joint_min >= self._elevator_joint_max
        ):
            raise ValueError('elevator joint limits are invalid')

        self._position: Optional[VehicleLocalPosition] = None
        self._attitude: Optional[VehicleAttitude] = None
        self._attitude_setpoint: Optional[VehicleAttitudeSetpoint] = None
        self._angular_velocity: Optional[VehicleAngularVelocity] = None
        self._airspeed: Optional[AirspeedValidated] = None
        self._status: Optional[VehicleStatus] = None
        self._vtol_status: Optional[VtolVehicleStatus] = None
        self._esc: Optional[EscStatus] = None
        self._servos: Optional[ActuatorServos] = None
        self._actuator_motors: Optional[ActuatorMotors] = None
        self._thrust_setpoint: Optional[VehicleThrustSetpoint] = None
        self._trajectory_setpoint: Optional[TrajectorySetpoint] = None
        self._battery: Optional[BatteryStatus] = None
        self._lambda_status: Optional[NormalizedUnsignedSetpoint] = None
        self._lambda_active_status: Optional[NormalizedUnsignedSetpoint] = None
        self._pusher_throttle_status: Optional[NormalizedUnsignedSetpoint] = None
        self._pusher_throttle_active: Optional[NormalizedUnsignedSetpoint] = None
        self._mc_pitch_weight_status: Optional[NormalizedUnsignedSetpoint] = None
        self._fw_pitch_weight_status: Optional[NormalizedUnsignedSetpoint] = None
        self._mc_torque_setpoint: Optional[VehicleTorqueSetpoint] = None
        self._fw_torque_setpoint: Optional[VehicleTorqueSetpoint] = None
        self._gust_status: Optional[Vector3Stamped] = None
        self._wing_lift_left_gt: Optional[WrenchStamped] = None
        self._wing_lift_right_gt: Optional[WrenchStamped] = None
        self._elevator_aero_wrench_gt: Optional[WrenchStamped] = None
        self._elevator_joint_position_gt: Optional[Float64] = None
        self._elevator_effective_angle_gt: Optional[Float64] = None
        self._elevator_id_dither_scale_command = 0.0
        self._command_state = ''
        self._command_state_metadata: dict[str, float] = {}
        self._command_state_error = ''
        self._home_z: Optional[float] = None
        self._command_altitude_datum_local_m: Optional[float] = None
        self._command_target_altitude_local_m: Optional[float] = None
        # The first local-position sample can arrive while the spawned model
        # is still settling.  The command node captures its takeoff datum when
        # PX4 becomes ready (immediately before PREFLIGHT_STREAM), so lock the
        # recorder to the same event instead of permanently using that first
        # sample.  Otherwise the two nodes can disagree on "50 m" by the
        # initial settling displacement.
        self._home_reference_locked = False
        self._lambda_command = math.nan
        self._lambda_schedule_active = False
        self._lambda_target_dwell_started_at: Optional[float] = None
        self._lambda_characterization_complete = False
        self._physics_prior_g_l = math.nan
        self._physics_prior_g_c = math.nan
        self._lambda_phy = math.nan
        self._lambda_max_hard = math.nan
        self._lambda_projected = math.nan
        self._lambda_shield_projection_intervention = math.nan
        self._lambda_shield_total_intervention = math.nan
        self._lambda_shield_emergency_recovery = False
        self._physics_prior_last_update_at: Optional[float] = None
        self._physics_prior_previous_command = 0.0
        self._va_hold_phase = 'inactive'
        self._va_settle_started_at: Optional[float] = None
        self._va_lambda_settle_started_at: Optional[float] = None
        self._va_measurement_started_at: Optional[float] = None
        self._va_measurement_complete = False
        self._va_measurement_interrupted = False
        self._va_velocity_command = math.nan
        self._va_down_velocity_command = math.nan
        self._pusher_throttle_command = math.nan
        self._pusher_pi_integral = 0.0
        self._pusher_airspeed_error_filtered = math.nan
        self._pusher_throttle_unsaturated = math.nan
        self._pusher_last_update_at: Optional[float] = None
        self._pusher_control_active = False
        self._va_filtered = math.nan
        self._a0_rl_action_seq = -1
        self._a0_rl_action = math.nan
        self._a0_rl_lambda_d = math.nan
        self._a0_rl_action_source = ''
        self._a0_rl_action_received_at: Optional[float] = None
        self._a0_rl_action_sim_time_s: Optional[float] = None
        self._a0_rl_handover_ready_started_at: Optional[float] = None
        self._a0_rl_handover_ready = False
        self._a0_rl_initial_readiness_available = False
        self._a0_rl_initial_readiness_latched = False
        self._a0_rl_initial_height_gate_ok = False
        self._a0_rl_initial_vz_gate_ok = False
        self._a0_rl_initial_groundspeed_gate_ok = False
        self._a0_rl_terminal_reason = ''
        self._va_runaway_started_at: Optional[float] = None
        self._va_altitude_abort_started_at: Optional[float] = None
        self._va_vertical_abort_started_at: Optional[float] = None
        self._va_abort_reason = ''
        self._gust_phase_started_at: Optional[float] = None
        self._gust_phase_elapsed_s = math.nan
        self._gust_profile_scale = 0.0
        self._local_stability_since_s: Optional[float] = None
        self._received_at: dict[str, float] = {}

        self._terminal_wall_time: Optional[float] = None
        self._early_abort_wall_time: Optional[float] = None
        self._fw_hold_wall_time: Optional[float] = None
        self._last_a0_observation_sim_time_s: Optional[float] = None
        self._last_a0_observation_wall_time_s: Optional[float] = None
        self._last_a0_observation: Optional[list[float]] = None

        self._subscribe(VehicleLocalPosition, '/fmu/out/vehicle_local_position', '_position')
        self._subscribe(VehicleAttitude, '/fmu/out/vehicle_attitude', '_attitude')
        self._subscribe(
            VehicleAttitudeSetpoint,
            '/fmu/out/vehicle_attitude_setpoint',
            '_attitude_setpoint',
        )
        self._subscribe(VehicleAngularVelocity, '/fmu/out/vehicle_angular_velocity', '_angular_velocity')
        self._subscribe(AirspeedValidated, '/fmu/out/airspeed_validated', '_airspeed')
        self._subscribe(VehicleStatus, '/fmu/out/vehicle_status', '_status')
        self._subscribe(VtolVehicleStatus, '/fmu/out/vtol_vehicle_status', '_vtol_status')
        self._subscribe(EscStatus, '/fmu/out/esc_status', '_esc')
        self._subscribe(ActuatorServos, '/fmu/out/actuator_servos', '_servos')
        self._subscribe(ActuatorMotors, '/fmu/out/actuator_motors', '_actuator_motors')
        self._subscribe(
            VehicleThrustSetpoint,
            '/fmu/out/vehicle_thrust_setpoint',
            '_thrust_setpoint',
        )
        self._subscribe(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            '_trajectory_setpoint',
        )
        self._subscribe(BatteryStatus, '/fmu/out/battery_status', '_battery')
        self._subscribe(
            NormalizedUnsignedSetpoint,
            '/fmu/out/lift_rotor_unloading_status',
            '_lambda_status',
        )
        self._subscribe(
            NormalizedUnsignedSetpoint,
            '/fmu/out/lift_rotor_unloading_active',
            '_lambda_active_status',
        )
        self._subscribe(
            NormalizedUnsignedSetpoint,
            '/fmu/out/pusher_throttle_status',
            '_pusher_throttle_status',
        )
        self._subscribe(
            NormalizedUnsignedSetpoint,
            '/fmu/out/pusher_throttle_active',
            '_pusher_throttle_active',
        )
        self._subscribe(
            NormalizedUnsignedSetpoint,
            '/fmu/out/vtol_mc_pitch_weight',
            '_mc_pitch_weight_status',
        )
        self._subscribe(
            NormalizedUnsignedSetpoint,
            '/fmu/out/vtol_fw_pitch_weight',
            '_fw_pitch_weight_status',
        )
        self._subscribe(
            VehicleTorqueSetpoint,
            '/fmu/out/vehicle_torque_setpoint_virtual_mc',
            '_mc_torque_setpoint',
        )
        self._subscribe(
            VehicleTorqueSetpoint,
            '/fmu/out/vehicle_torque_setpoint_virtual_fw',
            '_fw_torque_setpoint',
        )
        self._subscribe(
            Vector3Stamped,
            '/ca_lsc/gust_status_enu',
            '_gust_status',
        )
        self._subscribe(
            WrenchStamped,
            '/ca_lsc/wing_lift_left_gt_enu',
            '_wing_lift_left_gt',
        )
        self._subscribe(
            WrenchStamped,
            '/ca_lsc/wing_lift_right_gt_enu',
            '_wing_lift_right_gt',
        )
        self._subscribe(
            WrenchStamped,
            '/ca_lsc/elevator_aero_wrench_gt_enu',
            '_elevator_aero_wrench_gt',
        )
        self._subscribe(
            Float64,
            '/ca_lsc/elevator_joint_position_gt',
            '_elevator_joint_position_gt',
        )
        self._subscribe(
            Float64,
            '/ca_lsc/elevator_effective_angle_gt',
            '_elevator_effective_angle_gt',
        )
        self._state_subscription = self.create_subscription(
            String, '/vtol/command/state', self._state_callback, 10
        )
        self._lambda_publisher = self.create_publisher(
            NormalizedUnsignedSetpoint,
            '/fmu/in/lift_rotor_unloading_setpoint',
            10,
        )
        self._velocity_publisher = self.create_publisher(
            Twist,
            '/vtol/command/velocity_setpoint_ned',
            10,
        )
        self._pusher_throttle_publisher = self.create_publisher(
            NormalizedUnsignedSetpoint,
            '/fmu/in/pusher_throttle_setpoint',
            10,
        )
        self._elevator_id_dither_publisher = self.create_publisher(
            Float64, '/ca_lsc/elevator_id_dither_scale', 10,
        )
        self._gust_publisher = self.create_publisher(
            Vector3Stamped, '/ca_lsc/gust_command_enu', 10
        )
        self._a0_observation_publisher = self.create_publisher(
            String, '/ca_lsc/a0_observation', 10
        )
        self.create_subscription(
            String, '/ca_lsc/a0_rl_action', self._a0_rl_action_callback, 10
        )

        sample_rate = float(self.get_parameter('sample_rate_hz').value)
        if not math.isfinite(sample_rate) or sample_rate <= 0.0:
            raise ValueError('sample_rate_hz must be positive')
        self._sample_period_s = 1.0 / sample_rate
        self._wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self._sample_timer = None
        self._sample_bootstrap_timer = self.create_timer(
            min(self._sample_period_s, 0.02),
            self._bootstrap_sample_timer,
            clock=self._wall_clock,
        )
        self.get_logger().info(
            f'recording {self._schedule_mode} transition telemetry to {output}'
        )

    def _bootstrap_sample_timer(self) -> None:
        if self._sample_timer is not None:
            return
        # Creating a ROS-time timer before the first /clock sample can leave its
        # initial deadline in the system-time epoch.  Start with a wall-clock
        # bootstrap timer, then switch the actual sampler to the node clock as
        # soon as Gazebo /clock has advanced past zero.
        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= 0:
            return
        self.destroy_timer(self._sample_bootstrap_timer)
        self._sample_bootstrap_timer = None
        self._sample_timer = self.create_timer(
            self._sample_period_s,
            self._sample,
        )
        self.get_logger().info(
            'simulation clock ready; transition telemetry sampler running at '
            f'{1.0 / self._sample_period_s:.1f} Hz simulation time'
        )

    def _a0_rl_action_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().warning('ignoring malformed A0 RL action JSON')
            return
        action = _finite(payload.get('action'))
        lambda_d = _finite(payload.get('lambda_d'))
        sequence_id = int(_finite(payload.get('sequence_id'), -1.0))
        if not math.isfinite(action) or not -1.0 <= action <= 1.0:
            self.get_logger().warning('ignoring out-of-range A0 RL action')
            return
        mapped_lambda = min(max(0.5 * (action + 1.0), 0.0), 1.0)
        if not math.isfinite(lambda_d):
            lambda_d = mapped_lambda
        if abs(lambda_d - mapped_lambda) > 1.0e-3:
            self.get_logger().warning(
                'ignoring A0 RL action with inconsistent lambda_d'
            )
            return
        self._a0_rl_action_seq = sequence_id
        self._a0_rl_action = action
        self._a0_rl_lambda_d = lambda_d
        self._a0_rl_action_source = str(
            payload.get('action_source', 'legacy_unknown')
        ).strip() or 'legacy_unknown'
        self._a0_rl_action_received_at = time.monotonic()
        action_sim_time = _finite(payload.get('timestamp_sim_s'))
        self._a0_rl_action_sim_time_s = (
            action_sim_time if math.isfinite(action_sim_time) else None
        )

    def _publish_elevator_id_dither_scale(self, scale: float) -> None:
        scale = min(max(_finite(scale, 0.0), 0.0), 1.0)
        self._elevator_id_dither_scale_command = scale
        message = Float64()
        message.data = scale
        self._elevator_id_dither_publisher.publish(message)

    def _subscribe(self, message_type: type, topic: str, attribute: str) -> None:
        def callback(message: Any) -> None:
            setattr(self, attribute, message)
            self._received_at[attribute] = time.monotonic()
            if attribute == '_position' and self._home_z is None and message.z_valid:
                self._home_z = _finite(message.z)

        self.create_subscription(
            message_type, topic, callback, qos_profile_sensor_data
        )

    def _state_callback(self, message: String) -> None:
        previous_state = self._command_state.split('|', 1)[0]
        self._command_state = message.data
        self._command_state_metadata = _parse_state_metadata(message.data)
        self._command_state_error = _parse_state_error(message.data)
        state = message.data.split('|', 1)[0]
        if self._schedule_mode == 'a0_rl_external':
            if state == 'TRANSITION_FW' and previous_state != 'TRANSITION_FW':
                readiness = transition_start_readiness(
                    self._command_state_metadata
                )
                self._a0_rl_initial_readiness_available = all(
                    key in self._command_state_metadata
                    for key in (
                        'height_gate_ok',
                        'vz_gate_ok',
                        'groundspeed_gate_ok',
                    )
                )
                (
                    self._a0_rl_initial_height_gate_ok,
                    self._a0_rl_initial_vz_gate_ok,
                    self._a0_rl_initial_groundspeed_gate_ok,
                    self._a0_rl_initial_readiness_latched,
                ) = readiness
                if (
                    not self._a0_rl_initial_readiness_available
                    or not self._a0_rl_initial_readiness_latched
                ):
                    reason = (
                        'initial_rl_state_pre_rl_readiness_unavailable'
                        if not self._a0_rl_initial_readiness_available
                        else 'initial_rl_state_pre_rl_readiness_failed'
                    )
                    self._a0_rl_terminal_reason = reason
                    self._va_abort_reason = reason
                    self._va_measurement_interrupted = True
                    self.get_logger().error(
                        'A0 RL transition-start readiness invalid; '
                        f'aborting attempt immediately: {reason}'
                    )
            terminal_reason = a0_transition_terminal_reason(
                previous_state,
                state,
                self._command_state_error,
                self._a0_rl_terminal_reason,
            )
            if terminal_reason and not self._a0_rl_terminal_reason:
                self._a0_rl_terminal_reason = terminal_reason
                if terminal_reason == 'success':
                    # HOLD_FW is published only after PX4 reports fixed-wing
                    # mode with transition_mode cleared.  This, rather than
                    # the preceding capability dwell, is the task success.
                    self._va_measurement_complete = True
                    self.get_logger().info(
                        'A0 RL actual fixed-wing transition confirmed'
                    )
        command_datum = self._command_state_metadata.get(
            'altitude_datum_local_m'
        )
        if command_datum is not None and math.isfinite(command_datum):
            self._command_altitude_datum_local_m = command_datum
        command_target = self._command_state_metadata.get(
            'target_altitude_local_m'
        )
        if command_target is not None and math.isfinite(command_target):
            self._command_target_altitude_local_m = command_target
        if (
            state == 'PREFLIGHT_STREAM'
            and previous_state != 'PREFLIGHT_STREAM'
            and not self._home_reference_locked
            and self._position is not None
            and self._position.z_valid
        ):
            position_z = _finite(self._position.z)
            if math.isfinite(position_z):
                self._home_z = position_z
                self._home_reference_locked = True
                self.get_logger().info(
                    'locked altitude datum at PX4-ready/preflight boundary'
                )
        if state in {'COMPLETED', 'ABORTED', 'ERROR'}:
            if self._terminal_wall_time is None:
                self._terminal_wall_time = time.monotonic()
        if state == 'HOLD_FW' and self._fw_hold_wall_time is None:
            self._fw_hold_wall_time = time.monotonic()

    def _reset_va_hold(self) -> None:
        self._va_hold_phase = 'inactive'
        self._va_settle_started_at = None
        self._va_lambda_settle_started_at = None
        self._va_measurement_started_at = None
        self._va_measurement_interrupted = False
        self._va_velocity_command = math.nan
        self._va_down_velocity_command = math.nan
        self._pusher_throttle_command = math.nan
        self._pusher_pi_integral = 0.0
        self._pusher_airspeed_error_filtered = math.nan
        self._pusher_throttle_unsaturated = math.nan
        self._pusher_last_update_at = None
        self._pusher_control_active = False
        self._va_filtered = math.nan
        self._va_runaway_started_at = None
        self._va_altitude_abort_started_at = None
        self._va_vertical_abort_started_at = None
        self._gust_phase_started_at = None
        self._gust_phase_elapsed_s = math.nan
        self._gust_profile_scale = 0.0

    def _reset_physics_prior_trace(self) -> None:
        self._physics_prior_g_l = math.nan
        self._physics_prior_g_c = math.nan
        self._lambda_phy = math.nan
        self._lambda_max_hard = math.nan
        self._lambda_projected = math.nan
        self._lambda_shield_projection_intervention = math.nan
        self._lambda_shield_total_intervention = math.nan
        self._lambda_shield_emergency_recovery = False
        self._physics_prior_last_update_at = None
        self._physics_prior_previous_command = 0.0

    def _publish_va_hold_velocity(
        self,
        airspeed: float,
        altitude: float,
        vertical_speed_up: float,
    ) -> None:
        error = (
            self._va_target - airspeed if math.isfinite(airspeed) else 0.0
        )
        command_speed = min(
            max(
                self._va_target + self._va_velocity_kp * error,
                self._va_velocity_min,
            ),
            self._va_velocity_max,
        )
        heading = 0.0
        if self._position is not None:
            heading = _finite(getattr(self._position, 'heading', math.nan), 0.0)
        message = Twist()
        message.linear.x = command_speed * math.cos(heading)
        message.linear.y = command_speed * math.sin(heading)
        down_velocity = altitude_hold_down_velocity(
            altitude,
            self._target_altitude,
            vertical_speed_up,
            self._va_altitude_kp,
            self._va_altitude_vertical_speed_kd,
            self._va_altitude_velocity_limit,
        )
        message.linear.z = down_velocity
        message.angular.z = 0.0
        self._velocity_publisher.publish(message)
        self._va_velocity_command = command_speed
        self._va_down_velocity_command = down_velocity

    def _publish_pusher_throttle(self, target: float) -> None:
        message = NormalizedUnsignedSetpoint()
        message.timestamp = self.get_clock().now().nanoseconds // 1000
        message.normalized_setpoint = float(target)
        self._pusher_throttle_publisher.publish(message)
        self._pusher_throttle_command = float(target)

    def _publish_pusher_airspeed_control(self, airspeed: float) -> None:
        if not math.isfinite(airspeed):
            self._pusher_throttle_command = math.nan
            self._pusher_airspeed_error_filtered = math.nan
            self._pusher_throttle_unsaturated = math.nan
            self._pusher_last_update_at = None
            return

        if not math.isfinite(self._va_filtered):
            self._va_filtered = airspeed
        else:
            self._va_filtered = (
                self._va_filter_alpha * airspeed
                + (1.0 - self._va_filter_alpha) * self._va_filtered
            )

        handover_speed = self._va_target - self._pusher_handover_below_target
        if not self._pusher_control_active and self._va_filtered < handover_speed:
            self._pusher_throttle_command = math.nan
            self._pusher_airspeed_error_filtered = math.nan
            self._pusher_throttle_unsaturated = math.nan
            self._pusher_last_update_at = None
            return

        now_wall = time.monotonic()
        if not self._pusher_control_active:
            self._pusher_control_active = True
            self._pusher_pi_integral = 0.0
            self._pusher_last_update_at = now_wall
            initial = min(
                max(self._pusher_airspeed_ff, self._pusher_throttle_min),
                self._pusher_throttle_max,
            )
            self._publish_pusher_throttle(initial)
            return

        previous = (
            self._pusher_throttle_command
            if math.isfinite(self._pusher_throttle_command)
            else self._pusher_airspeed_ff
        )
        dt = (
            now_wall - self._pusher_last_update_at
            if self._pusher_last_update_at is not None else 0.0
        )
        dt = min(max(dt, 0.0), 0.1)
        self._pusher_last_update_at = now_wall

        error = self._va_target - self._va_filtered
        self._pusher_airspeed_error_filtered = error
        unsaturated = (
            self._pusher_airspeed_ff
            + self._pusher_airspeed_kp * error
            + self._pusher_airspeed_ki * self._pusher_pi_integral
        )
        self._pusher_throttle_unsaturated = unsaturated
        saturated = min(
            max(unsaturated, self._pusher_throttle_min),
            self._pusher_throttle_max,
        )
        # Anti-windup: integrate only when the requested correction is not
        # driving further into the active saturation limit.
        if (
            (saturated == unsaturated)
            or (saturated >= self._pusher_throttle_max and error < 0.0)
            or (saturated <= self._pusher_throttle_min and error > 0.0)
        ):
            self._pusher_pi_integral += error * dt

        upper = previous + self._pusher_throttle_slew_up * dt
        lower = previous - self._pusher_throttle_slew_down * dt
        command = min(max(saturated, lower), upper)
        self._publish_pusher_throttle(command)

    def _publish_lambda_command(self, target: float) -> None:
        message = NormalizedUnsignedSetpoint()
        message.timestamp = self.get_clock().now().nanoseconds // 1000
        message.normalized_setpoint = float(target)
        self._lambda_publisher.publish(message)
        self._lambda_command = target

    def _a0_rl_action_age(self, time_s: float = math.nan) -> float:
        if (
            self._a0_rl_action_sim_time_s is not None
            and math.isfinite(time_s)
        ):
            return max(0.0, time_s - self._a0_rl_action_sim_time_s)
        if self._a0_rl_action_received_at is None:
            return math.nan
        return time.monotonic() - self._a0_rl_action_received_at

    def _publish_a0_rl_external_command(
        self,
        *,
        airspeed: float,
        altitude: float,
        vertical_speed_up: float,
        time_s: float = math.nan,
    ) -> None:
        self._publish_va_hold_velocity(airspeed, altitude, vertical_speed_up)
        self._publish_pusher_airspeed_control(airspeed)
        self._lambda_schedule_active = True
        action_age = self._a0_rl_action_age(time_s)
        stale = (
            not math.isfinite(action_age)
            or action_age > self._a0_rl_command_timeout_s
        )
        if stale:
            self._publish_lambda_command(0.0)
            # A physical terminal may be repeated for a short DDS delivery
            # grace period.  The trainer stops publishing actions after the
            # first terminal payload, so action age naturally becomes stale
            # during that grace period.  Preserve the already-latched task
            # outcome instead of relabelling it as a transport timeout.
            if (
                self._va_hold_phase == 'rl_active'
                and not self._va_measurement_interrupted
                and not self._a0_rl_terminal_reason
                and not self._va_abort_reason
            ):
                self._va_abort_reason = 'a0_rl_action_timeout'
                self._va_measurement_interrupted = True
            return
        self._publish_lambda_command(self._a0_rl_lambda_d)
        if self._va_hold_phase == 'inactive':
            self._va_hold_phase = 'rl_active'

        executed = (
            _finite(self._lambda_status.normalized_setpoint)
            if self._lambda_status is not None else math.nan
        )
        stable_altitude = (
            math.isfinite(altitude)
            and abs(altitude - self._target_altitude)
            <= self._transition_altitude_tolerance_m
        )
        stable_vz = (
            math.isfinite(vertical_speed_up)
            and abs(vertical_speed_up)
            <= self._transition_vertical_speed_tolerance_mps
        )
        handover_ready = (
            math.isfinite(airspeed)
            and airspeed >= self._vt_transition_airspeed
            and math.isfinite(executed)
            and executed >= self._a0_rl_handover_lambda
            and stable_altitude
            and stable_vz
        )
        if handover_ready:
            if self._a0_rl_handover_ready_started_at is None:
                self._a0_rl_handover_ready_started_at = time_s
            elif (
                math.isfinite(time_s)
                and time_s - self._a0_rl_handover_ready_started_at
                >= self._a0_rl_handover_dwell_s
            ):
                self._a0_rl_handover_ready = True
                self._lambda_characterization_complete = True
                self.get_logger().info(
                    'A0 RL external handover-ready dwell complete; '
                    'releasing PX4 test hold'
                )
        else:
            self._a0_rl_handover_ready_started_at = None

    def _publish_a0_observation(self, row: dict[str, Any]) -> None:
        if self._schedule_mode != 'a0_rl_external':
            return
        terminal = (
            bool(self._a0_rl_terminal_reason)
            or bool(self._command_state_error)
            or bool(self._va_measurement_complete)
            or bool(self._va_measurement_interrupted)
            or bool(_finite(row.get('failsafe'), 0.0) > 0.5)
        )
        command_error_reason = (
            self._command_state_error.strip().lower().replace(' ', '_')
            if self._command_state_error else ''
        )
        termination_reason = (
            self._a0_rl_terminal_reason
            or command_error_reason
            or self._va_abort_reason
            or ('px4_failsafe' if _finite(row.get('failsafe'), 0.0) > 0.5 else 'running')
        )
        if self._a0_rl_handover_ready and not terminal:
            # Once the external hold is released, actions are intentionally no
            # longer applied.  Suppress intermediate observations so the last
            # applied action connects directly to the authoritative HOLD_FW or
            # command-error terminal instead of adding action-free transitions
            # to replay.
            return
        values = {
            'airspeed_mps': _finite(row.get('selected_airspeed_mps')),
            'angle_of_attack_rad': _finite(row.get('alpha_est_rad')),
            'flight_path_angle_rad': _finite(row.get('gamma_est_rad')),
            'altitude_error_m': _finite(row.get('altitude_error_m')),
            'vertical_speed_up_mps': _finite(row.get('vz_up_mps')),
            'pitch_angle_rad': _finite(row.get('pitch_rad')),
            'pitch_rate_rps': _finite(row.get('q_rad_s')),
            'lambda_executed': _finite(row.get('lambda_exec')),
            'lift_rotor_power_w': _finite(row.get('lift_power_w')),
            'pusher_power_w': _finite(row.get('pusher_power_w')),
        }
        terminal_fallback_used = False
        terminal_fallback_fields: list[str] = []
        try:
            observation = pack_a0_observation(values)
        except (KeyError, ValueError):
            if not terminal:
                return
            fallback_defaults = {
                'airspeed_mps': 0.0,
                'angle_of_attack_rad': 0.0,
                'flight_path_angle_rad': 0.0,
                'altitude_error_m': 0.0,
                'vertical_speed_up_mps': 0.0,
                'pitch_angle_rad': 0.0,
                'pitch_rate_rps': 0.0,
                'lambda_executed': 0.0,
                'lift_rotor_power_w': 0.0,
                'pusher_power_w': 0.0,
            }
            repaired: dict[str, float] = {}
            last = self._last_a0_observation
            for index, field in enumerate(A0_OBSERVATION_FIELDS):
                value = values[field]
                if math.isfinite(value):
                    repaired[field] = value
                    continue
                terminal_fallback_fields.append(field)
                if last is not None and index < len(last):
                    repaired[field] = float(last[index])
                else:
                    repaired[field] = fallback_defaults[field]
            try:
                observation = pack_a0_observation(repaired)
            except (KeyError, ValueError):
                return
            terminal_fallback_used = True
        else:
            self._last_a0_observation = [float(item) for item in observation]
        payload = {
            'schema_version': row.get('schema_version'),
            'sim_time_s': row.get('time_s'),
            'condition_id': row.get('condition_id'),
            'command_state': row.get('command_state'),
            'vtol_state': row.get('vtol_state'),
            'observation_fields': list(A0_OBSERVATION_FIELDS),
            'observation': [float(item) for item in observation],
            'terminated': terminal,
            'truncated': False,
            'termination_reason': termination_reason,
            'a0_observation_terminal_fallback_used': int(
                terminal_fallback_used
            ),
            'a0_observation_terminal_fallback_fields': (
                terminal_fallback_fields
            ),
            'a0_rl_action_seq': self._a0_rl_action_seq,
            'a0_rl_action': row.get('a0_rl_action'),
            'a0_rl_lambda_d': row.get('a0_rl_lambda_d'),
            'a0_rl_action_source': row.get('a0_rl_action_source'),
            'a0_rl_action_age_s': row.get('a0_rl_action_age_s'),
            'a0_rl_action_stale': row.get('a0_rl_action_stale'),
            'a0_rl_control_phase': row.get('a0_rl_control_phase'),
            'a0_rl_handover_ready': row.get('a0_rl_handover_ready'),
            'a0_rl_actual_fw_confirmed': row.get(
                'a0_rl_actual_fw_confirmed'
            ),
            'a0_rl_initial_readiness_available': row.get(
                'a0_rl_initial_readiness_available'
            ),
            'a0_rl_initial_readiness_latched': row.get(
                'a0_rl_initial_readiness_latched'
            ),
            'a0_rl_initial_height_gate_ok': row.get(
                'a0_rl_initial_height_gate_ok'
            ),
            'a0_rl_initial_vz_gate_ok': row.get(
                'a0_rl_initial_vz_gate_ok'
            ),
            'a0_rl_initial_groundspeed_gate_ok': row.get(
                'a0_rl_initial_groundspeed_gate_ok'
            ),
            'height_gate_ok': row.get('height_gate_ok'),
            'vz_gate_ok': row.get('vz_gate_ok'),
            'groundspeed_gate_ok': row.get('groundspeed_gate_ok'),
            'stability_gate_ok': row.get('stability_gate_ok'),
            'a0_observation_sim_dt_s': row.get('a0_observation_sim_dt_s'),
            'a0_observation_wall_dt_s': row.get('a0_observation_wall_dt_s'),
        }
        message = String()
        message.data = json.dumps(_json_safe(payload), allow_nan=False)
        self._a0_observation_publisher.publish(message)

    @staticmethod
    def _inverse_smooth_gate(value: float, lower: float, upper: float) -> float:
        if not math.isfinite(value):
            return 0.0
        return 1.0 - smoothstep(value, lower, upper)

    def _publish_physics_prior_command(
        self,
        *,
        eta_l: float,
        eta_c: float,
        altitude: float,
        vertical_speed_up: float,
        alpha: float,
        alpha_valid: bool,
    ) -> None:
        """Publish lambda_phy after hard-envelope projection and rate limiting."""
        self._lambda_schedule_active = True
        eta_l_for_prior = eta_l if math.isfinite(eta_l) else 0.0
        eta_c_for_prior = eta_c if math.isfinite(eta_c) else 0.0
        self._physics_prior_g_l = smoothstep(
            min(max(eta_l_for_prior, 0.0), 1.0),
            self._physics_prior_eta_l_lower,
            self._physics_prior_eta_l_upper,
        )
        self._physics_prior_g_c = smoothstep(
            min(max(eta_c_for_prior, 0.0), 1.0),
            self._physics_prior_eta_c_lower,
            self._physics_prior_eta_c_upper,
        )
        self._lambda_phy = physics_prior(
            eta_l_for_prior,
            eta_c_for_prior,
            eta_l_lower=self._physics_prior_eta_l_lower,
            eta_l_upper=self._physics_prior_eta_l_upper,
            eta_c_lower=self._physics_prior_eta_c_lower,
            eta_c_upper=self._physics_prior_eta_c_upper,
        )
        eta_l_hard_gate = smoothstep(
            min(max(eta_l_for_prior, 0.0), 1.0),
            self._physics_prior_hard_eta_l_lower,
            self._physics_prior_hard_eta_l_upper,
        )
        eta_c_hard_gate = smoothstep(
            min(max(eta_c_for_prior, 0.0), 1.0),
            self._physics_prior_hard_eta_c_lower,
            self._physics_prior_hard_eta_c_upper,
        )
        altitude_drop = (
            max(0.0, self._target_altitude - altitude)
            if math.isfinite(altitude) else self._physics_prior_altitude_drop_hard
        )
        descent_rate = (
            max(0.0, -vertical_speed_up)
            if math.isfinite(vertical_speed_up)
            else self._physics_prior_descent_hard
        )
        alpha_magnitude = (
            abs(alpha)
            if alpha_valid and math.isfinite(alpha)
            else 0.0
        )
        altitude_gate = self._inverse_smooth_gate(
            altitude_drop,
            self._physics_prior_altitude_drop_soft,
            self._physics_prior_altitude_drop_hard,
        )
        descent_gate = self._inverse_smooth_gate(
            descent_rate,
            self._physics_prior_descent_soft,
            self._physics_prior_descent_hard,
        )
        alpha_gate = self._inverse_smooth_gate(
            alpha_magnitude,
            self._physics_prior_alpha_soft,
            self._physics_prior_alpha_hard,
        )
        emergency = (
            altitude_drop >= self._physics_prior_altitude_drop_soft
            or descent_rate >= self._physics_prior_descent_soft
            or alpha_magnitude >= self._physics_prior_alpha_soft
        )
        now_wall = time.monotonic()
        dt_s = (
            now_wall - self._physics_prior_last_update_at
            if self._physics_prior_last_update_at is not None
            else self._sample_period_s
        )
        dt_s = min(max(dt_s, 1.0e-3), 0.2)
        result = apply_unloading_shield(
            candidate=self._lambda_phy,
            gates=(
                eta_l_hard_gate,
                eta_c_hard_gate,
                altitude_gate,
                descent_gate,
                alpha_gate,
            ),
            previous_executed=self._physics_prior_previous_command,
            dt_s=dt_s,
            rate_limiter=self._physics_prior_rate_limiter,
            emergency_recovery=emergency,
        )
        self._physics_prior_last_update_at = now_wall
        self._physics_prior_previous_command = result.executed
        self._lambda_max_hard = result.hard_limit
        self._lambda_projected = result.projected
        self._lambda_shield_projection_intervention = (
            result.projection_intervention
        )
        self._lambda_shield_total_intervention = result.total_intervention
        self._lambda_shield_emergency_recovery = emergency
        self._publish_lambda_command(result.executed)

    def _publish_gust_command(self, magnitude_mps: float) -> None:
        """Publish a longitudinal tailwind command in the Gazebo ENU frame."""
        heading = 0.0
        if self._position is not None:
            heading = _finite(getattr(self._position, 'heading', math.nan), 0.0)
        # PX4 heading is measured clockwise from north.  Gazebo world x/y are
        # east/north in this project, so a positive longitudinal tailwind is
        # (east, north) = magnitude * (sin(heading), cos(heading)).
        self._wind_cmd_e = magnitude_mps * math.sin(heading)
        self._wind_cmd_n = magnitude_mps * math.cos(heading)
        self._wind_cmd_u = 0.0
        message = Vector3Stamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'world_enu'
        message.vector.x = self._wind_cmd_e
        message.vector.y = self._wind_cmd_n
        message.vector.z = self._wind_cmd_u
        self._gust_publisher.publish(message)

    def _gust_actual_vector(self, now_wall: float) -> tuple[float, float, float, float]:
        received = self._received_at.get('_gust_status')
        age = now_wall - received if received is not None else math.nan
        if (
            self._gust_status is None
            or not math.isfinite(age)
            or age > self._gust_status_timeout_s
        ):
            return math.nan, math.nan, math.nan, age
        return (
            _finite(self._gust_status.vector.x),
            _finite(self._gust_status.vector.y),
            _finite(self._gust_status.vector.z),
            age,
        )

    def _advance_gust_phase(self, now_wall: float) -> None:
        """Advance the frozen baseline / 1-cos exposure / recovery protocol."""
        if self._gust_phase_started_at is None:
            self._gust_phase_started_at = now_wall
        elapsed = max(0.0, now_wall - self._gust_phase_started_at)
        self._gust_phase_elapsed_s = elapsed
        scale = 0.0
        if self._va_hold_phase == 'gust_baseline':
            if elapsed >= self._gust_delay_s:
                self._va_hold_phase = 'gust_exposure'
                self._gust_phase_started_at = now_wall
                self._gust_phase_elapsed_s = 0.0
                self.get_logger().info('F2-B baseline established; starting gust')
        elif self._va_hold_phase == 'gust_exposure':
            phase_elapsed = min(elapsed, self._gust_duration_s)
            scale = 0.5 * (
                1.0 - math.cos(2.0 * math.pi * phase_elapsed /
                               self._gust_duration_s)
            )
            if elapsed >= self._gust_duration_s:
                self._va_hold_phase = 'gust_recovery'
                self._gust_phase_started_at = now_wall
                self._gust_phase_elapsed_s = 0.0
                scale = 0.0
                self.get_logger().info('F2-B gust complete; starting recovery')
        elif self._va_hold_phase == 'gust_recovery':
            if elapsed >= self._gust_recovery_s:
                self._va_measurement_complete = True
                self._lambda_characterization_complete = True
                self._va_hold_phase = 'gust_complete'
                self._gust_phase_started_at = now_wall
                self._gust_phase_elapsed_s = 0.0
                self.get_logger().info(
                    'F2-B recovery window complete; releasing PX4 test hold'
                )
        self._gust_profile_scale = scale
        self._publish_gust_command(self._gust_amplitude * scale)

    def _publish_schedule(
        self,
        airspeed: float,
        altitude: float = math.nan,
        vertical_speed_up: float = math.nan,
        eta_l: float = math.nan,
        eta_c: float = math.nan,
        alpha: float = math.nan,
        alpha_valid: bool = False,
        time_s: float = math.nan,
    ) -> None:
        if self._schedule_mode == 'none':
            self._publish_elevator_id_dither_scale(0.0)
            self._lambda_command = math.nan
            self._reset_va_hold()
            self._reset_physics_prior_trace()
            return
        state = self._command_state.split('|', 1)[0]
        if state != 'TRANSITION_FW':
            self._lambda_schedule_active = False
            self._lambda_target_dwell_started_at = None
            self._lambda_command = math.nan
            self._publish_elevator_id_dither_scale(0.0)
            self._reset_physics_prior_trace()
            if not self._va_measurement_complete:
                self._reset_va_hold()
            return
        if (
            self._schedule_mode in {
                'manual_target', 'va_hold_target', 'gust_target',
                'physics_prior_only', 'a0_rl_external',
            }
            and self._lambda_characterization_complete
        ):
            # Stop refreshing the command. PX4's 500 ms fail-safe releases
            # the test hold, restores collective support, and permits normal
            # completion to fixed-wing.
            self._lambda_schedule_active = False
            self._lambda_command = math.nan
            self._va_velocity_command = math.nan
            self._va_down_velocity_command = math.nan
            self._pusher_throttle_command = math.nan
            self._publish_elevator_id_dither_scale(0.0)
            return
        if self._schedule_mode == 'a0_rl_external':
            self._publish_a0_rl_external_command(
                airspeed=airspeed,
                altitude=altitude,
                vertical_speed_up=vertical_speed_up,
                time_s=time_s,
            )
            self._publish_elevator_id_dither_scale(0.0)
            return
        if self._schedule_mode == 'physics_prior_only':
            self._publish_pusher_airspeed_control(airspeed)
            self._publish_physics_prior_command(
                eta_l=eta_l,
                eta_c=eta_c,
                altitude=altitude,
                vertical_speed_up=vertical_speed_up,
                alpha=alpha,
                alpha_valid=alpha_valid,
            )
            executed = (
                _finite(self._lambda_status.normalized_setpoint)
                if self._lambda_status is not None else math.nan
            )
            full_prior_ready = (
                math.isfinite(self._lambda_phy)
                and math.isfinite(executed)
                and self._lambda_phy >= self._physics_prior_release_lambda
                and executed >= self._physics_prior_release_lambda
                and math.isfinite(airspeed)
                and airspeed >= self._vt_transition_airspeed
            )
            if full_prior_ready:
                if self._lambda_target_dwell_started_at is None:
                    self._lambda_target_dwell_started_at = time_s
                elif (
                    math.isfinite(time_s)
                    and time_s - self._lambda_target_dwell_started_at
                    >= self._physics_prior_release_dwell_s
                ):
                    self._lambda_characterization_complete = True
                    self.get_logger().info(
                        'physics-prior full-allocation dwell complete; '
                        'releasing PX4 test hold'
                    )
            else:
                self._lambda_target_dwell_started_at = None
            self._publish_elevator_id_dither_scale(0.0)
            return
        if self._schedule_mode in {'va_hold_target', 'gust_target'}:
            if self._schedule_mode == 'gust_target' and self._va_hold_phase in {
                'inactive', 'airspeed_settle', 'lambda_settle'
            }:
                # Keep the bridge alive at zero wind while the target
                # condition is established.  No gust is permitted before the
                # Va/lambda dwell completes.
                self._gust_profile_scale = 0.0
                self._publish_gust_command(0.0)
            self._publish_va_hold_velocity(
                airspeed, altitude, vertical_speed_up
            )
            self._publish_pusher_airspeed_control(airspeed)
            self._lambda_schedule_active = True
            external_active = (
                _finite(self._lambda_active_status.normalized_setpoint) > 0.5
                if self._lambda_active_status is not None else False
            )
            executed = (
                _finite(self._lambda_status.normalized_setpoint)
                if self._lambda_status is not None else math.nan
            )
            va_ok = (
                math.isfinite(airspeed)
                and abs(airspeed - self._va_target)
                <= self._va_target_tolerance
            )
            now_wall = time.monotonic()

            if self._va_hold_phase == 'inactive':
                self._va_hold_phase = 'airspeed_settle'
            target = 0.0
            if self._va_hold_phase == 'airspeed_settle':
                if va_ok:
                    if self._va_settle_started_at is None:
                        self._va_settle_started_at = now_wall
                    elif (
                        now_wall - self._va_settle_started_at
                        >= self._va_target_dwell_s
                    ):
                        self._va_hold_phase = 'lambda_settle'
                        self._va_lambda_settle_started_at = None
                        target = self._lambda_target
                        self.get_logger().info(
                            'airspeed target dwell complete; settling lambda'
                        )
                else:
                    self._va_settle_started_at = None
            else:
                target = self._lambda_target

            self._publish_lambda_command(target)

            lambda_ok = external_active and within_lambda_target(
                executed,
                self._lambda_target,
                self._lambda_target_tolerance,
            )
            both_ok = va_ok and lambda_ok
            if self._va_hold_phase == 'lambda_settle':
                if both_ok:
                    if self._va_lambda_settle_started_at is None:
                        self._va_lambda_settle_started_at = now_wall
                    elif (
                        now_wall - self._va_lambda_settle_started_at
                        >= self._lambda_target_dwell_s
                    ):
                        if self._schedule_mode == 'gust_target':
                            self._va_hold_phase = 'gust_baseline'
                            self._gust_phase_started_at = now_wall
                            self._gust_phase_elapsed_s = 0.0
                            self.get_logger().info(
                                'airspeed and lambda dwell complete; '
                                'starting F2-B baseline delay'
                            )
                        else:
                            self._va_hold_phase = 'measurement'
                            self._va_measurement_started_at = None
                            self.get_logger().info(
                                'airspeed and lambda dwell complete; measuring'
                            )
                else:
                    self._va_lambda_settle_started_at = None
            if self._va_hold_phase == 'measurement':
                # Schema v9 freezes one continuous wall-clock window after
                # the joint Va/lambda dwell.  A soft-band excursion does not
                # reset, splice, or terminate this window; the evaluator
                # applies the predeclared occupancy and maximum-error guards
                # after all three seconds have been collected.  Hard flight
                # safety remains an independent real-time abort below.
                if self._va_measurement_started_at is None:
                    self._va_measurement_started_at = now_wall
                elif (
                    now_wall - self._va_measurement_started_at
                    >= self._va_measurement_s
                ):
                    self._va_measurement_complete = True
                    self._lambda_characterization_complete = True
                    self.get_logger().info(
                        'Va-hold fixed measurement complete; '
                        'releasing PX4 test hold'
                    )
            if self._schedule_mode == 'gust_target' and self._va_hold_phase in {
                'gust_baseline', 'gust_exposure', 'gust_recovery'
            }:
                self._advance_gust_phase(now_wall)
            self._publish_elevator_id_dither_scale(
                1.0
                if self._elevator_id_dither_enabled
                and self._va_hold_phase == 'measurement'
                and not self._va_measurement_complete
                else 0.0
            )
            return
        if math.isfinite(airspeed) and airspeed > self._lambda_start:
            self._lambda_schedule_active = True
        if not self._lambda_schedule_active:
            # Keep PX4's native transition active until the manual schedule's
            # zero point.  Sending an external zero earlier would take over the
            # collective path before the pusher has established airspeed.
            self._lambda_command = math.nan
            return
        target = (
            self._lambda_target
            if self._schedule_mode == 'manual_target'
            else airspeed_unloading(
                airspeed,
                self._lambda_start,
                self._lambda_full,
                self._lambda_max,
            )
        )
        self._publish_lambda_command(target)
        self._publish_elevator_id_dither_scale(0.0)

        if self._schedule_mode == 'manual_target':
            executed = (
                _finite(self._lambda_status.normalized_setpoint)
                if self._lambda_status is not None else math.nan
            )
            external_active = (
                _finite(self._lambda_active_status.normalized_setpoint) > 0.5
                if self._lambda_active_status is not None else False
            )
            if external_active and within_lambda_target(
                executed,
                self._lambda_target,
                self._lambda_target_tolerance,
            ):
                if self._lambda_target_dwell_started_at is None:
                    self._lambda_target_dwell_started_at = time.monotonic()
                elif (
                    time.monotonic() - self._lambda_target_dwell_started_at
                    >= self._lambda_target_dwell_s
                ):
                    self._lambda_characterization_complete = True
                    self.get_logger().info(
                        'lambda target dwell complete; releasing PX4 test hold'
                    )
            else:
                self._lambda_target_dwell_started_at = None

    def _motor_values(self) -> tuple[list[float], float, float, float, float]:
        omega = [math.nan] * 5
        if self._esc is not None:
            for index in range(min(5, int(self._esc.esc_count), len(self._esc.esc))):
                omega[index] = abs(_finite(self._esc.esc[index].esc_rpm))
        lift_valid = [item for item in omega[:4] if math.isfinite(item)]
        pusher = omega[4]
        lift_thrust = sum(self._lift_kt * item ** 2 for item in lift_valid)
        lift_power = sum(self._lift_kt * self._lift_km * item ** 3 for item in lift_valid)
        pusher_thrust = (
            self._pusher_kt * pusher ** 2 if math.isfinite(pusher) else math.nan
        )
        pusher_power = (
            self._pusher_kt * self._pusher_km * pusher ** 3
            if math.isfinite(pusher) else math.nan
        )
        if not lift_valid:
            lift_thrust = math.nan
            lift_power = math.nan
        return omega, lift_thrust, pusher_thrust, lift_power, pusher_power

    def _sample(self) -> None:
        sample_wall_time = time.monotonic()

        gust_actual_e, gust_actual_n, gust_actual_u, gust_status_age = (
            self._gust_actual_vector(sample_wall_time)
        )
        if self._schedule_mode == 'gust_target':
            wind_e, wind_n, wind_u = (
                gust_actual_e, gust_actual_n, gust_actual_u
            )
        else:
            wind_e, wind_n, wind_u = (
                self._wind_cmd_e, self._wind_cmd_n, self._wind_cmd_u
            )

        def message_age(attribute: str) -> float:
            received = self._received_at.get(attribute)
            return sample_wall_time - received if received is not None else math.nan

        position = self._position
        airspeed = (
            _finite(self._airspeed.true_airspeed_m_s)
            if self._airspeed is not None else math.nan
        )
        pre_vn = pre_ve = pre_vz_up = math.nan
        if position is not None:
            pre_vn = _finite(position.vx)
            pre_ve = _finite(position.vy)
            pre_vz_up = -_finite(position.vz)
        _, _, _, pre_vrel_norm = wind_relative_velocity_enu(
            pre_vn, pre_ve, pre_vz_up,
            wind_e, wind_n, wind_u,
        )
        selected_airspeed = (
            pre_vrel_norm
            if self._airspeed_source == 'relative_wind' else airspeed
        )
        (
            roll, pitch, yaw, raw_alpha, gamma,
            body_u, body_v, body_w, beta,
        ) = _euler_and_body_velocity(
            self._attitude,
            position,
            wind_east_mps=(
                wind_e
                if self._airspeed_source == 'relative_wind' else 0.0
            ),
            wind_north_mps=(
                wind_n
                if self._airspeed_source == 'relative_wind' else 0.0
            ),
            wind_up_mps=(
                wind_u
                if self._airspeed_source == 'relative_wind' else 0.0
            ),
        )
        roll_setpoint = pitch_setpoint = yaw_setpoint = math.nan
        if self._attitude_setpoint is not None:
            roll_setpoint = _finite(self._attitude_setpoint.roll_body)
            pitch_setpoint = _finite(self._attitude_setpoint.pitch_body)
            yaw_setpoint = _finite(self._attitude_setpoint.yaw_body)
        alpha, alpha_valid = gated_angle_of_attack(
            raw_alpha,
            selected_airspeed,
            self._alpha_min_airspeed,
        )
        beta_valid = bool(
            math.isfinite(beta)
            and math.isfinite(selected_airspeed)
            and selected_airspeed >= self._eta_l_min_airspeed
        )
        omega, lift_thrust, pusher_thrust, lift_power, pusher_power = self._motor_values()

        time_s = self.get_clock().now().nanoseconds * 1e-9
        a0_observation_sim_dt = math.nan
        a0_observation_wall_dt = math.nan
        if self._schedule_mode == 'a0_rl_external':
            if self._last_a0_observation_sim_time_s is not None:
                a0_observation_sim_dt = (
                    time_s - self._last_a0_observation_sim_time_s
                )
            if self._last_a0_observation_wall_time_s is not None:
                a0_observation_wall_dt = (
                    sample_wall_time - self._last_a0_observation_wall_time_s
                )
            self._last_a0_observation_sim_time_s = time_s
            self._last_a0_observation_wall_time_s = sample_wall_time
        else:
            self._last_a0_observation_sim_time_s = None
            self._last_a0_observation_wall_time_s = None
        altitude = math.nan
        altitude_local = math.nan
        altitude_datum_local = math.nan
        target_altitude_local = math.nan
        height_gate_ok = False
        vz_gate_ok = False
        groundspeed_gate_ok = False
        stability_gate_ok = False
        stability_dwell_s = math.nan
        north = east = vn = ve = vz_up = groundspeed = math.nan
        if position is not None:
            north, east = _finite(position.x), _finite(position.y)
            vn, ve, vz_up = _finite(position.vx), _finite(position.vy), -_finite(position.vz)
            groundspeed = math.hypot(vn, ve) if math.isfinite(vn) and math.isfinite(ve) else math.nan
            position_z = _finite(position.z)
            altitude_local = -position_z
            if self._command_altitude_datum_local_m is not None:
                altitude_datum_local = self._command_altitude_datum_local_m
            elif self._home_z is not None:
                altitude_datum_local = -self._home_z
            if math.isfinite(altitude_local) and math.isfinite(altitude_datum_local):
                altitude = altitude_local - altitude_datum_local
            if self._command_target_altitude_local_m is not None:
                target_altitude_local = self._command_target_altitude_local_m
            elif math.isfinite(altitude_datum_local):
                target_altitude_local = (
                    altitude_datum_local + self._target_altitude
                )

        if math.isfinite(altitude):
            height_gate_ok = (
                abs(altitude - self._target_altitude)
                <= self._transition_altitude_tolerance_m
            )
        if math.isfinite(vz_up):
            vz_gate_ok = (
                abs(vz_up) <= self._transition_vertical_speed_tolerance_mps
            )
        if math.isfinite(groundspeed):
            groundspeed_gate_ok = (
                groundspeed <= self._transition_groundspeed_tolerance_mps
            )
        state = self._command_state.split('|', 1)[0]
        stability_gate_ok = height_gate_ok and vz_gate_ok and groundspeed_gate_ok
        if state == 'HOLD_MC' and stability_gate_ok and math.isfinite(time_s):
            if self._local_stability_since_s is None:
                self._local_stability_since_s = time_s
            stability_dwell_s = max(0.0, time_s - self._local_stability_since_s)
        else:
            self._local_stability_since_s = None
            stability_dwell_s = 0.0 if state == 'HOLD_MC' else math.nan
        command_dwell = self._command_state_metadata.get('stability_dwell_s')
        if command_dwell is not None and math.isfinite(command_dwell):
            stability_dwell_s = command_dwell

        # Preserve the frozen fixed-Va/gust timing: those schedules do not
        # need eta_C and historically advanced before the real-time abort
        # checks below.  Physics-prior-only is published later, after eta_L
        # and eta_C have been computed for this sample.
        if self._schedule_mode != 'physics_prior_only':
            self._publish_schedule(
                selected_airspeed,
                altitude,
                vz_up,
                time_s=time_s,
            )

        va_abort_now = self._va_measurement_interrupted
        va_vertical_abort_enabled = False
        va_low_speed_vertical_transient = False
        if (
            self._schedule_mode in {
                'va_hold_target', 'gust_target', 'a0_rl_external'
            }
            and state == 'TRANSITION_FW'
            and not self._va_measurement_complete
            and not self._va_measurement_interrupted
        ):
            overspeed = (
                math.isfinite(selected_airspeed)
                and selected_airspeed
                > self._va_target + self._va_runaway_margin
            )
            altitude_runaway = (
                math.isfinite(altitude)
                and abs(altitude - self._target_altitude)
                > self._va_altitude_abort_error
            )
            vertical_speed_exceeded = (
                math.isfinite(vz_up)
                and abs(vz_up) > self._va_vertical_speed_abort
            )
            va_vertical_abort_enabled = (
                (
                    math.isfinite(selected_airspeed)
                    and selected_airspeed
                    >= self._va_vertical_abort_min_airspeed
                )
                or self._pusher_control_active
            )
            vertical_runaway = (
                vertical_speed_exceeded and va_vertical_abort_enabled
            )
            va_low_speed_vertical_transient = (
                vertical_speed_exceeded and not va_vertical_abort_enabled
            )

            (
                self._va_runaway_started_at,
                overspeed_sustained,
            ) = sustained_sim_time(
                overspeed,
                self._va_runaway_started_at,
                time_s,
                self._va_runaway_dwell_s,
            )
            (
                self._va_altitude_abort_started_at,
                altitude_sustained,
            ) = sustained_sim_time(
                altitude_runaway,
                self._va_altitude_abort_started_at,
                time_s,
                self._va_runaway_dwell_s,
            )
            (
                self._va_vertical_abort_started_at,
                vertical_sustained,
            ) = sustained_sim_time(
                vertical_runaway,
                self._va_vertical_abort_started_at,
                time_s,
                self._va_runaway_dwell_s,
            )
            if overspeed_sustained:
                self._va_abort_reason = 'airspeed_hold_runaway'
                va_abort_now = True
            elif altitude_sustained:
                if (
                    self._va_hold_phase == 'airspeed_settle'
                    and (
                        not math.isfinite(selected_airspeed)
                        or selected_airspeed < self._alpha_min_airspeed
                    )
                ):
                    self._va_abort_reason = (
                        'pre_measurement_altitude_transient'
                    )
                else:
                    self._va_abort_reason = 'va_hold_altitude_runaway'
                va_abort_now = True
            elif vertical_sustained:
                self._va_abort_reason = 'va_hold_vertical_speed_runaway'
                va_abort_now = True

        if va_abort_now:
            self._va_measurement_interrupted = True

        rates = [math.nan] * 3
        if self._angular_velocity is not None:
            rates = [_finite(item) for item in self._angular_velocity.xyz]
        servos = [math.nan] * 3
        if self._servos is not None:
            servos = [_finite(item) for item in self._servos.control[:3]]
        motor_controls = [math.nan] * 5
        if self._actuator_motors is not None:
            motor_controls = [
                _finite(item) for item in self._actuator_motors.control[:5]
            ]
        pusher_thrust_setpoint = lift_collective_thrust_setpoint = math.nan
        if self._thrust_setpoint is not None:
            pusher_thrust_setpoint = _finite(self._thrust_setpoint.xyz[0])
            lift_collective_thrust_setpoint = -_finite(
                self._thrust_setpoint.xyz[2]
            )
        trajectory_sp_altitude_local = math.nan
        trajectory_sp_altitude_relative = math.nan
        trajectory_sp_vz_up = math.nan
        if self._trajectory_setpoint is not None:
            sp_z = _finite(self._trajectory_setpoint.position[2])
            if math.isfinite(sp_z):
                trajectory_sp_altitude_local = -sp_z
                if math.isfinite(altitude_datum_local):
                    trajectory_sp_altitude_relative = (
                        trajectory_sp_altitude_local - altitude_datum_local
                    )
            sp_vz_down = _finite(self._trajectory_setpoint.velocity[2])
            if math.isfinite(sp_vz_down):
                trajectory_sp_vz_up = -sp_vz_down
        (
            vrel_e,
            vrel_n,
            vrel_u,
            vrel_norm,
        ) = wind_relative_velocity_enu(
            vn, ve, vz_up,
            wind_e, wind_n, wind_u,
        )
        airspeed_relative_error = (
            airspeed - vrel_norm
            if math.isfinite(airspeed) and math.isfinite(vrel_norm)
            else math.nan
        )
        selected_airspeed = (
            vrel_norm if self._airspeed_source == 'relative_wind' else airspeed
        )
        selected_airspeed_error = (
            selected_airspeed - self._va_target
            if self._schedule_mode in {'va_hold_target', 'gust_target'}
            and math.isfinite(selected_airspeed)
            else math.nan
        )
        wind_heading_parallel = wind_heading_cross = math.nan
        if all(math.isfinite(value) for value in (wind_e, wind_n, yaw)):
            wind_heading_parallel = (
                wind_e * math.sin(yaw) + wind_n * math.cos(yaw)
            )
            wind_heading_cross = (
                wind_e * math.cos(yaw) - wind_n * math.sin(yaw)
            )
        battery_voltage = battery_current = battery_power = math.nan
        if self._battery is not None:
            battery_voltage = _finite(self._battery.voltage_v)
            battery_current = _finite(self._battery.current_a)
            if math.isfinite(battery_voltage) and math.isfinite(battery_current) and battery_current >= 0.0:
                battery_power = battery_voltage * battery_current
        pusher_throttle_status = (
            _finite(self._pusher_throttle_status.normalized_setpoint)
            if self._pusher_throttle_status is not None else math.nan
        )
        pusher_throttle_active = (
            _finite(self._pusher_throttle_active.normalized_setpoint)
            if self._pusher_throttle_active is not None else math.nan
        )
        lambda_exec = (
            _finite(self._lambda_status.normalized_setpoint)
            if self._lambda_status is not None else math.nan
        )
        mc_pitch_weight = (
            _finite(self._mc_pitch_weight_status.normalized_setpoint)
            if self._mc_pitch_weight_status is not None else math.nan
        )
        fw_pitch_weight = (
            _finite(self._fw_pitch_weight_status.normalized_setpoint)
            if self._fw_pitch_weight_status is not None else math.nan
        )
        mc_pitch_demand = (
            _finite(self._mc_torque_setpoint.xyz[1])
            if self._mc_torque_setpoint is not None else math.nan
        )
        fw_pitch_demand = (
            _finite(self._fw_torque_setpoint.xyz[1])
            if self._fw_torque_setpoint is not None else math.nan
        )
        elevator_angle_proxy = (
            min(max(servos[2], self._elevator_joint_min),
                self._elevator_joint_max)
            if math.isfinite(servos[2]) else math.nan
        )
        elevator_limit_tolerance = 0.01 * (
            self._elevator_joint_max - self._elevator_joint_min
        )
        elevator_joint_limit_commanded = (
            int(
                servos[2] <= self._elevator_joint_min + elevator_limit_tolerance
                or servos[2] >= self._elevator_joint_max - elevator_limit_tolerance
            )
            if math.isfinite(servos[2]) else math.nan
        )

        wing_left_age = message_age('_wing_lift_left_gt')
        wing_right_age = message_age('_wing_lift_right_gt')

        def wing_force(message: Optional[WrenchStamped]) -> tuple[float, float, float]:
            if message is None:
                return math.nan, math.nan, math.nan
            return (
                _finite(message.wrench.force.x),
                _finite(message.wrench.force.y),
                _finite(message.wrench.force.z),
            )

        def wrench_components(
            message: Optional[WrenchStamped],
        ) -> tuple[float, float, float, float, float, float]:
            if message is None:
                return (math.nan,) * 6
            return (
                _finite(message.wrench.force.x),
                _finite(message.wrench.force.y),
                _finite(message.wrench.force.z),
                _finite(message.wrench.torque.x),
                _finite(message.wrench.torque.y),
                _finite(message.wrench.torque.z),
            )

        wing_left_force = wing_force(self._wing_lift_left_gt)
        wing_right_force = wing_force(self._wing_lift_right_gt)
        elevator_aero_age = message_age('_elevator_aero_wrench_gt')
        elevator_aero_wrench = wrench_components(self._elevator_aero_wrench_gt)
        elevator_aero_wrench_valid = bool(
            self._wing_force_gt_enabled
            and math.isfinite(elevator_aero_age)
            and elevator_aero_age <= self._wing_force_gt_timeout_s
            and all(math.isfinite(value) for value in elevator_aero_wrench)
        )
        # For the current straight-ahead SITL qualification runs this is the
        # ENU-Y torque proxy.  The full ENU torque vector is also recorded so
        # F3-B1 can re-project the moment if the body-axis convention changes.
        elevator_pitch_moment_gt = (
            elevator_aero_wrench[4]
            if elevator_aero_wrench_valid else math.nan
        )
        wing_lift_gt_age = (
            max(wing_left_age, wing_right_age)
            if math.isfinite(wing_left_age) and math.isfinite(wing_right_age)
            else math.nan
        )
        wing_lift_gt_valid = bool(
            self._wing_force_gt_enabled
            and math.isfinite(wing_lift_gt_age)
            and wing_lift_gt_age <= self._wing_force_gt_timeout_s
            and all(math.isfinite(value) for value in (
                *wing_left_force, *wing_right_force,
            ))
        )
        wing_total_force = (
            tuple(
                wing_left_force[index] + wing_right_force[index]
                for index in range(3)
            )
            if wing_lift_gt_valid else (math.nan, math.nan, math.nan)
        )

        eta_l_valid = False
        eta_l_force_model_valid = False
        eta_l_force_model_invalid_reason = 'unavailable'
        eta_l_cl = math.nan
        eta_l_dynamic_pressure = (
            0.5 * self._eta_l_density * selected_airspeed**2
            if math.isfinite(selected_airspeed) and selected_airspeed >= 0.0
            else math.nan
        )
        wing_lift_est = wing_vertical_support_est = math.nan
        wing_required_support = eta_l = eta_l_gt = math.nan
        if (
            math.isfinite(selected_airspeed)
            and selected_airspeed >= 0.0
            and math.isfinite(self._eta_l_estimated_mass_kg)
            and self._eta_l_estimated_mass_kg > 0.0
        ):
            below_alpha_gate = selected_airspeed < self._eta_l_min_airspeed
            if below_alpha_gate or (
                alpha_valid
                and all(math.isfinite(value) for value in (
                    alpha, gamma, roll,
                ))
            ):
                alpha_for_estimate = 0.0 if below_alpha_gate else alpha
                gamma_for_estimate = 0.0 if below_alpha_gate else gamma
                roll_for_estimate = 0.0 if below_alpha_gate else roll
                eta_l_cl = (
                    0.0 if below_alpha_gate else piecewise_lift_coefficient(
                        geometric_alpha_rad=alpha_for_estimate,
                        alpha_offset_rad=self._eta_l_alpha_offset,
                        lift_slope_per_rad=self._eta_l_lift_slope,
                        stall_angle_rad=self._eta_l_stall_angle,
                        post_stall_slope_per_rad=(
                            self._eta_l_post_stall_slope
                        ),
                    )
                )
                estimate = estimate_wing_vertical_support(
                    air_density_kg_m3=self._eta_l_density,
                    airspeed_mps=selected_airspeed,
                    wing_area_m2=self._eta_l_wing_area,
                    lift_coefficient=eta_l_cl,
                    flight_path_angle_rad=gamma_for_estimate,
                    roll_angle_rad=roll_for_estimate,
                    estimated_mass_kg=self._eta_l_estimated_mass_kg,
                    desired_vertical_acceleration_mps2=(
                        self._eta_l_desired_vertical_acceleration
                    ),
                    gravity_mps2=self._gravity,
                    minimum_valid_airspeed_mps=self._eta_l_min_airspeed,
                )
                eta_l_valid = True
                eta_l_dynamic_pressure = estimate.dynamic_pressure_pa
                wing_lift_est = estimate.wing_lift_n
                wing_vertical_support_est = estimate.vertical_support_n
                wing_required_support = estimate.required_support_n
                eta_l = estimate.eta_l
                if wing_lift_gt_valid:
                    eta_l_gt = min(
                        1.0,
                        max(
                            0.0,
                            wing_total_force[2]
                            / estimate.required_support_n,
                        ),
                    )
                if below_alpha_gate:
                    eta_l_force_model_invalid_reason = 'below_minimum_airspeed'
                elif not math.isfinite(body_u) or body_u <= self._eta_l_min_body_forward_airspeed:
                    eta_l_force_model_invalid_reason = 'non_forward_body_airflow'
                elif not beta_valid or abs(beta) > self._eta_l_max_abs_beta:
                    eta_l_force_model_invalid_reason = 'outside_sideslip_domain'
                elif not alpha_valid or not math.isfinite(alpha) or abs(alpha) > self._eta_l_max_abs_alpha:
                    eta_l_force_model_invalid_reason = 'outside_alpha_domain'
                else:
                    eta_l_force_model_valid = True
                    eta_l_force_model_invalid_reason = 'none'

        eta_c_valid = False
        eta_c_pressure_gate = eta_c_moment_margin = eta_c = math.nan
        eta_c_gt = math.nan
        cm_delta_e_est = math.nan
        shadow_target_angle = elevator_moment_derivative = math.nan
        shadow_target_moment = current_elevator_moment = math.nan
        shadow_increment_moment = shadow_increment_for_eta_c = math.nan
        eta_c_remaining_angle = math.nan
        eta_c_available_moment = math.nan
        eta_c_available_moment_gt = math.nan
        fw_demand_age = message_age('_fw_torque_setpoint')
        elevator_actual_age = message_age('_elevator_joint_position_gt')
        elevator_actual_angle = (
            _finite(self._elevator_joint_position_gt.data)
            if self._elevator_joint_position_gt is not None else math.nan
        )
        elevator_effective_age = message_age('_elevator_effective_angle_gt')
        elevator_effective_angle = (
            _finite(self._elevator_effective_angle_gt.data)
            if self._elevator_effective_angle_gt is not None else math.nan
        )
        elevator_actual_valid = bool(
            self._wing_force_gt_enabled
            and math.isfinite(elevator_actual_angle)
            and math.isfinite(elevator_actual_age)
            and elevator_actual_age <= self._wing_force_gt_timeout_s
        )
        elevator_effective_valid = bool(
            self._wing_force_gt_enabled
            and math.isfinite(elevator_effective_angle)
            and math.isfinite(elevator_effective_age)
            and elevator_effective_age <= self._wing_force_gt_timeout_s
        )
        elevator_control_angle = (
            elevator_effective_angle
            if elevator_effective_valid else elevator_actual_angle
        )
        elevator_control_angle_valid = (
            elevator_effective_valid or elevator_actual_valid
        )
        if (
            math.isfinite(eta_l_dynamic_pressure)
            and math.isfinite(fw_pitch_demand)
            and math.isfinite(fw_demand_age)
            and fw_demand_age <= 0.5
            and elevator_control_angle_valid
        ):
            clipped_elevator_angle = min(
                max(elevator_control_angle, self._elevator_joint_min),
                self._elevator_joint_max,
            )
            try:
                elevator_moment_derivative = (
                    eta_l_dynamic_pressure
                    * self._elevator_dmoment_ddelta_per_q
                )
                diagnostic_elevator_angle = clipped_elevator_angle
                shadow = shadow_pitch_moment_increment(
                    dynamic_pressure_pa=eta_l_dynamic_pressure,
                    dimensional_moment_derivative_per_q_m3=(
                        self._elevator_dmoment_ddelta_per_q
                    ),
                    normalized_shadow_pitch_demand=fw_pitch_demand,
                    current_elevator_angle_rad=diagnostic_elevator_angle,
                    elevator_min_rad=self._elevator_joint_min,
                    elevator_max_rad=self._elevator_joint_max,
                    command_radians_per_normalized=(
                        self._shadow_command_radians_per_normalized
                    ),
                )
                shadow_increment_raw = shadow.increment_pitch_moment_nm
                shadow_increment_for_eta_c = (
                    shadow.increment_pitch_moment_nm
                    + self._f3b2_shadow_demand_offset
                )
                if math.isfinite(self._f3b2_shadow_demand_for_eta_c):
                    shadow_increment_for_eta_c = (
                        self._f3b2_shadow_demand_for_eta_c
                    )
                if (
                    self._f3b2_test_mode == 'remaining_travel'
                    and self._f3b2_elevator_bias > 0.0
                    and math.isfinite(elevator_moment_derivative)
                    and abs(elevator_moment_derivative) > 1.0e-12
                    and math.isfinite(shadow_increment_for_eta_c)
                ):
                    angle_direction = (
                        shadow_increment_for_eta_c
                        / elevator_moment_derivative
                    )
                    if angle_direction > 0.0:
                        diagnostic_elevator_angle = min(
                            self._elevator_joint_max,
                            diagnostic_elevator_angle
                            + self._f3b2_elevator_bias,
                        )
                    elif angle_direction < 0.0:
                        diagnostic_elevator_angle = max(
                            self._elevator_joint_min,
                            diagnostic_elevator_angle
                            - self._f3b2_elevator_bias,
                        )
                authority = aerodynamic_control_authority_from_dimensional_derivative(
                    dynamic_pressure_pa=eta_l_dynamic_pressure,
                    dimensional_moment_derivative_per_q_m3=(
                        self._elevator_dmoment_ddelta_per_q
                    ),
                    elevator_angle_rad=diagnostic_elevator_angle,
                    elevator_min_rad=self._elevator_joint_min,
                    elevator_max_rad=self._elevator_joint_max,
                    shadow_pitch_moment_increment_nm=(
                        shadow_increment_for_eta_c
                    ),
                    pressure_gate_lower_pa=self._eta_c_pressure_gate_lower,
                    pressure_gate_upper_pa=self._eta_c_pressure_gate_upper,
                )
                gt_derivative_per_q = (
                    self._f3b2_gt_dmoment_ddelta_per_q
                    if math.isfinite(self._f3b2_gt_dmoment_ddelta_per_q)
                    else self._elevator_dmoment_ddelta_per_q
                )
                authority_gt = aerodynamic_control_authority_from_dimensional_derivative(
                    dynamic_pressure_pa=eta_l_dynamic_pressure,
                    dimensional_moment_derivative_per_q_m3=gt_derivative_per_q,
                    elevator_angle_rad=diagnostic_elevator_angle,
                    elevator_min_rad=self._elevator_joint_min,
                    elevator_max_rad=self._elevator_joint_max,
                    shadow_pitch_moment_increment_nm=(
                        shadow_increment_for_eta_c
                    ),
                    pressure_gate_lower_pa=self._eta_c_pressure_gate_lower,
                    pressure_gate_upper_pa=self._eta_c_pressure_gate_upper,
                )
            except ValueError:
                shadow = None
                authority = None
                authority_gt = None
            if shadow is not None and authority is not None:
                eta_c_valid = bool(
                    self._elevator_joint_min
                    <= diagnostic_elevator_angle
                    <= self._elevator_joint_max
                )
                shadow_target_angle = shadow.target_elevator_angle_rad
                shadow_target_moment = shadow.target_pitch_moment_nm
                current_elevator_moment = shadow.current_pitch_moment_nm
                shadow_increment_moment = shadow_increment_raw
                eta_c_remaining_angle = authority.remaining_elevator_angle_rad
                eta_c_available_moment = authority.available_increment_moment_nm
                if authority_gt is not None:
                    eta_c_available_moment_gt = (
                        authority_gt.available_increment_moment_nm
                    )
                    eta_c_gt = authority_gt.eta_c if eta_c_valid else 0.0
                eta_c_pressure_gate = authority.pressure_gate
                eta_c_moment_margin = authority.moment_margin
                eta_c = authority.eta_c if eta_c_valid else 0.0
                elevator_control_angle = diagnostic_elevator_angle
                if (
                    math.isfinite(self._eta_c_mean_chord)
                    and self._eta_l_wing_area > 0.0
                    and self._eta_c_mean_chord > 0.0
                ):
                    cm_delta_e_est = (
                        self._elevator_dmoment_ddelta_per_q
                        / (self._eta_l_wing_area * self._eta_c_mean_chord)
                    )

        # Physics-prior-only rows must be auditable as eta_L/eta_C ->
        # lambda_phy -> shielded lambda_command, so publish after the current
        # capability estimates have been resolved.
        if self._schedule_mode == 'physics_prior_only':
            self._publish_schedule(
                selected_airspeed,
                altitude,
                vz_up,
                eta_l=eta_l,
                eta_c=eta_c,
                alpha=alpha,
                alpha_valid=alpha_valid,
                time_s=time_s,
            )

        status = self._status
        vtol = self._vtol_status
        a0_rl_action_age = (
            self._a0_rl_action_age(time_s)
            if self._schedule_mode == 'a0_rl_external' else math.nan
        )
        a0_rl_action_stale = int(
            self._schedule_mode == 'a0_rl_external'
            and (
                not math.isfinite(a0_rl_action_age)
                or a0_rl_action_age > self._a0_rl_command_timeout_s
            )
        )
        if self._schedule_mode != 'a0_rl_external':
            a0_rl_control_phase = ''
        elif state != 'TRANSITION_FW':
            a0_rl_control_phase = 'inactive'
        elif a0_rl_action_stale:
            a0_rl_control_phase = 'pre_rl'
        else:
            a0_rl_control_phase = 'rl_active'
        a0_rl_handover_ready_dwell = (
            max(0.0, time_s - self._a0_rl_handover_ready_started_at)
            if self._schedule_mode == 'a0_rl_external'
            and self._a0_rl_handover_ready_started_at is not None
            and math.isfinite(time_s)
            else math.nan
        )
        row = {
            'time_s': time_s,
            # Schema 13 adds lambda-synchronized expected/actual PX4 attitude
            # weights on top of the schema-12 normalized controller-demand
            # and GZ elevator-limit diagnostics.  The fixed-window protocol
            # itself remains protocol v9.
            # Schema 14 adds wind-qualification metadata and the ENU
            # ground-minus-wind relative-air diagnostic.  It deliberately
            # does not change the frozen schema-13 transition allocator.
            # Schema 15 adds F2-A true-payload metadata.  Schema 16 adds the
            # independently acknowledged deterministic-gust command and the
            # frozen baseline/exposure/recovery phase labels for F2-B.
            # Schema 17 adds continuous eta_L/eta_C audit terms and the exact
            # lift vector applied by the optional instrumented main wings.
            # Schema 18 separates online eta_L availability from the stricter
            # longitudinal force-model validation mask and records body-axis
            # relative-air diagnostics.
            # Schema 19 adds explicit F3-B eta_C audit aliases while keeping
            # eta_C read-only: it does not alter PX4 blending.
            # Schema 20 adds optional elevator aerodynamic wrench / pitch
            # moment ground-truth telemetry for F3-B1 validation.  Schema 21
            # adds the instrumented elevator effective angle and a
            # measurement-window-only dither scale used when a dedicated
            # F3-B1 identification dither is enabled.
            # Schema 22 adds F3-B2 eta_C behavior-smoke diagnostic labels and
            # read-only shadow-demand / remaining-travel perturbations.
            # Schema 23 separates the raw shadow pitch-moment request from
            # the fixed request used by remaining-travel eta_C diagnostics.
            # Schema 24 adds Physics Prior Only trace fields: g_L, g_C,
            # lambda_phy, lambda_max_hard, projected lambda, and shield
            # intervention diagnostics.
            # Schema 25 adds A0 ROS/PX4 online-action metadata and publishes
            # the frozen 10-D A0 observation on /ca_lsc/a0_observation.
            # Schema 26 makes the A0 control phase explicit so the trainer can
            # keep the safe pre-RL transition bootstrap out of replay.
            # Schema 27 records the requested simulation-speed factor and uses
            # simulation time for A0 action freshness / success dwell.
            # Schema 28 moves the Python telemetry / A0 observation sampler from
            # wall time to ROS simulation time and records the resulting sample
            # dt in both time bases.
            # Schema 29 publishes explicit A0 terminal payloads for hard
            # early-aborts even when derived fields such as alpha are invalid.
            # Schema 30 repeats early-abort terminal payloads during a short
            # DDS delivery grace period before the recorder exits.
            # Schema 31 latches the first A0 terminal outcome: terminal
            # delivery grace can no longer overwrite a physical abort with a
            # synthetic action timeout after the trainer stops publishing.
            # Schema 32 separates handover-ready from actual success, which
            # now requires command-node HOLD_FW confirmation, and evaluates
            # all runaway dwell timers in simulation time.
            # Schema 33 records the source of every A0 action so deterministic
            # evaluation can prove that no warm-up/exploration path was used.
            # Schema 34 latches the command-node readiness snapshot at the
            # HOLD_MC -> TRANSITION_FW edge.  A0 admission must not depend on
            # live gates after acceleration has started.
            'schema_version': 34,
            'condition_id': self._condition_id,
            'wind_model_config': self._wind_model,
            'sim_speed_factor_config': self._sim_speed_factor,
            'a0_observation_sim_dt_s': a0_observation_sim_dt,
            'a0_observation_wall_dt_s': a0_observation_wall_dt,
            'schedule_mode': self._schedule_mode,
            'command_state': self._command_state,
            'a0_rl_action_seq': (
                self._a0_rl_action_seq
                if self._schedule_mode == 'a0_rl_external' else ''
            ),
            'a0_rl_action': (
                self._a0_rl_action
                if self._schedule_mode == 'a0_rl_external' else math.nan
            ),
            'a0_rl_lambda_d': (
                self._a0_rl_lambda_d
                if self._schedule_mode == 'a0_rl_external' else math.nan
            ),
            'a0_rl_action_source': (
                self._a0_rl_action_source
                if self._schedule_mode == 'a0_rl_external' else ''
            ),
            'a0_rl_action_age_s': a0_rl_action_age,
            'a0_rl_action_stale': a0_rl_action_stale,
            'a0_rl_control_phase': a0_rl_control_phase,
            'a0_rl_handover_ready': int(self._a0_rl_handover_ready),
            'a0_rl_handover_ready_dwell_s': a0_rl_handover_ready_dwell,
            'a0_rl_actual_fw_confirmed': int(
                state == 'HOLD_FW'
                and self._a0_rl_terminal_reason == 'success'
            ),
            'a0_rl_initial_readiness_available': int(
                self._a0_rl_initial_readiness_available
            ),
            'a0_rl_initial_readiness_latched': int(
                self._a0_rl_initial_readiness_latched
            ),
            'a0_rl_initial_height_gate_ok': int(
                self._a0_rl_initial_height_gate_ok
            ),
            'a0_rl_initial_vz_gate_ok': int(
                self._a0_rl_initial_vz_gate_ok
            ),
            'a0_rl_initial_groundspeed_gate_ok': int(
                self._a0_rl_initial_groundspeed_gate_ok
            ),
            'a0_rl_terminal_reason': self._a0_rl_terminal_reason,
            'f3b2_test_mode': self._f3b2_test_mode,
            'f3b2_shadow_demand_offset_nm': (
                self._f3b2_shadow_demand_offset
            ),
            'f3b2_shadow_demand_for_eta_c_nm': (
                self._f3b2_shadow_demand_for_eta_c
            ),
            'f3b2_elevator_bias_rad': self._f3b2_elevator_bias,
            'f3b2_stage_index': self._f3b2_stage_index,
            'vtol_state': int(vtol.vehicle_vtol_state) if vtol is not None else '',
            'nav_state': int(status.nav_state) if status is not None else '',
            'armed': int(status.arming_state == VehicleStatus.ARMING_STATE_ARMED) if status is not None else '',
            'failsafe': int(status.failsafe) if status is not None else '',
            'north_m': north,
            'east_m': east,
            'altitude_local_m': altitude_local,
            'altitude_datum_local_m': altitude_datum_local,
            'altitude_relative_m': altitude,
            'target_altitude_local_m': target_altitude_local,
            'target_altitude_relative_m': self._target_altitude,
            'altitude_m': altitude,
            'altitude_error_m': altitude - self._target_altitude if math.isfinite(altitude) else math.nan,
            'trajectory_sp_altitude_local_m': trajectory_sp_altitude_local,
            'trajectory_sp_altitude_relative_m': trajectory_sp_altitude_relative,
            'trajectory_sp_vz_up_mps': trajectory_sp_vz_up,
            'height_gate_ok': int(height_gate_ok),
            'vz_gate_ok': int(vz_gate_ok),
            'groundspeed_gate_ok': int(groundspeed_gate_ok),
            'stability_gate_ok': int(stability_gate_ok),
            'stability_dwell_s': stability_dwell_s,
            'va_target_config': (
                self._va_target
                if self._schedule_mode in {
                    'va_hold_target', 'gust_target', 'physics_prior_only',
                    'a0_rl_external',
                }
                else math.nan
            ),
            'va_target_tolerance': (
                self._va_target_tolerance
                if self._schedule_mode in {
                    'va_hold_target', 'gust_target', 'physics_prior_only',
                    'a0_rl_external',
                }
                else math.nan
            ),
            'va_error_mps': (
                selected_airspeed - self._va_target
                if self._schedule_mode in {
                    'va_hold_target', 'gust_target', 'physics_prior_only',
                    'a0_rl_external',
                }
                and math.isfinite(selected_airspeed)
                else math.nan
            ),
            'va_measurement_min_band_fraction_config': (
                self._va_measurement_min_band_fraction
                if self._schedule_mode in {'va_hold_target', 'gust_target'}
                else math.nan
            ),
            'va_measurement_max_abs_airspeed_error_mps_config': (
                self._va_measurement_max_abs_airspeed_error
                if self._schedule_mode in {'va_hold_target', 'gust_target'}
                else math.nan
            ),
            'va_target_dwell_s_config': (
                self._va_target_dwell_s
                if self._schedule_mode in {'va_hold_target', 'gust_target'}
                else math.nan
            ),
            'lambda_target_dwell_s_config': (
                self._lambda_target_dwell_s
                if self._schedule_mode in {'va_hold_target', 'gust_target'}
                else math.nan
            ),
            'va_measurement_duration_s_config': (
                self._va_measurement_s
                if self._schedule_mode == 'va_hold_target'
                else math.nan
            ),
            'vt_unload_altitude_pitch_kp_config': (
                self._vt_unload_altitude_pitch_kp
            ),
            'vt_unload_vertical_speed_pitch_kd_config': (
                self._vt_unload_vertical_speed_pitch_kd
            ),
            'vt_unload_pitch_min_deg_config': self._vt_unload_pitch_min_deg,
            'vt_unload_pitch_max_deg_config': self._vt_unload_pitch_max_deg,
            'vt_airspeed_blend_mps_config': self._vt_airspeed_blend,
            'vt_transition_airspeed_mps_config': (
                self._vt_transition_airspeed
            ),
            'lambda_attitude_blend_start_config': (
                self._lambda_attitude_blend_start
            ),
            'lambda_attitude_blend_full_config': (
                self._lambda_attitude_blend_full
            ),
            'mc_pitch_weight_proxy': transition_mc_weight_proxy(
                airspeed,
                self._vt_airspeed_blend,
                self._vt_transition_airspeed,
            ),
            'mc_pitch_weight_actual': mc_pitch_weight,
            'fw_pitch_weight_actual': fw_pitch_weight,
            'mc_pitch_weight_lambda_expected': (
                transition_attitude_weights(
                    lambda_exec,
                    self._lambda_attitude_blend_start,
                    self._lambda_attitude_blend_full,
                )[0]
                if math.isfinite(lambda_exec) else math.nan
            ),
            'fw_pitch_weight_lambda_expected': (
                transition_attitude_weights(
                    lambda_exec,
                    self._lambda_attitude_blend_start,
                    self._lambda_attitude_blend_full,
                )[1]
                if math.isfinite(lambda_exec) else math.nan
            ),
            'mc_pitch_torque_demand_normalized': mc_pitch_demand,
            'fw_pitch_torque_demand_normalized': fw_pitch_demand,
            'mc_pitch_torque_contribution_normalized': (
                mc_pitch_demand * mc_pitch_weight
                if math.isfinite(mc_pitch_demand)
                and math.isfinite(mc_pitch_weight) else math.nan
            ),
            'fw_pitch_torque_contribution_normalized': (
                fw_pitch_demand * fw_pitch_weight
                if math.isfinite(fw_pitch_demand)
                and math.isfinite(fw_pitch_weight) else math.nan
            ),
            'va_hold_velocity_command_mps': self._va_velocity_command,
            'va_hold_down_velocity_command_mps': (
                self._va_down_velocity_command
            ),
            'va_hold_phase': self._va_hold_phase,
            'va_hold_measurement_active': int(
                self._schedule_mode == 'va_hold_target'
                and self._va_hold_phase == 'measurement'
                and self._va_measurement_started_at is not None
                and not self._va_measurement_complete
            ),
            'va_hold_measurement_complete': int(
                self._va_measurement_complete
            ),
            'va_hold_measurement_interrupted': int(
                self._va_measurement_interrupted
            ),
            'va_hold_abort_reason': self._va_abort_reason,
            'va_vertical_abort_enabled': int(va_vertical_abort_enabled),
            'va_low_speed_vertical_transient': int(
                va_low_speed_vertical_transient
            ),
            'vn_mps': vn,
            've_mps': ve,
            'vz_up_mps': vz_up,
            'groundspeed_mps': groundspeed,
            'airspeed_mps': airspeed,
            'airspeed_source_config': self._airspeed_source,
            'selected_airspeed_mps': selected_airspeed,
            'selected_airspeed_error_mps': selected_airspeed_error,
            'wind_cmd_e_enu_mps': self._wind_cmd_e,
            'wind_cmd_n_enu_mps': self._wind_cmd_n,
            'wind_cmd_u_enu_mps': self._wind_cmd_u,
            'wind_actual_e_enu_mps': wind_e,
            'wind_actual_n_enu_mps': wind_n,
            'wind_actual_u_enu_mps': wind_u,
            'wind_heading_parallel_mps': wind_heading_parallel,
            'wind_heading_cross_mps': wind_heading_cross,
            'wind_actual_source': (
                'gz_transport_bridge_ack'
                if self._schedule_mode == 'gust_target' and
                math.isfinite(gust_status_age)
                and gust_status_age <= self._gust_status_timeout_s
                else 'bridge_stale'
                if self._schedule_mode == 'gust_target'
                else 'configured_steady_world'
                if self._wind_model != 'none' else 'none'
            ),
            'gust_amplitude_mps_config': (
                self._gust_amplitude
                if self._schedule_mode == 'gust_target' else math.nan
            ),
            'gust_delay_s_config': (
                self._gust_delay_s
                if self._schedule_mode == 'gust_target' else math.nan
            ),
            'gust_duration_s_config': (
                self._gust_duration_s
                if self._schedule_mode == 'gust_target' else math.nan
            ),
            'gust_recovery_s_config': (
                self._gust_recovery_s
                if self._schedule_mode == 'gust_target' else math.nan
            ),
            'gust_phase': (
                self._va_hold_phase
                if self._schedule_mode == 'gust_target' else 'inactive'
            ),
            'gust_phase_elapsed_s': (
                self._gust_phase_elapsed_s
                if self._schedule_mode == 'gust_target' else math.nan
            ),
            'gust_profile_scale': (
                self._gust_profile_scale
                if self._schedule_mode == 'gust_target' else math.nan
            ),
            'gust_active': int(
                self._schedule_mode == 'gust_target'
                and self._va_hold_phase == 'gust_exposure'
            ),
            'gust_bridge_ready': int(
                self._schedule_mode == 'gust_target'
                and math.isfinite(gust_status_age)
                and gust_status_age <= self._gust_status_timeout_s
            ),
            'gust_status_age_s': gust_status_age,
            'vg_e_enu_mps': ve,
            'vg_n_enu_mps': vn,
            'vg_u_enu_mps': vz_up,
            'vrel_e_enu_mps': vrel_e,
            'vrel_n_enu_mps': vrel_n,
            'vrel_u_enu_mps': vrel_u,
            'vrel_norm_mps': vrel_norm,
            'vrel_body_u_mps': body_u,
            'vrel_body_v_mps': body_v,
            'vrel_body_w_mps': body_w,
            'beta_est_rad': beta,
            'beta_valid': int(beta_valid),
            'airspeed_relative_error_mps': airspeed_relative_error,
            'mass_scale_config': self._mass_scale,
            'nominal_model_mass_kg_config': self._nominal_model_mass_kg,
            'actual_model_mass_kg_config': self._actual_model_mass_kg,
            'payload_mass_kg_config': self._payload_mass_kg,
            'wing_lift_gt_enabled': int(self._wing_force_gt_enabled),
            'wing_lift_gt_valid': int(wing_lift_gt_valid),
            'wing_lift_left_fx_enu_n': wing_left_force[0],
            'wing_lift_left_fy_enu_n': wing_left_force[1],
            'wing_lift_left_fz_enu_n': wing_left_force[2],
            'wing_lift_right_fx_enu_n': wing_right_force[0],
            'wing_lift_right_fy_enu_n': wing_right_force[1],
            'wing_lift_right_fz_enu_n': wing_right_force[2],
            'wing_lift_total_fx_enu_n': wing_total_force[0],
            'wing_lift_total_fy_enu_n': wing_total_force[1],
            'wing_lift_total_fz_enu_n': wing_total_force[2],
            'wing_lift_gt_vertical_enu_n': wing_total_force[2],
            'wing_lift_gt_age_s': wing_lift_gt_age,
            'elevator_aero_wrench_gt_valid': int(
                elevator_aero_wrench_valid),
            'elevator_aero_fx_enu_n': elevator_aero_wrench[0],
            'elevator_aero_fy_enu_n': elevator_aero_wrench[1],
            'elevator_aero_fz_enu_n': elevator_aero_wrench[2],
            'elevator_aero_mx_enu_nm': elevator_aero_wrench[3],
            'elevator_aero_my_enu_nm': elevator_aero_wrench[4],
            'elevator_aero_mz_enu_nm': elevator_aero_wrench[5],
            'elevator_pitch_moment_gt_nm': elevator_pitch_moment_gt,
            'elevator_aero_wrench_gt_age_s': elevator_aero_age,
            'roll_rad': roll,
            'pitch_rad': pitch,
            'yaw_rad': yaw,
            'roll_setpoint_rad': roll_setpoint,
            'pitch_setpoint_rad': pitch_setpoint,
            'yaw_setpoint_rad': yaw_setpoint,
            'p_rad_s': rates[0],
            'q_rad_s': rates[1],
            'r_rad_s': rates[2],
            'alpha_ground_est_raw_rad': raw_alpha,
            'alpha_est_rad': alpha,
            'alpha_valid': int(alpha_valid),
            'gamma_est_rad': gamma,
            'eta_l_valid': int(eta_l_valid),
            'eta_l_available': int(eta_l_valid),
            'eta_l_force_model_valid': int(eta_l_force_model_valid),
            'eta_l_force_model_invalid_reason': (
                eta_l_force_model_invalid_reason
            ),
            'eta_l_min_body_forward_airspeed_mps_config': (
                self._eta_l_min_body_forward_airspeed
            ),
            'eta_l_max_abs_beta_rad_config': self._eta_l_max_abs_beta,
            'eta_l_max_abs_alpha_rad_config': self._eta_l_max_abs_alpha,
            'eta_l_mass_source': self._eta_l_mass_source,
            'eta_l_estimated_mass_kg': self._eta_l_estimated_mass_kg,
            'eta_l_desired_vertical_acceleration_mps2': (
                self._eta_l_desired_vertical_acceleration
            ),
            'eta_l_dynamic_pressure_pa': eta_l_dynamic_pressure,
            'eta_l_cl_est': eta_l_cl,
            'wing_lift_est_n': wing_lift_est,
            'wing_vertical_support_est_n': wing_vertical_support_est,
            'wing_required_support_est_n': wing_required_support,
            'eta_l': eta_l,
            'eta_l_gt': eta_l_gt,
            'eta_c_valid': int(eta_c_valid),
            'eta_c_mapping_source': (
                'px4_virtual_fw_normalized_demand_and_gz_actual_elevator_'
                'joint_position_with_sdf_dimensional_moment_derivative'
            ),
            'eta_c_dynamic_pressure_pa': eta_l_dynamic_pressure,
            'eta_c_pressure_gate_lower_pa_config': (
                self._eta_c_pressure_gate_lower
            ),
            'eta_c_pressure_gate_upper_pa_config': (
                self._eta_c_pressure_gate_upper
            ),
            'qbar_selected_pa': eta_l_dynamic_pressure,
            'cm_delta_e_est': cm_delta_e_est,
            'elevator_dmoment_ddelta_per_q_m3_config': (
                self._elevator_dmoment_ddelta_per_q
            ),
            'shadow_fw_pitch_demand_normalized': fw_pitch_demand,
            'shadow_fw_pitch_moment_req_nm': shadow_increment_moment,
            'shadow_fw_pitch_moment_req_raw_nm': shadow_increment_moment,
            'shadow_fw_pitch_moment_req_for_eta_c_nm': (
                shadow_increment_for_eta_c
            ),
            'shadow_target_elevator_angle_rad': shadow_target_angle,
            'elevator_actual_angle_rad': elevator_actual_angle,
            'elevator_actual_angle_gt_rad': elevator_actual_angle,
            'elevator_actual_angle_gt_valid': int(elevator_actual_valid),
            'elevator_actual_angle_gt_age_s': elevator_actual_age,
            'elevator_effective_angle_gt_rad': elevator_effective_angle,
            'elevator_effective_angle_gt_valid': int(
                elevator_effective_valid),
            'elevator_effective_angle_gt_age_s': elevator_effective_age,
            'elevator_control_angle_for_eta_c_rad': elevator_control_angle,
            'elevator_id_dither_scale_command': (
                self._elevator_id_dither_scale_command
            ),
            'elevator_min_rad_config': self._elevator_joint_min,
            'elevator_max_rad_config': self._elevator_joint_max,
            'elevator_moment_derivative_nm_per_rad': (
                elevator_moment_derivative
            ),
            'shadow_target_pitch_moment_nm': shadow_target_moment,
            'current_elevator_pitch_moment_nm': current_elevator_moment,
            'shadow_pitch_moment_increment_nm': shadow_increment_moment,
            'elevator_remaining_directional_rad': eta_c_remaining_angle,
            'eta_c_remaining_elevator_angle_rad': eta_c_remaining_angle,
            'moment_available_est_nm': eta_c_available_moment,
            'moment_available_gt_nm': eta_c_available_moment_gt,
            'eta_c_available_increment_moment_nm': eta_c_available_moment,
            'eta_c_raw': eta_c_moment_margin,
            'eta_c_q_gate': eta_c_pressure_gate,
            'eta_c_pressure_gate': eta_c_pressure_gate,
            'eta_c_gt': eta_c_gt,
            'eta_c_moment_margin': eta_c_moment_margin,
            'eta_c': eta_c,
            'eta_c_modifies_px4_blending': 0,
            'physics_prior_active': int(
                self._schedule_mode == 'physics_prior_only'
            ),
            'physics_prior_eta_l_lower_config': (
                self._physics_prior_eta_l_lower
            ),
            'physics_prior_eta_l_upper_config': (
                self._physics_prior_eta_l_upper
            ),
            'physics_prior_eta_c_lower_config': (
                self._physics_prior_eta_c_lower
            ),
            'physics_prior_eta_c_upper_config': (
                self._physics_prior_eta_c_upper
            ),
            'physics_prior_release_lambda_config': (
                self._physics_prior_release_lambda
            ),
            'physics_prior_release_dwell_s_config': (
                self._physics_prior_release_dwell_s
            ),
            'physics_prior_g_l': self._physics_prior_g_l,
            'physics_prior_g_c': self._physics_prior_g_c,
            'lambda_phy': self._lambda_phy,
            'lambda_max_hard': self._lambda_max_hard,
            'lambda_projected': self._lambda_projected,
            'lambda_shield_projection_intervention': (
                self._lambda_shield_projection_intervention
            ),
            'lambda_shield_total_intervention': (
                self._lambda_shield_total_intervention
            ),
            'lambda_shield_emergency_recovery': int(
                self._lambda_shield_emergency_recovery
            ),
            'lambda_command': self._lambda_command,
            'lambda_exec': lambda_exec,
            'lambda_external_active': (
                _finite(self._lambda_active_status.normalized_setpoint)
                if self._lambda_active_status is not None else math.nan
            ),
            'lambda_target_config': (
                self._lambda_target
                if self._schedule_mode in {
                    'manual_target', 'va_hold_target', 'gust_target'
                }
                else math.nan
            ),
            'lambda_target_tolerance': (
                self._lambda_target_tolerance
                if self._schedule_mode in {
                    'manual_target', 'va_hold_target', 'gust_target'
                }
                else math.nan
            ),
            'lambda_characterization_complete': int(
                self._lambda_characterization_complete
            ),
            'pusher_throttle_command': self._pusher_throttle_command,
            'pusher_throttle_status': pusher_throttle_status,
            'pusher_throttle_external_active': pusher_throttle_active,
            'airspeed_filtered_mps': self._va_filtered,
            'pusher_airspeed_error_filtered_mps': (
                self._pusher_airspeed_error_filtered
            ),
            'pusher_pi_integral_mps_s': self._pusher_pi_integral,
            'pusher_throttle_unsaturated': self._pusher_throttle_unsaturated,
            'servo_0': servos[0],
            'servo_1': servos[1],
            'servo_2': servos[2],
            'elevator_angle_proxy_rad': elevator_angle_proxy,
            'elevator_joint_limit_commanded': elevator_joint_limit_commanded,
            'motor_control_0': motor_controls[0],
            'motor_control_1': motor_controls[1],
            'motor_control_2': motor_controls[2],
            'motor_control_3': motor_controls[3],
            'motor_control_4': motor_controls[4],
            'pusher_thrust_setpoint': pusher_thrust_setpoint,
            'lift_collective_thrust_setpoint': (
                lift_collective_thrust_setpoint
            ),
            'omega_0_rad_s': omega[0],
            'omega_1_rad_s': omega[1],
            'omega_2_rad_s': omega[2],
            'omega_3_rad_s': omega[3],
            'omega_4_rad_s': omega[4],
            'lift_thrust_n': lift_thrust,
            'pusher_thrust_n': pusher_thrust,
            'lift_power_w': lift_power,
            'pusher_power_w': pusher_power,
            'battery_voltage_v': battery_voltage,
            'battery_current_a': battery_current,
            'battery_power_w': battery_power,
            'position_age_s': message_age('_position'),
            'attitude_age_s': message_age('_attitude'),
            'attitude_setpoint_age_s': message_age('_attitude_setpoint'),
            'angular_velocity_age_s': message_age('_angular_velocity'),
            'airspeed_age_s': message_age('_airspeed'),
            'vehicle_status_age_s': message_age('_status'),
            'vtol_status_age_s': message_age('_vtol_status'),
            'esc_age_s': message_age('_esc'),
            'servos_age_s': message_age('_servos'),
            'lambda_status_age_s': message_age('_lambda_status'),
            'lambda_active_age_s': message_age('_lambda_active_status'),
            'pusher_throttle_status_age_s': message_age('_pusher_throttle_status'),
            'pusher_throttle_active_age_s': message_age('_pusher_throttle_active'),
            'actuator_motors_age_s': message_age('_actuator_motors'),
            'mc_pitch_weight_age_s': message_age('_mc_pitch_weight_status'),
            'fw_pitch_weight_age_s': message_age('_fw_pitch_weight_status'),
            'mc_torque_setpoint_age_s': message_age('_mc_torque_setpoint'),
            'fw_torque_setpoint_age_s': message_age('_fw_torque_setpoint'),
            'thrust_setpoint_age_s': message_age('_thrust_setpoint'),
            'trajectory_setpoint_age_s': message_age('_trajectory_setpoint'),
            'wing_lift_left_gt_age_s': wing_left_age,
            'wing_lift_right_gt_age_s': wing_right_age,
        }
        self._writer.writerow(row)
        self._publish_a0_observation(row)
        self._rows += 1
        if self._rows % 20 == 0:
            self._stream.flush()
        if va_abort_now:
            # The terminal observation was published just above.  Keep the
            # producer alive briefly so DDS can deliver it; immediate process
            # exit occasionally left the trainer with a synthetic "running"
            # episode even though telemetry contained the abort reason.
            if self._early_abort_wall_time is None:
                self._early_abort_wall_time = time.monotonic()
                self.get_logger().error(
                    f'Phase-0.75 aborted early: {self._va_abort_reason}'
                )
            if time.monotonic() - self._early_abort_wall_time >= 0.15:
                self.close()
                raise SystemExit(0)
            return
        terminal_done = (
            self._exit_on_terminal
            and self._terminal_wall_time is not None
            and time.monotonic() - self._terminal_wall_time >= 2.0
        )
        forward_hold_done = (
            self._stop_after_fw_hold_s > 0.0
            and self._fw_hold_wall_time is not None
            and time.monotonic() - self._fw_hold_wall_time
            >= self._stop_after_fw_hold_s
        )
        if terminal_done or forward_hold_done:
            self.get_logger().info('required transition segment recorded; stopping test')
            self.close()
            raise SystemExit(0)

    def close(self) -> None:
        if not self._stream.closed:
            self._stream.flush()
            self._stream.close()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = TransitionExperiment()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
