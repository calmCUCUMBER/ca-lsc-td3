import csv
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path


ROS_PACKAGE = (
    Path(__file__).resolve().parents[1]
    / 'ros2_ws'
    / 'src'
    / 'ca_lsc_transition'
)
sys.path.insert(0, str(ROS_PACKAGE))

from ca_lsc_transition.diagnostics import (  # noqa: E402
    a0_transition_terminal_reason,
    gated_angle_of_attack,
    sustained_sim_time,
    transition_start_readiness,
    transition_mc_weight_proxy,
    within_lambda_target,
    wind_relative_velocity_enu,
)
from ca_lsc_transition.evaluate_run import evaluate  # noqa: E402
from ca_lsc_transition.schedule import (  # noqa: E402
    airspeed_unloading,
    altitude_hold_down_velocity,
    smoothstep,
    transition_attitude_weights,
)


FIELDS = [
    'time_s', 'schema_version', 'schedule_mode', 'command_state',
    'vtol_state', 'failsafe', 'altitude_m', 'altitude_error_m',
    'altitude_local_m', 'altitude_datum_local_m', 'altitude_relative_m',
    'target_altitude_local_m', 'target_altitude_relative_m',
    'trajectory_sp_altitude_local_m', 'trajectory_sp_altitude_relative_m',
    'trajectory_sp_vz_up_mps',
    'height_gate_ok', 'vz_gate_ok', 'groundspeed_gate_ok',
    'stability_gate_ok', 'stability_dwell_s',
    'va_target_config', 'va_target_tolerance', 'va_error_mps',
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
    'va_hold_abort_reason', 'va_vertical_abort_enabled',
    'va_low_speed_vertical_transient',
    'vz_up_mps', 'groundspeed_mps', 'airspeed_mps', 'pitch_rad',
    'q_rad_s', 'alpha_valid', 'lambda_command', 'lambda_exec',
    'lambda_external_active', 'lambda_target_config',
    'lambda_target_tolerance', 'lambda_characterization_complete',
    'pusher_throttle_command', 'pusher_throttle_status',
    'pusher_throttle_external_active',
    'servo_0', 'servo_1', 'servo_2',
    'elevator_angle_proxy_rad', 'elevator_joint_limit_commanded',
    'motor_control_0', 'motor_control_1', 'motor_control_2',
    'motor_control_3', 'motor_control_4',
    'pusher_thrust_setpoint', 'lift_collective_thrust_setpoint',
    'omega_0_rad_s', 'omega_1_rad_s', 'omega_2_rad_s',
    'omega_3_rad_s', 'omega_4_rad_s', 'lift_power_w',
    'pusher_power_w', 'position_age_s', 'attitude_age_s',
    'angular_velocity_age_s', 'airspeed_age_s', 'esc_age_s',
    'servos_age_s', 'lambda_status_age_s', 'lambda_active_age_s',
    'pusher_throttle_status_age_s', 'pusher_throttle_active_age_s',
    'actuator_motors_age_s', 'mc_pitch_weight_age_s',
    'fw_pitch_weight_age_s', 'mc_torque_setpoint_age_s',
    'fw_torque_setpoint_age_s', 'thrust_setpoint_age_s',
    'trajectory_setpoint_age_s',
]


class ScheduleTests(unittest.TestCase):
    def test_smoothstep_limits_and_midpoint(self):
        self.assertEqual(smoothstep(4.0, 8.0, 16.0), 0.0)
        self.assertAlmostEqual(smoothstep(12.0, 8.0, 16.0), 0.5)
        self.assertEqual(smoothstep(20.0, 8.0, 16.0), 1.0)

    def test_airspeed_schedule_is_bounded(self):
        values = [
            airspeed_unloading(speed, 8.0, 16.0, 0.6)
            for speed in range(0, 25)
        ]
        self.assertTrue(all(0.0 <= value <= 0.6 for value in values))
        self.assertTrue(all(
            left <= right for left, right in zip(values, values[1:])
        ))
        self.assertEqual(
            airspeed_unloading(math.nan, 8.0, 16.0, 0.6), 0.0
        )

    def test_transition_attitude_weights_follow_lambda_allocation(self):
        self.assertEqual(transition_attitude_weights(0.0), (1.0, 0.0))
        self.assertEqual(transition_attitude_weights(0.1), (1.0, 0.0))
        self.assertEqual(transition_attitude_weights(0.5), (0.5, 0.5))
        self.assertEqual(transition_attitude_weights(0.9), (0.0, 1.0))
        self.assertEqual(transition_attitude_weights(1.0), (0.0, 1.0))
        with self.assertRaises(ValueError):
            transition_attitude_weights(0.5, 0.9, 0.1)

    def test_altitude_hold_commands_down_for_climb_only(self):
        self.assertAlmostEqual(
            altitude_hold_down_velocity(52.0, 50.0, 0.4, 0.35, 0.5, 1.5),
            0.9,
        )
        self.assertEqual(
            altitude_hold_down_velocity(48.0, 50.0, -0.4, 0.35, 0.5, 1.5),
            0.0,
        )

    def test_altitude_hold_is_bounded_and_safe_on_missing_state(self):
        self.assertEqual(
            altitude_hold_down_velocity(60.0, 50.0, 4.0, 0.35, 0.5, 1.5),
            1.5,
        )
        self.assertEqual(
            altitude_hold_down_velocity(math.nan, 50.0, 1.0, 0.35, 0.5, 1.5),
            0.0,
        )


class DiagnosticHelperTests(unittest.TestCase):
    def test_a0_success_requires_confirmed_hold_fw(self):
        self.assertEqual(
            a0_transition_terminal_reason(
                'TRANSITION_FW', 'TRANSITION_FW', '', ''
            ),
            '',
        )
        self.assertEqual(
            a0_transition_terminal_reason(
                'TRANSITION_FW', 'HOLD_FW', '', ''
            ),
            'success',
        )
        self.assertEqual(
            a0_transition_terminal_reason(
                'TRANSITION_FW',
                'TRANSITION_MC',
                'forward transition timed out',
                '',
            ),
            'forward_transition_timeout',
        )
        self.assertEqual(
            a0_transition_terminal_reason(
                'TRANSITION_FW',
                'HOLD_FW',
                '',
                'va_hold_vertical_speed_runaway',
            ),
            'va_hold_vertical_speed_runaway',
        )

    def test_runaway_dwell_uses_simulation_time_and_handles_rollback(self):
        started, sustained = sustained_sim_time(True, None, 10.0, 0.5)
        self.assertEqual(started, 10.0)
        self.assertFalse(sustained)
        started, sustained = sustained_sim_time(True, started, 10.49, 0.5)
        self.assertFalse(sustained)
        started, sustained = sustained_sim_time(True, started, 10.5, 0.5)
        self.assertTrue(sustained)
        started, sustained = sustained_sim_time(True, started, 1.0, 0.5)
        self.assertEqual(started, 1.0)
        self.assertFalse(sustained)
        self.assertEqual(
            sustained_sim_time(False, started, 1.2, 0.5),
            (None, False),
        )

    def test_transition_mc_weight_proxy_exposes_authority_handover(self):
        self.assertEqual(transition_mc_weight_proxy(6.0, 8.0, 13.0), 1.0)
        self.assertAlmostEqual(
            transition_mc_weight_proxy(10.0, 8.0, 13.0), 0.6
        )
        self.assertEqual(transition_mc_weight_proxy(10.0, 8.0, 10.0), 0.0)
        self.assertEqual(transition_mc_weight_proxy(14.0, 8.0, 13.0), 0.0)
        self.assertTrue(math.isnan(
            transition_mc_weight_proxy(math.nan, 8.0, 13.0)
        ))

    def test_transition_mc_weight_proxy_rejects_reversed_thresholds(self):
        with self.assertRaises(ValueError):
            transition_mc_weight_proxy(10.0, 13.0, 8.0)

    def test_angle_of_attack_is_invalid_below_speed_gate(self):
        alpha, valid = gated_angle_of_attack(2.5, 4.99, 5.0)
        self.assertFalse(valid)
        self.assertTrue(math.isnan(alpha))
        alpha, valid = gated_angle_of_attack(0.1, 5.0, 5.0)
        self.assertTrue(valid)
        self.assertAlmostEqual(alpha, 0.1)

    def test_wind_relative_velocity_uses_enu_ground_minus_wind(self):
        # PX4 local velocity is logged as north/east/up in the recorder, while
        # Gazebo wind is configured as ENU.  A vehicle moving +10 m/s East in
        # a +4 m/s East wind has 6 m/s air-relative speed.
        east, north, up, norm = wind_relative_velocity_enu(
            north_mps=0.0,
            east_mps=10.0,
            up_mps=0.0,
            wind_east_mps=4.0,
            wind_north_mps=0.0,
            wind_up_mps=0.0,
        )
        self.assertEqual((east, north, up, norm), (6.0, 0.0, 0.0, 6.0))

        east, north, up, norm = wind_relative_velocity_enu(
            north_mps=3.0,
            east_mps=4.0,
            up_mps=1.0,
            wind_east_mps=1.0,
            wind_north_mps=-1.0,
            wind_up_mps=1.0,
        )
        self.assertEqual((east, north, up), (3.0, 4.0, 0.0))
        self.assertEqual(norm, 5.0)

    def test_lambda_target_band_validates_inputs(self):
        self.assertTrue(within_lambda_target(0.48, 0.5, 0.03))
        self.assertFalse(within_lambda_target(0.45, 0.5, 0.03))
        with self.assertRaises(ValueError):
            within_lambda_target(0.5, 1.1, 0.03)


class EvaluationTests(unittest.TestCase):
    @staticmethod
    def _rows(
        *,
        mode='manual_airspeed',
        stale=False,
        stable_hold=True,
        pusher_peak=0.45,
        target=0.5,
    ):
        rows = []
        for index in range(171):
            time_s = index * 0.1
            if index < 30:
                state = 'HOLD_MC'
            elif index < 100:
                state = 'TRANSITION_FW'
            else:
                state = 'HOLD_FW'
            transition_index = max(0, index - 30)
            airspeed = (
                min(14.0, transition_index * 0.2)
                if state == 'TRANSITION_FW'
                else 0.0 if state == 'HOLD_MC' else 14.0
            )
            altitude = 50.0
            if state == 'HOLD_MC' and not stable_hold and index < 20:
                altitude = 48.5
            if state == 'TRANSITION_FW':
                altitude = max(49.5, 50.0 - transition_index * 0.01)
            command = ''
            active = 0.0
            executed = 0.0 if state != 'HOLD_FW' else 1.0
            target_config = ''
            target_tolerance = ''
            complete = 0
            va_phase = ''
            va_measurement_active = 0
            va_measurement_complete = 0
            va_target_config = ''
            va_target_tolerance = ''
            va_error = ''
            va_velocity_command = ''
            pusher_throttle_command = ''
            pusher_throttle_status = ''
            pusher_throttle_active = ''
            if mode == 'manual_airspeed' and state == 'TRANSITION_FW' and airspeed > 8.0:
                command = 0.3
                executed = 0.25
                active = 1.0
            elif mode == 'manual_target':
                target_config = target
                target_tolerance = 0.03
                if state == 'TRANSITION_FW' and airspeed > 8.0:
                    command = target
                    executed = target
                    active = 1.0
                if state == 'HOLD_FW':
                    complete = 1
            elif mode == 'va_hold_target':
                target_config = target
                target_tolerance = 0.03
                va_target_config = 12.0
                va_target_tolerance = 0.3
                if state == 'TRANSITION_FW':
                    if index < 50:
                        va_phase = 'airspeed_settle'
                        airspeed = 0.4 * max(0, index - 30)
                    elif index < 80:
                        va_phase = 'measurement'
                        va_measurement_active = 1
                        va_measurement_complete = 0
                        airspeed = 12.0
                        command = target
                        executed = target
                        active = 1.0
                        pusher_throttle_command = 0.24
                        pusher_throttle_status = 0.24
                        pusher_throttle_active = 1.0
                    else:
                        # Regression trap: a legacy recorder kept
                        # measurement_active true after completion.  These
                        # rows are the PX4 release tail and must not be folded
                        # into fixed-Va measurement statistics.
                        va_phase = 'measurement'
                        va_measurement_active = 1
                        va_measurement_complete = 1
                        airspeed = 16.0
                        command = ''
                        executed = 1.0
                        active = 0.0
                        pusher_throttle_command = ''
                        pusher_throttle_status = 0.45
                        pusher_throttle_active = 0.0
                    va_error = airspeed - 12.0
                    va_velocity_command = 12.0
                if state == 'HOLD_FW':
                    va_measurement_complete = 1
            age = 10.0 if stale else 0.01
            row = {
                'time_s': time_s,
                'schema_version': 3,
                'schedule_mode': mode,
                'command_state': state,
                'vtol_state': 3 if state == 'HOLD_MC' else 1 if state == 'TRANSITION_FW' else 4,
                'failsafe': 0,
                'altitude_m': altitude,
                'altitude_error_m': altitude - 50.0,
                'altitude_local_m': altitude + 1.5,
                'altitude_datum_local_m': 1.5,
                'altitude_relative_m': altitude,
                'target_altitude_local_m': 51.5,
                'target_altitude_relative_m': 50.0,
                'trajectory_sp_altitude_local_m': 51.5,
                'trajectory_sp_altitude_relative_m': 50.0,
                'trajectory_sp_vz_up_mps': '',
                'height_gate_ok': int(abs(altitude - 50.0) <= 1.0),
                'vz_gate_ok': 1,
                'groundspeed_gate_ok': int(
                    0.05 <= 0.2 if state == 'HOLD_MC' else airspeed <= 0.2
                ),
                'stability_gate_ok': int(state == 'HOLD_MC' and abs(altitude - 50.0) <= 1.0),
                'stability_dwell_s': '',
                'va_target_config': va_target_config,
                'va_target_tolerance': va_target_tolerance,
                'va_error_mps': va_error,
                'vt_airspeed_blend_mps_config': 8.0,
                'vt_transition_airspeed_mps_config': 13.0,
                'mc_pitch_weight_proxy': transition_mc_weight_proxy(
                    airspeed, 8.0, 13.0
                ),
                'mc_pitch_weight_actual': transition_mc_weight_proxy(
                    airspeed, 8.0, 13.0
                ),
                'fw_pitch_weight_actual': 1.0 - transition_mc_weight_proxy(
                    airspeed, 8.0, 13.0
                ),
                'mc_pitch_torque_demand_normalized': 0.2,
                'fw_pitch_torque_demand_normalized': -0.1,
                'mc_pitch_torque_contribution_normalized': (
                    0.2 * transition_mc_weight_proxy(
                        airspeed, 8.0, 13.0
                    )
                ),
                'fw_pitch_torque_contribution_normalized': (
                    -0.1 * (1.0 - transition_mc_weight_proxy(
                        airspeed, 8.0, 13.0
                    ))
                ),
                'va_hold_velocity_command_mps': va_velocity_command,
                'va_hold_phase': va_phase,
                'va_hold_measurement_active': va_measurement_active,
                'va_hold_measurement_complete': va_measurement_complete,
                'va_hold_abort_reason': 'none',
                'va_vertical_abort_enabled': 0,
                'va_low_speed_vertical_transient': 0,
                'vz_up_mps': 0.0,
                'groundspeed_mps': 0.05 if state == 'HOLD_MC' else airspeed,
                'airspeed_mps': airspeed,
                'pitch_rad': 0.02,
                'q_rad_s': 0.01,
                'alpha_valid': int(airspeed >= 5.0),
                'lambda_command': command,
                'lambda_exec': executed,
                'lambda_external_active': active,
                'lambda_target_config': target_config,
                'lambda_target_tolerance': target_tolerance,
                'lambda_characterization_complete': complete,
                'pusher_throttle_command': pusher_throttle_command,
                'pusher_throttle_status': pusher_throttle_status,
                'pusher_throttle_external_active': pusher_throttle_active,
                'servo_0': 0.0,
                'servo_1': 0.0,
                'servo_2': 0.0,
                'elevator_angle_proxy_rad': 0.0,
                'elevator_joint_limit_commanded': 0,
                'motor_control_0': 0.4,
                'motor_control_1': 0.4,
                'motor_control_2': 0.4,
                'motor_control_3': 0.4,
                'motor_control_4': pusher_peak,
                'pusher_thrust_setpoint': pusher_peak,
                'lift_collective_thrust_setpoint': 0.5,
                'omega_0_rad_s': 700.0,
                'omega_1_rad_s': 700.0,
                'omega_2_rad_s': 700.0,
                'omega_3_rad_s': 700.0,
                'omega_4_rad_s': 1000.0,
                'lift_power_w': 1000.0 if airspeed < 8.0 else 500.0,
                'pusher_power_w': 100.0,
                'position_age_s': age,
                'attitude_age_s': age,
                'angular_velocity_age_s': age,
                'airspeed_age_s': age,
                'esc_age_s': age,
                'servos_age_s': age,
                'lambda_status_age_s': age,
                'lambda_active_age_s': age,
                'pusher_throttle_status_age_s': age,
                'pusher_throttle_active_age_s': age,
                'actuator_motors_age_s': age,
                'mc_pitch_weight_age_s': age,
                'fw_pitch_weight_age_s': age,
                'mc_torque_setpoint_age_s': age,
                'fw_torque_setpoint_age_s': age,
                'thrust_setpoint_age_s': age,
                'trajectory_setpoint_age_s': age,
            }
            rows.append(row)
        return rows

    @staticmethod
    def _write_and_evaluate(rows):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'run.csv'
            with path.open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=FIELDS)
                writer.writeheader()
                writer.writerows(rows)
            return evaluate(path)

    def test_nominal_pass_has_strict_gate_and_split_metrics(self):
        result = self._write_and_evaluate(self._rows())
        self.assertTrue(result['test_1_native_transition_pass'])
        self.assertTrue(result['test_2_nominal_50m_pass'])
        self.assertTrue(result['test_3_telemetry_pass'])
        self.assertTrue(result['test_4_manual_lambda_pass'])
        self.assertTrue(result['phase_0_5_pusher_diagnostic_pass'])
        self.assertEqual(result['pusher_anomaly_class'], 'none')
        self.assertGreaterEqual(result['pretransition_stability_dwell_s'], 2.0)
        self.assertLess(result['pretransition_hold_duration_s'], 8.0)
        self.assertAlmostEqual(
            result['pretransition_hold_mean_trajectory_sp_altitude_m'],
            50.0,
        )
        self.assertAlmostEqual(result['transition_start_altitude_error_m'], 0.0)
        self.assertAlmostEqual(result['additional_transition_drop_m'], 0.5)
        self.assertAlmostEqual(result['maximum_target_referenced_drop_m'], 0.5)
        self.assertGreater(result['pre_8mps_total_energy_proxy_j'], 0.0)
        self.assertGreater(result['from_8mps_total_energy_proxy_j'], 0.0)
        self.assertEqual(
            result['energy_measurement_kind'],
            'static_motor_model_effort_proxy',
        )
        self.assertTrue(math.isnan(result['transition_lift_energy_j']))
        self.assertTrue(math.isnan(result['transition_pusher_energy_j']))
        self.assertTrue(math.isnan(
            result['transition_total_propulsion_energy_j']
        ))
        json.dumps(result, allow_nan=True)

    def test_nominal_fails_without_two_second_final_stability_dwell(self):
        result = self._write_and_evaluate(
            self._rows(stable_hold=False)
        )
        self.assertLess(result['pretransition_stability_dwell_s'], 2.0)
        self.assertFalse(result['test_2_nominal_50m_pass'])
        self.assertLess(result['pretransition_height_gate_fraction'], 1.0)

    def test_pusher_undercommand_is_automatically_flagged(self):
        result = self._write_and_evaluate(
            self._rows(pusher_peak=0.34)
        )
        self.assertFalse(result['phase_0_5_pusher_diagnostic_pass'])
        self.assertEqual(
            result['pusher_anomaly_class'], 'controller_undercommand'
        )

    def test_commanded_lift_spike_is_not_mislabeled_as_telemetry_only(self):
        rows = self._rows()
        spike = rows[31]
        spike['lift_collective_thrust_setpoint'] = 1.0
        for index in range(4):
            spike[f'motor_control_{index}'] = 1.0
            spike[f'omega_{index}_rad_s'] = 1500.0
        result = self._write_and_evaluate(rows)
        self.assertEqual(result['lift_rotor_spike_samples'], 1)
        self.assertEqual(
            result['lift_rotor_spike_source_class'],
            'commanded_collective_saturation',
        )

    def test_fixed_wing_stock_lambda_cannot_fake_manual_command(self):
        result = self._write_and_evaluate(self._rows(mode='none'))
        self.assertTrue(result['test_1_native_transition_pass'])
        self.assertTrue(math.isnan(result['max_lambda_command']))
        self.assertTrue(math.isnan(result['max_lambda_exec']))
        self.assertEqual(result['lambda_tracking_samples'], 0)
        self.assertEqual(result['lambda_external_active_fraction'], 0.0)
        self.assertFalse(result['test_4_manual_lambda_pass'])

    def test_stale_cached_telemetry_fails_completeness(self):
        result = self._write_and_evaluate(self._rows(stale=True))
        self.assertTrue(result['test_1_native_transition_pass'])
        self.assertFalse(result['test_3_telemetry_pass'])

    def test_relative_altitude_takes_precedence_over_legacy_altitude(self):
        rows = self._rows()
        for row in rows:
            row['altitude_m'] = 48.9
            row['altitude_relative_m'] = 50.0
            row['altitude_error_m'] = 0.0
        result = self._write_and_evaluate(rows)
        self.assertTrue(result['test_2_nominal_50m_pass'])
        self.assertAlmostEqual(result['transition_start_altitude_m'], 50.0)

    def test_pretransition_timeout_is_classified(self):
        rows = self._rows(mode='manual_target')[:55]
        for row in rows:
            row['command_state'] = (
                'HOLD_MC|error=timed out waiting for nominal transition stability'
            )
            row['vtol_state'] = 3
            row['altitude_m'] = 48.9
            row['altitude_relative_m'] = 48.9
            row['altitude_error_m'] = -1.1
            row['vz_up_mps'] = 0.0
            row['groundspeed_mps'] = 0.05
        result = self._write_and_evaluate(rows)
        self.assertEqual(
            result['primary_failure_cause'],
            'pretransition_stability_timeout',
        )
        self.assertEqual(result['pretransition_vz_gate_fraction'], 1.0)
        self.assertEqual(result['pretransition_groundspeed_gate_fraction'], 1.0)
        self.assertEqual(result['pretransition_height_gate_fraction'], 0.0)

    def test_transition_pusher_undercommand_is_primary_failure(self):
        rows = self._rows(mode='manual_target')
        for index, row in enumerate(rows):
            if index >= 30:
                row['command_state'] = 'TRANSITION_FW'
                row['vtol_state'] = 1
                row['airspeed_mps'] = 2.0
                row['groundspeed_mps'] = 0.05
                row['pusher_thrust_setpoint'] = 0.0
                row['omega_4_rad_s'] = 0.0
                row['pusher_power_w'] = 0.0
                row['lambda_command'] = ''
                row['lambda_external_active'] = 0.0
                row['lambda_exec'] = 0.0
                row['lambda_characterization_complete'] = 0
        result = self._write_and_evaluate(rows)
        self.assertEqual(
            result['primary_failure_cause'],
            'transition_pusher_undercommand',
        )
        self.assertEqual(
            result['secondary_failure_cause'],
            'lambda_never_activated',
        )

    def test_manual_target_zero_is_a_valid_characterization_point(self):
        result = self._write_and_evaluate(
            self._rows(mode='manual_target', target=0.0)
        )
        self.assertEqual(result['max_lambda_command'], 0.0)
        self.assertTrue(result['lambda_target_reached'])
        self.assertTrue(result['test_4_manual_lambda_pass'])
        self.assertTrue(result['phase_0_5_lambda_characterization_pass'])

    def test_va_hold_measurement_excludes_release_tail_after_complete(self):
        result = self._write_and_evaluate(
            self._rows(mode='va_hold_target', target=0.0)
        )
        self.assertTrue(result['va_hold_measurement_complete'])
        self.assertEqual(result['va_hold_measurement_samples'], 30)
        self.assertAlmostEqual(result['va_hold_mean_airspeed_mps'], 12.0)
        self.assertAlmostEqual(
            result['va_hold_measurement_pusher_external_active_fraction'],
            1.0,
        )
        self.assertTrue(result['phase_0_75_pusher_airspeed_regulation_pass'])
        self.assertTrue(result['phase_0_75_va_hold_pass'])
        self.assertTrue(result['f1_measurement_pass'])
        self.assertEqual(result['vt_airspeed_blend_mps_config'], 8.0)
        self.assertEqual(result['vt_transition_airspeed_mps_config'], 13.0)
        self.assertAlmostEqual(
            result['va_hold_mean_mc_pitch_weight_proxy'], 0.2
        )

    def test_schema12_uses_native_authority_and_physical_joint_limit_proxy(self):
        rows = self._rows(mode='va_hold_target', target=0.6)
        for row in rows:
            row['schema_version'] = 12
            if (
                row['va_hold_measurement_active'] == 1
                and row['va_hold_measurement_complete'] == 0
            ):
                row['servo_2'] = 0.72
                row['elevator_angle_proxy_rad'] = 0.53
                row['elevator_joint_limit_commanded'] = 1

        result = self._write_and_evaluate(rows)

        self.assertTrue(result['test_3_telemetry_pass'])
        self.assertAlmostEqual(
            result['va_hold_mean_mc_pitch_weight_actual'], 0.2
        )
        self.assertAlmostEqual(
            result['va_hold_mean_fw_pitch_weight_actual'], 0.8
        )
        self.assertAlmostEqual(
            result['va_hold_mc_pitch_weight_proxy_rmse'], 0.0
        )
        self.assertAlmostEqual(
            result['va_hold_elevator_joint_limit_command_fraction'], 1.0
        )
        self.assertAlmostEqual(
            result['va_hold_mean_fw_pitch_torque_demand_normalized'], -0.1
        )

    def test_schema13_requires_lambda_synchronized_attitude_weights(self):
        rows = self._rows(mode='va_hold_target', target=0.6)
        expected_mc, expected_fw = transition_attitude_weights(0.6)
        for row in rows:
            row['schema_version'] = 13
            row['lambda_attitude_blend_start_config'] = 0.1
            row['lambda_attitude_blend_full_config'] = 0.9
            if (
                row['va_hold_measurement_active'] == 1
                and row['va_hold_measurement_complete'] == 0
            ):
                row['mc_pitch_weight_actual'] = expected_mc
                row['fw_pitch_weight_actual'] = expected_fw
                row['mc_pitch_weight_lambda_expected'] = expected_mc
                row['fw_pitch_weight_lambda_expected'] = expected_fw

        result = self._write_and_evaluate(rows)
        self.assertTrue(result['lambda_attitude_sync_pass'])
        self.assertTrue(result['target_lambda_attitude_sync_pass'])
        self.assertGreaterEqual(
            result['target_lambda_attitude_sync_samples'], 10
        )
        self.assertTrue(result['f1_measurement_pass'])
        self.assertAlmostEqual(
            result['va_hold_mc_pitch_weight_lambda_sync_rmse'], 0.0
        )

        for row in rows:
            if row['va_hold_measurement_active'] == 1:
                row['mc_pitch_weight_actual'] = 1.0
        result = self._write_and_evaluate(rows)
        self.assertFalse(result['lambda_attitude_sync_pass'])
        self.assertFalse(result['target_lambda_attitude_sync_pass'])
        self.assertFalse(result['f1_measurement_pass'])

    def test_schema13_zero_lambda_endpoint_cannot_validate_nonzero_target(self):
        rows = self._rows(mode='va_hold_target', target=0.7)
        for row in rows:
            row['schema_version'] = 13
            row['lambda_attitude_blend_start_config'] = 0.1
            row['lambda_attitude_blend_full_config'] = 0.9
            if row['lambda_external_active'] == 1:
                # This is the exact pre-measurement failure mode seen in the
                # 14 m/s, lambda=0.7 attempt: lambda never left zero, so the
                # MC endpoint is self-consistent but the requested point was
                # not exercised.
                row['lambda_exec'] = 0.0
                row['mc_pitch_weight_actual'] = 1.0
                row['fw_pitch_weight_actual'] = 0.0
                row['mc_pitch_weight_lambda_expected'] = 1.0
                row['fw_pitch_weight_lambda_expected'] = 0.0

        result = self._write_and_evaluate(rows)

        self.assertTrue(result['transition_lambda_attitude_sync_pass'])
        self.assertFalse(result['lambda_target_reached'])
        self.assertEqual(result['target_lambda_attitude_sync_samples'], 0)
        self.assertFalse(result['target_lambda_attitude_sync_pass'])

    def test_v9_fixed_window_tolerates_one_soft_band_excursion(self):
        rows = self._rows(mode='va_hold_target', target=0.3)
        measurement = [
            row for row in rows
            if row['va_hold_measurement_active'] == 1
            and row['va_hold_measurement_complete'] == 0
        ]
        for row in rows:
            row['schema_version'] = 9
        measurement[10]['airspeed_mps'] = 12.4
        measurement[10]['va_error_mps'] = 0.4

        result = self._write_and_evaluate(rows)

        self.assertAlmostEqual(
            result['va_hold_airspeed_in_band_fraction'], 29 / 30
        )
        self.assertAlmostEqual(
            result['va_hold_max_abs_airspeed_error_mps'], 0.4
        )
        self.assertTrue(result['va_hold_airspeed_band_occupancy_pass'])
        self.assertTrue(result['va_hold_airspeed_max_error_guard_pass'])
        self.assertTrue(result['va_hold_fixed_window_quality_pass'])
        self.assertTrue(result['f1_measurement_pass'])

    def test_v9_fixed_window_enforces_occupancy_and_max_error_guard(self):
        rows = self._rows(mode='va_hold_target', target=0.3)
        measurement = [
            row for row in rows
            if row['va_hold_measurement_active'] == 1
            and row['va_hold_measurement_complete'] == 0
        ]
        for row in rows:
            row['schema_version'] = 9
        for row in measurement[:4]:
            row['airspeed_mps'] = 12.4
            row['va_error_mps'] = 0.4

        result = self._write_and_evaluate(rows)
        self.assertFalse(result['va_hold_airspeed_band_occupancy_pass'])
        self.assertTrue(result['va_hold_airspeed_max_error_guard_pass'])
        self.assertIn(
            'airspeed_band_occupancy',
            result['va_hold_fixed_window_quality_failures'],
        )
        self.assertFalse(result['f1_measurement_pass'])

        rows = self._rows(mode='va_hold_target', target=0.3)
        measurement = [
            row for row in rows
            if row['va_hold_measurement_active'] == 1
            and row['va_hold_measurement_complete'] == 0
        ]
        for row in rows:
            row['schema_version'] = 9
        measurement[10]['airspeed_mps'] = 12.7
        measurement[10]['va_error_mps'] = 0.7
        result = self._write_and_evaluate(rows)
        self.assertTrue(result['va_hold_airspeed_band_occupancy_pass'])
        self.assertFalse(result['va_hold_airspeed_max_error_guard_pass'])
        self.assertIn(
            'airspeed_max_error_guard',
            result['va_hold_fixed_window_quality_failures'],
        )
        self.assertFalse(result['f1_measurement_pass'])

    def test_low_speed_f1_measurement_does_not_require_reaching_8_mps(self):
        rows = self._rows(mode='va_hold_target', target=0.3)
        for row in rows:
            if row['schedule_mode'] != 'va_hold_target':
                continue
            row['va_target_config'] = 6.0
            if row['command_state'] == 'TRANSITION_FW':
                if row['va_hold_measurement_active'] == 1:
                    row['airspeed_mps'] = 6.0
                    row['groundspeed_mps'] = 6.0
                    row['va_error_mps'] = 0.0
                elif row['va_hold_measurement_complete'] == 1:
                    # Do not let the release tail satisfy the old 8 m/s
                    # acceleration diagnostic by accident.
                    row['airspeed_mps'] = 6.0
                    row['groundspeed_mps'] = 6.0
                    row['va_error_mps'] = 0.0

        result = self._write_and_evaluate(rows)

        self.assertFalse(result['phase_0_5_pusher_diagnostic_pass'])
        self.assertFalse(result['phase_0_75_va_hold_pass'])
        self.assertTrue(result['f1_measurement_pass'])

    def test_f1_fixed_window_pass_does_not_require_post_window_fw_hold(self):
        rows = self._rows(mode='va_hold_target', target=0.3)[:95]

        result = self._write_and_evaluate(rows)

        self.assertFalse(result['test_1_native_transition_pass'])
        self.assertFalse(result['test_2_nominal_50m_pass'])
        self.assertTrue(result['va_hold_measurement_complete'])
        self.assertTrue(result['va_hold_fixed_window_altitude_pass'])
        self.assertTrue(result['f1_measurement_pass'])
        self.assertEqual(result['f1_attempt_quality'], 'valid')

    def test_fragmented_va_hold_measurement_is_rejected(self):
        rows = self._rows(mode='va_hold_target', target=0.0)
        for index, row in enumerate(rows):
            if row['command_state'] != 'TRANSITION_FW':
                continue
            if 50 <= index < 55:
                row['va_hold_phase'] = 'measurement'
                row['va_hold_measurement_active'] = 1
                row['va_hold_measurement_complete'] = 0
                row['airspeed_mps'] = 16.0
                row['va_error_mps'] = 4.0
                row['lambda_exec'] = 1.0
                row['pusher_throttle_external_active'] = 0.0
            elif 55 <= index < 60:
                row['va_hold_phase'] = 'measurement'
                row['va_hold_measurement_active'] = 0
                row['va_hold_measurement_complete'] = 0
            elif 60 <= index < 95:
                row['va_hold_phase'] = 'measurement'
                row['va_hold_measurement_active'] = 1
                row['va_hold_measurement_complete'] = 0
                row['airspeed_mps'] = 12.0
                row['va_error_mps'] = 0.0
                row['lambda_command'] = 0.0
                row['lambda_exec'] = 0.0
                row['lambda_external_active'] = 1.0
                row['pusher_throttle_command'] = 0.24
                row['pusher_throttle_status'] = 0.24
                row['pusher_throttle_external_active'] = 1.0
            elif 95 <= index < 100:
                row['va_hold_phase'] = 'measurement'
                row['va_hold_measurement_active'] = 0
                row['va_hold_measurement_complete'] = 1
        result = self._write_and_evaluate(rows)
        self.assertEqual(result['va_hold_measurement_segment_count'], 2)
        self.assertEqual(result['va_hold_selected_measurement_segment_index'], 1)
        self.assertEqual(result['va_hold_measurement_samples'], 35)
        self.assertAlmostEqual(result['va_hold_mean_airspeed_mps'], 12.0)
        self.assertAlmostEqual(result['va_hold_mean_lambda_exec'], 0.0)
        self.assertAlmostEqual(
            result['va_hold_measurement_pusher_external_active_fraction'],
            1.0,
        )
        self.assertFalse(result['va_hold_measurement_protocol_valid'])
        self.assertEqual(
            result['primary_failure_cause'],
            'measurement_window_fragmented',
        )
        self.assertFalse(result['phase_0_75_va_hold_pass'])
        self.assertFalse(result['f1_measurement_pass'])

    def test_low_speed_altitude_abort_is_protocol_transient(self):
        rows = self._rows(mode='va_hold_target', target=0.3)[:50]
        for index, row in enumerate(rows):
            if row['command_state'] != 'TRANSITION_FW':
                continue
            transition_index = index - 30
            altitude = 50.0 + 0.6 * transition_index
            row['altitude_m'] = altitude
            row['altitude_relative_m'] = altitude
            row['altitude_error_m'] = altitude - 50.0
            row['airspeed_mps'] = min(4.0, 0.25 * transition_index)
            row['groundspeed_mps'] = row['airspeed_mps']
            row['vz_up_mps'] = 5.0
            row['va_hold_phase'] = 'airspeed_settle'
            row['va_hold_measurement_active'] = 0
            row['va_hold_measurement_complete'] = 0
            row['va_hold_abort_reason'] = 'none'
        rows[-1]['va_hold_abort_reason'] = 'va_hold_altitude_runaway'

        result = self._write_and_evaluate(rows)

        self.assertEqual(
            result['primary_failure_cause'],
            'pre_measurement_altitude_transient',
        )
        self.assertEqual(
            result['secondary_failure_cause'],
            'pre_measurement_not_f1_boundary',
        )
        self.assertTrue(result['va_hold_pre_measurement_altitude_transient'])
        self.assertFalse(result['phase_0_75_va_hold_pass'])

    def test_measurement_band_altitude_abort_remains_unsafe(self):
        rows = self._rows(mode='va_hold_target', target=0.7)[:90]
        for row in rows:
            if row['command_state'] != 'TRANSITION_FW':
                continue
            row['airspeed_mps'] = 10.0
            row['groundspeed_mps'] = 10.0
            row['va_hold_phase'] = 'lambda_settle'
            row['va_hold_measurement_active'] = 0
            row['va_hold_measurement_complete'] = 0
            row['lambda_command'] = 0.7
            row['lambda_exec'] = 0.7
            row['lambda_external_active'] = 1.0
            row['altitude_m'] = 44.8
            row['altitude_relative_m'] = 44.8
            row['altitude_error_m'] = -5.2
            row['vz_up_mps'] = -0.8
            row['va_hold_abort_reason'] = 'none'
        rows[-1]['va_hold_abort_reason'] = 'va_hold_altitude_runaway'

        result = self._write_and_evaluate(rows)

        self.assertEqual(
            result['primary_failure_cause'],
            'va_hold_altitude_runaway',
        )
        self.assertFalse(result['va_hold_pre_measurement_altitude_transient'])
        self.assertFalse(result['phase_0_75_va_hold_pass'])

    def test_target_directed_ramp_abort_is_valid_f1_physical_evidence(self):
        rows = self._rows(mode='va_hold_target', target=0.9)[:70]
        for row in rows:
            row['va_target_config'] = 10.0 if row['va_target_config'] != '' else ''
        for index, row in enumerate(rows):
            if row['command_state'] != 'TRANSITION_FW':
                continue
            transition_index = max(0, index - 30)
            row['airspeed_mps'] = min(10.0, 0.35 * transition_index)
            row['groundspeed_mps'] = row['airspeed_mps']
            row['va_target_tolerance'] = 0.3
            row['va_error_mps'] = row['airspeed_mps'] - 10.0
            row['va_hold_phase'] = 'lambda_settle'
            row['va_hold_measurement_active'] = 0
            row['va_hold_measurement_complete'] = 0
            row['lambda_command'] = 0.9
            row['lambda_exec'] = min(0.62, max(0.0, 0.02 * transition_index))
            row['lambda_external_active'] = 1.0
            row['mc_pitch_weight_actual'] = 0.4
            row['fw_pitch_weight_actual'] = 0.6
            row['mc_pitch_weight_lambda_expected'] = 0.4
            row['fw_pitch_weight_lambda_expected'] = 0.6
            row['altitude_m'] = max(44.7, 50.0 - 0.15 * transition_index)
            row['altitude_relative_m'] = row['altitude_m']
            row['altitude_error_m'] = row['altitude_m'] - 50.0
            row['vz_up_mps'] = -1.5
        for row in rows:
            row['schema_version'] = 13
            row['lambda_attitude_blend_start_config'] = 0.1
            row['lambda_attitude_blend_full_config'] = 0.9
        rows[-1]['va_hold_abort_reason'] = 'va_hold_altitude_runaway'

        result = self._write_and_evaluate(rows)

        self.assertEqual(result['primary_failure_cause'],
                         'va_hold_altitude_runaway')
        self.assertEqual(result['f1_attempt_quality'], 'valid')
        self.assertTrue(result['f1_target_condition_established'])
        self.assertTrue(result['f1_target_directed_ramp_established'])
        self.assertTrue(result['transition_lambda_attitude_sync_pass'])

    def test_manual_target_ramp_transient_does_not_fail_steady_characterization(self):
        rows = self._rows(mode='manual_target', target=0.5)
        commanded_index = 0
        for row in rows:
            if row['schedule_mode'] != 'manual_target':
                continue
            if row['command_state'] != 'TRANSITION_FW':
                continue
            if row['lambda_command'] == '':
                continue
            commanded_index += 1
            if commanded_index <= 20:
                row['lambda_exec'] = 0.01 * commanded_index
            else:
                row['lambda_exec'] = 0.5
        result = self._write_and_evaluate(rows)
        self.assertGreater(result['mean_lambda_tracking_error'], 0.1)
        self.assertGreater(result['max_lambda_tracking_error'], 0.2)
        self.assertLessEqual(
            result['lambda_steady_state_mean_abs_error'],
            result['lambda_target_tolerance'],
        )
        self.assertTrue(result['test_4_manual_lambda_pass'])
        self.assertTrue(result['phase_0_5_lambda_characterization_pass'])


if __name__ == '__main__':
    unittest.main()
class TransitionStartReadinessTests(unittest.TestCase):
    def test_latches_complete_transition_entry_snapshot(self) -> None:
        self.assertEqual(
            transition_start_readiness({
                'height_gate_ok': 1.0,
                'vz_gate_ok': 1.0,
                'groundspeed_gate_ok': 1.0,
            }),
            (True, True, True, True),
        )

    def test_missing_or_failed_snapshot_is_not_ready(self) -> None:
        self.assertEqual(
            transition_start_readiness({
                'height_gate_ok': 1.0,
                'vz_gate_ok': 1.0,
            }),
            (False, False, False, False),
        )
        self.assertEqual(
            transition_start_readiness({
                'height_gate_ok': 1.0,
                'vz_gate_ok': 0.0,
                'groundspeed_gate_ok': 1.0,
            }),
            (True, False, True, False),
        )
