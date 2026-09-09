#!/usr/bin/env python3
"""Run the navigation-free 50 m hover-to-cruise validation mission."""

import os
from pathlib import Path
import re

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    Shutdown,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _speed_token(value: float) -> str:
    return f'{value:g}'.replace('.', 'p').replace('-', 'm')


def _materialize_sim_speed_world(
    *,
    world: Path,
    output_csv: Path,
    sim_speed_factor: float,
) -> tuple[Path, float]:
    text = world.read_text(encoding='utf-8')
    physics_match = re.search(
        r'(<physics\b[^>]*>)(.*?)(</physics>)',
        text,
        flags=re.DOTALL,
    )
    if not physics_match:
        raise RuntimeError(f'world has no <physics> block: {world}')
    physics_body = physics_match.group(2)
    max_step_match = re.search(
        r'<max_step_size>\s*([^<]+)\s*</max_step_size>',
        physics_body,
    )
    if not max_step_match:
        raise RuntimeError(f'world physics has no max_step_size: {world}')
    max_step_size = float(max_step_match.group(1))
    if max_step_size <= 0.0:
        raise RuntimeError(f'invalid max_step_size in {world}: {max_step_size}')
    if abs(sim_speed_factor - 1.0) <= 1.0e-12:
        return world, max_step_size

    def replace_or_insert(
        body: str,
        tag: str,
        value: str,
        *,
        after_tag: str | None = None,
    ) -> str:
        pattern = rf'(<{tag}>)([^<]*)(</{tag}>)'
        if re.search(pattern, body):
            return re.sub(
                pattern,
                lambda match: f'{match.group(1)}{value}{match.group(3)}',
                body,
                count=1,
            )
        insert = f'\n      <{tag}>{value}</{tag}>'
        if after_tag is not None:
            after_pattern = rf'(</{after_tag}>)'
            if re.search(after_pattern, body):
                return re.sub(after_pattern, rf'\1{insert}', body, count=1)
        return insert + body

    updated_body = replace_or_insert(
        physics_body,
        'real_time_factor',
        f'{sim_speed_factor:g}',
        after_tag='max_step_size',
    )
    updated_body = replace_or_insert(
        updated_body,
        'real_time_update_rate',
        f'{sim_speed_factor / max_step_size:.9g}',
        after_tag='real_time_factor',
    )
    patched_text = (
        text[:physics_match.start(2)]
        + updated_body
        + text[physics_match.end(2):]
    )
    world_dir = output_csv.parent / 'worlds'
    world_dir.mkdir(parents=True, exist_ok=True)
    patched_world = (
        world_dir / f'{world.stem}_sim_speed_{_speed_token(sim_speed_factor)}.sdf'
    )
    patched_world.write_text(patched_text, encoding='utf-8')
    return patched_world, max_step_size


def _setup(context):
    share = Path(get_package_share_directory('ca_lsc_transition'))
    px4_dir = Path(LaunchConfiguration('px4_dir').perform(context)).expanduser().resolve()
    build_dir = Path(LaunchConfiguration('px4_build_dir').perform(context)).expanduser().resolve()
    world = Path(LaunchConfiguration('world').perform(context)).expanduser().resolve()
    output_csv = Path(LaunchConfiguration('output_csv').perform(context)).expanduser().resolve()
    px4_workdir = Path(LaunchConfiguration('px4_workdir').perform(context)).expanduser().resolve()
    world_name = LaunchConfiguration('world_name').perform(context)
    model_name = LaunchConfiguration('model_name').perform(context)
    gz_partition = LaunchConfiguration('gz_partition').perform(context)
    sim_speed_factor = float(
        LaunchConfiguration('sim_speed_factor').perform(context)
    )
    px4_sim_speed_factor = float(
        LaunchConfiguration('px4_sim_speed_factor').perform(context)
    )
    if sim_speed_factor <= 0.0:
        raise RuntimeError(
            f'sim_speed_factor must be positive, got {sim_speed_factor}'
        )
    if px4_sim_speed_factor <= 0.0:
        raise RuntimeError(
            'px4_sim_speed_factor must be positive, got '
            f'{px4_sim_speed_factor}'
        )
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    px4_workdir.mkdir(parents=True, exist_ok=True)

    px4_binary = build_dir / 'bin' / 'px4'
    px4_rcs = build_dir / 'etc' / 'init.d-posix' / 'rcS'
    px4_romfs = px4_dir / 'ROMFS' / 'px4fmu_common'
    px4_param = build_dir / 'bin' / 'px4-param'
    required = (world, px4_binary, px4_rcs, px4_romfs, px4_param)
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise RuntimeError('missing nominal-transition inputs: ' + ', '.join(missing))
    gazebo_world, world_max_step_size = _materialize_sim_speed_world(
        world=world,
        output_csv=output_csv,
        sim_speed_factor=sim_speed_factor,
    )

    resource_paths = [
        str(share / 'worlds'),
        str(px4_dir / 'Tools' / 'simulation' / 'gz' / 'models'),
        str(px4_dir / 'Tools' / 'simulation' / 'gz' / 'worlds'),
    ]
    extra_resource_path = LaunchConfiguration(
        'extra_gz_resource_path'
    ).perform(context).strip()
    if extra_resource_path:
        resource_paths.extend(
            item for item in extra_resource_path.split(':') if item
        )
    resource_path = ':'.join(resource_paths)
    diagnostic_plugin_path = str(
        Path(get_package_prefix('vtol_px4_control'))
        / 'lib' / 'vtol_px4_control'
    )
    inherited_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    common_gz_env = {
        'GZ_PARTITION': gz_partition,
        'GZ_SIM_RESOURCE_PATH': resource_path,
        'GZ_SIM_SYSTEM_PLUGIN_PATH': ':'.join(
            item for item in (
                diagnostic_plugin_path, inherited_plugin_path,
            ) if item
        ),
    }
    gazebo = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-s', '-r', '-v',
            LaunchConfiguration('gz_verbosity').perform(context),
            str(gazebo_world),
        ],
        name='ca_lsc_gazebo',
        output='screen',
        emulate_tty=True,
        additional_env=common_gz_env,
        on_exit=Shutdown(reason='Gazebo exited'),
    )
    agent = ExecuteProcess(
        cmd=[
            'MicroXRCEAgent', 'udp4', '-p',
            LaunchConfiguration('agent_port').perform(context), '-v', '1',
        ],
        name='ca_lsc_micro_xrce_agent',
        output='screen',
        emulate_tty=True,
    )
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ca_lsc_clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
        additional_env={'GZ_PARTITION': gz_partition},
    )
    px4_env = {
        **common_gz_env,
        'PX4_GZ_STANDALONE': '1',
        'PX4_GZ_WORLD': world_name,
        'PX4_GZ_MODEL_NAME': model_name,
        'PX4_SYS_AUTOSTART': '4004',
        'PX4_SIM_SPEED_FACTOR': f'{px4_sim_speed_factor:g}',
        'PX4_UXRCE_DDS_SYNCT': '0',
        'CA_LSC_SIM_SPEED_FACTOR': f'{sim_speed_factor:g}',
        'CA_LSC_PX4_SIM_SPEED_FACTOR': f'{px4_sim_speed_factor:g}',
        'CA_LSC_WORLD_MAX_STEP_SIZE': f'{world_max_step_size:g}',
    }
    px4 = ExecuteProcess(
        cmd=[
            str(px4_binary), '-d', '-s', str(px4_rcs), str(px4_romfs),
            '-i', '0', '-w', str(px4_workdir),
        ],
        name='ca_lsc_px4_sitl',
        output='screen',
        emulate_tty=True,
        additional_env=px4_env,
    )

    def parameter(name, value):
        return ExecuteProcess(
            cmd=[str(px4_param), '--instance', '0', 'set', name, value],
            name=f'ca_lsc_param_{name.lower()}',
            cwd=str(px4_workdir),
            output='screen',
        )

    parameters = [
        parameter('UXRCE_DDS_SYNCT', '0'),
        parameter(
            'FW_AIRSPD_MIN',
            LaunchConfiguration('fw_airspeed_min').perform(context),
        ),
        parameter(
            'FW_AIRSPD_TRIM',
            LaunchConfiguration('fw_airspeed_trim').perform(context),
        ),
        parameter(
            'FW_AIRSPD_MAX',
            LaunchConfiguration('fw_airspeed_max').perform(context),
        ),
        parameter(
            'FW_THR_MAX',
            LaunchConfiguration('fw_throttle_max').perform(context),
        ),
        parameter(
            'FW_T_CLMB_MAX',
            LaunchConfiguration('fw_climb_rate_max').perform(context),
        ),
        parameter(
            'VT_ARSP_BLEND',
            LaunchConfiguration('vt_blend_airspeed').perform(context),
        ),
        parameter(
            'VT_ARSP_TRANS',
            LaunchConfiguration('vt_transition_airspeed').perform(context),
        ),
        parameter(
            'VT_F_TRANS_THR',
            LaunchConfiguration('vt_forward_transition_throttle').perform(context),
        ),
        parameter(
            'VT_UNLD_TEST_EN',
            LaunchConfiguration('vt_unload_test_enable').perform(context),
        ),
        parameter(
            'VT_UNLD_ALT_P',
            LaunchConfiguration('vt_unload_altitude_pitch_kp').perform(context),
        ),
        parameter(
            'VT_UNLD_VZ_D',
            LaunchConfiguration('vt_unload_vertical_speed_pitch_kd').perform(context),
        ),
        parameter(
            'VT_UNLD_P_MIN',
            LaunchConfiguration('vt_unload_pitch_min').perform(context),
        ),
        parameter(
            'VT_UNLD_P_MAX',
            LaunchConfiguration('vt_unload_pitch_max').perform(context),
        ),
        parameter(
            'VT_UNLD_W0',
            LaunchConfiguration('lambda_attitude_blend_start').perform(context),
        ),
        parameter(
            'VT_UNLD_W1',
            LaunchConfiguration('lambda_attitude_blend_full').perform(context),
        ),
        parameter('VT_TRANS_TIMEOUT', '20.0'),
    ]
    state_monitor = Node(
        package='vtol_px4_control',
        executable='vtol_state_monitor',
        name='ca_lsc_state_monitor',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    recorder = Node(
        package='ca_lsc_transition',
        executable='transition_experiment',
        name='ca_lsc_transition_experiment',
        output='screen',
        on_exit=Shutdown(reason='transition experiment finished'),
        additional_env={
            'PYTHONPATH': ':'.join(
                item for item in (
                    str(px4_dir.parent / 'src'),
                    os.environ.get('PYTHONPATH', ''),
                ) if item
            ),
        },
        parameters=[{
            'use_sim_time': True,
            'output_csv': str(output_csv),
            'condition_id': LaunchConfiguration('condition_id'),
            'wind_model': LaunchConfiguration('wind_model'),
            'sim_speed_factor': ParameterValue(
                LaunchConfiguration('sim_speed_factor'), value_type=float),
            'wind_cmd_e_enu_mps': ParameterValue(
                LaunchConfiguration('wind_cmd_e_enu'), value_type=float),
            'wind_cmd_n_enu_mps': ParameterValue(
                LaunchConfiguration('wind_cmd_n_enu'), value_type=float),
            'wind_cmd_u_enu_mps': ParameterValue(
                LaunchConfiguration('wind_cmd_u_enu'), value_type=float),
            'gust_amplitude_mps': ParameterValue(
                LaunchConfiguration('gust_amplitude'), value_type=float),
            'gust_delay_s': ParameterValue(
                LaunchConfiguration('gust_delay'), value_type=float),
            'gust_duration_s': ParameterValue(
                LaunchConfiguration('gust_duration'), value_type=float),
            'gust_recovery_s': ParameterValue(
                LaunchConfiguration('gust_recovery'), value_type=float),
            'gust_status_timeout_s': ParameterValue(
                LaunchConfiguration('gust_status_timeout'), value_type=float),
            'airspeed_source': LaunchConfiguration('airspeed_source'),
            'mass_scale': ParameterValue(
                LaunchConfiguration('mass_scale'), value_type=float),
            'nominal_model_mass_kg': ParameterValue(
                LaunchConfiguration('nominal_model_mass_kg'), value_type=float),
            'actual_model_mass_kg': ParameterValue(
                LaunchConfiguration('actual_model_mass_kg'), value_type=float),
            'payload_mass_kg': ParameterValue(
                LaunchConfiguration('payload_mass_kg'), value_type=float),
            'wing_force_gt_enabled': ParameterValue(
                LaunchConfiguration('wing_force_gt_enabled'),
                value_type=bool,
            ),
            'wing_force_gt_timeout_s': ParameterValue(
                LaunchConfiguration('wing_force_gt_timeout'),
                value_type=float,
            ),
            'eta_l_estimated_mass_kg': ParameterValue(
                LaunchConfiguration('eta_l_estimated_mass_kg'),
                value_type=float,
            ),
            'eta_l_desired_vertical_acceleration_mps2': ParameterValue(
                LaunchConfiguration('eta_l_desired_vertical_acceleration'),
                value_type=float,
            ),
            'eta_c_pressure_gate_lower_pa': ParameterValue(
                LaunchConfiguration('eta_c_pressure_gate_lower'),
                value_type=float,
            ),
            'eta_c_pressure_gate_upper_pa': ParameterValue(
                LaunchConfiguration('eta_c_pressure_gate_upper'),
                value_type=float,
            ),
            'f3b2_test_mode': LaunchConfiguration('f3b2_test_mode'),
            'f3b2_shadow_demand_offset_nm': ParameterValue(
                LaunchConfiguration('f3b2_shadow_demand_offset'),
                value_type=float,
            ),
            'f3b2_shadow_demand_for_eta_c_nm': ParameterValue(
                LaunchConfiguration('f3b2_shadow_demand_for_eta_c'),
                value_type=float,
            ),
            'f3b2_elevator_bias_rad': ParameterValue(
                LaunchConfiguration('f3b2_elevator_bias'),
                value_type=float,
            ),
            'f3b2_stage_index': ParameterValue(
                LaunchConfiguration('f3b2_stage_index'), value_type=float,
            ),
            'f3b2_gt_dmoment_ddelta_per_q_m3': ParameterValue(
                LaunchConfiguration('f3b2_gt_dmoment_ddelta_per_q_m3'),
                value_type=float,
            ),
            'elevator_id_dither_enabled': ParameterValue(
                LaunchConfiguration('elevator_id_dither_enabled'),
                value_type=bool,
            ),
            'target_altitude_m': ParameterValue(
                LaunchConfiguration('takeoff_altitude'), value_type=float),
            'schedule_mode': LaunchConfiguration('schedule_mode'),
            'a0_rl_command_timeout_s': ParameterValue(
                LaunchConfiguration('a0_rl_command_timeout'), value_type=float),
            'a0_rl_handover_lambda': ParameterValue(
                LaunchConfiguration('a0_rl_handover_lambda'), value_type=float),
            'a0_rl_handover_dwell_s': ParameterValue(
                LaunchConfiguration('a0_rl_handover_dwell'), value_type=float),
            'lambda_start_airspeed_mps': ParameterValue(
                LaunchConfiguration('lambda_start_airspeed'), value_type=float),
            'lambda_full_airspeed_mps': ParameterValue(
                LaunchConfiguration('lambda_full_airspeed'), value_type=float),
            'lambda_maximum': ParameterValue(
                LaunchConfiguration('lambda_maximum'), value_type=float),
            'lambda_target': ParameterValue(
                LaunchConfiguration('lambda_target'), value_type=float),
            'lambda_target_tolerance': ParameterValue(
                LaunchConfiguration('lambda_target_tolerance'), value_type=float),
            'lambda_target_dwell_s': ParameterValue(
                LaunchConfiguration('lambda_target_dwell'), value_type=float),
            'va_target_mps': ParameterValue(
                LaunchConfiguration('va_target'), value_type=float),
            'va_target_tolerance_mps': ParameterValue(
                LaunchConfiguration('va_target_tolerance'), value_type=float),
            'va_target_dwell_s': ParameterValue(
                LaunchConfiguration('va_target_dwell'), value_type=float),
            'va_measurement_s': ParameterValue(
                LaunchConfiguration('va_measurement_s'), value_type=float),
            'va_measurement_min_band_fraction': ParameterValue(
                LaunchConfiguration('va_measurement_min_band_fraction'),
                value_type=float,
            ),
            'va_measurement_max_abs_airspeed_error_mps': ParameterValue(
                LaunchConfiguration(
                    'va_measurement_max_abs_airspeed_error'
                ),
                value_type=float,
            ),
            'va_velocity_kp': ParameterValue(
                LaunchConfiguration('va_velocity_kp'), value_type=float),
            'va_velocity_min_mps': ParameterValue(
                LaunchConfiguration('va_velocity_min'), value_type=float),
            'va_velocity_max_mps': ParameterValue(
                LaunchConfiguration('va_velocity_max'), value_type=float),
            'va_altitude_kp': ParameterValue(
                LaunchConfiguration('va_altitude_kp'), value_type=float),
            'va_altitude_vertical_speed_kd': ParameterValue(
                LaunchConfiguration('va_altitude_vertical_speed_kd'),
                value_type=float,
            ),
            'va_altitude_velocity_limit_mps': ParameterValue(
                LaunchConfiguration('va_altitude_velocity_limit'),
                value_type=float,
            ),
            'vt_unload_altitude_pitch_kp': ParameterValue(
                LaunchConfiguration('vt_unload_altitude_pitch_kp'),
                value_type=float,
            ),
            'vt_unload_vertical_speed_pitch_kd': ParameterValue(
                LaunchConfiguration('vt_unload_vertical_speed_pitch_kd'),
                value_type=float,
            ),
            'vt_unload_pitch_min_deg': ParameterValue(
                LaunchConfiguration('vt_unload_pitch_min'), value_type=float),
            'vt_unload_pitch_max_deg': ParameterValue(
                LaunchConfiguration('vt_unload_pitch_max'), value_type=float),
            'vt_airspeed_blend_mps': ParameterValue(
                LaunchConfiguration('vt_blend_airspeed'), value_type=float),
            'vt_transition_airspeed_mps': ParameterValue(
                LaunchConfiguration('vt_transition_airspeed'),
                value_type=float,
            ),
            'lambda_attitude_blend_start': ParameterValue(
                LaunchConfiguration('lambda_attitude_blend_start'),
                value_type=float,
            ),
            'lambda_attitude_blend_full': ParameterValue(
                LaunchConfiguration('lambda_attitude_blend_full'),
                value_type=float,
            ),
            'physics_prior_eta_l_lower': ParameterValue(
                LaunchConfiguration('physics_prior_eta_l_lower'),
                value_type=float,
            ),
            'physics_prior_eta_l_upper': ParameterValue(
                LaunchConfiguration('physics_prior_eta_l_upper'),
                value_type=float,
            ),
            'physics_prior_eta_c_lower': ParameterValue(
                LaunchConfiguration('physics_prior_eta_c_lower'),
                value_type=float,
            ),
            'physics_prior_eta_c_upper': ParameterValue(
                LaunchConfiguration('physics_prior_eta_c_upper'),
                value_type=float,
            ),
            'physics_prior_hard_eta_l_lower': ParameterValue(
                LaunchConfiguration('physics_prior_hard_eta_l_lower'),
                value_type=float,
            ),
            'physics_prior_hard_eta_l_upper': ParameterValue(
                LaunchConfiguration('physics_prior_hard_eta_l_upper'),
                value_type=float,
            ),
            'physics_prior_hard_eta_c_lower': ParameterValue(
                LaunchConfiguration('physics_prior_hard_eta_c_lower'),
                value_type=float,
            ),
            'physics_prior_hard_eta_c_upper': ParameterValue(
                LaunchConfiguration('physics_prior_hard_eta_c_upper'),
                value_type=float,
            ),
            'physics_prior_altitude_drop_soft_m': ParameterValue(
                LaunchConfiguration('physics_prior_altitude_drop_soft'),
                value_type=float,
            ),
            'physics_prior_altitude_drop_hard_m': ParameterValue(
                LaunchConfiguration('physics_prior_altitude_drop_hard'),
                value_type=float,
            ),
            'physics_prior_descent_soft_mps': ParameterValue(
                LaunchConfiguration('physics_prior_descent_soft'),
                value_type=float,
            ),
            'physics_prior_descent_hard_mps': ParameterValue(
                LaunchConfiguration('physics_prior_descent_hard'),
                value_type=float,
            ),
            'physics_prior_alpha_soft_rad': ParameterValue(
                LaunchConfiguration('physics_prior_alpha_soft'),
                value_type=float,
            ),
            'physics_prior_alpha_hard_rad': ParameterValue(
                LaunchConfiguration('physics_prior_alpha_hard'),
                value_type=float,
            ),
            'physics_prior_unloading_rate_per_s': ParameterValue(
                LaunchConfiguration('physics_prior_unloading_rate'),
                value_type=float,
            ),
            'physics_prior_recovery_rate_per_s': ParameterValue(
                LaunchConfiguration('physics_prior_recovery_rate'),
                value_type=float,
            ),
            'physics_prior_emergency_recovery_rate_per_s': ParameterValue(
                LaunchConfiguration('physics_prior_emergency_recovery_rate'),
                value_type=float,
            ),
            'physics_prior_release_lambda': ParameterValue(
                LaunchConfiguration('physics_prior_release_lambda'),
                value_type=float,
            ),
            'physics_prior_release_dwell_s': ParameterValue(
                LaunchConfiguration('physics_prior_release_dwell'),
                value_type=float,
            ),
            'elevator_joint_min_rad': ParameterValue(
                LaunchConfiguration('elevator_joint_min_rad'),
                value_type=float,
            ),
            'elevator_joint_max_rad': ParameterValue(
                LaunchConfiguration('elevator_joint_max_rad'),
                value_type=float,
            ),
            'pusher_airspeed_ff': ParameterValue(
                LaunchConfiguration('pusher_airspeed_ff'), value_type=float),
            'pusher_airspeed_kp': ParameterValue(
                LaunchConfiguration('pusher_airspeed_kp'), value_type=float),
            'pusher_airspeed_ki': ParameterValue(
                LaunchConfiguration('pusher_airspeed_ki'), value_type=float),
            'pusher_throttle_min': ParameterValue(
                LaunchConfiguration('pusher_throttle_min'), value_type=float),
            'pusher_throttle_max': ParameterValue(
                LaunchConfiguration('pusher_throttle_max'), value_type=float),
            'pusher_handover_below_target_mps': ParameterValue(
                LaunchConfiguration('pusher_handover_below_target'),
                value_type=float,
            ),
            'pusher_throttle_slew_up_per_s': ParameterValue(
                LaunchConfiguration('pusher_throttle_slew_up'), value_type=float),
            'pusher_throttle_slew_down_per_s': ParameterValue(
                LaunchConfiguration('pusher_throttle_slew_down'), value_type=float),
            'va_filter_alpha': ParameterValue(
                LaunchConfiguration('va_filter_alpha'), value_type=float),
            'va_runaway_margin_mps': ParameterValue(
                LaunchConfiguration('va_runaway_margin'), value_type=float),
            'va_runaway_dwell_s': ParameterValue(
                LaunchConfiguration('va_runaway_dwell'), value_type=float),
            'va_altitude_abort_error_m': ParameterValue(
                LaunchConfiguration('va_altitude_abort_error'), value_type=float),
            'va_vertical_speed_abort_mps': ParameterValue(
                LaunchConfiguration('va_vertical_speed_abort'), value_type=float),
            'va_vertical_abort_min_airspeed_mps': ParameterValue(
                LaunchConfiguration('va_vertical_abort_min_airspeed'),
                value_type=float,
            ),
            'transition_stability_dwell_s': ParameterValue(
                LaunchConfiguration('transition_stability_dwell'),
                value_type=float,
            ),
            'transition_altitude_tolerance_m': ParameterValue(
                LaunchConfiguration('transition_altitude_tolerance'),
                value_type=float,
            ),
            'transition_vertical_speed_tolerance_mps': ParameterValue(
                LaunchConfiguration('transition_vertical_speed_tolerance'),
                value_type=float,
            ),
            'transition_groundspeed_tolerance_mps': ParameterValue(
                LaunchConfiguration('transition_groundspeed_tolerance'),
                value_type=float,
            ),
            'alpha_min_airspeed_mps': ParameterValue(
                LaunchConfiguration('alpha_min_airspeed'), value_type=float),
        }],
    )
    command = Node(
        package='vtol_px4_control',
        executable='vtol_command_node',
        name='ca_lsc_command',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'auto_start_mission': True,
            'takeoff_altitude_m': ParameterValue(
                LaunchConfiguration('takeoff_altitude'), value_type=float),
            'takeoff_completion_tolerance_m': ParameterValue(
                LaunchConfiguration('takeoff_completion_tolerance'),
                value_type=float,
            ),
            'takeoff_completion_vertical_speed_tolerance_mps': ParameterValue(
                LaunchConfiguration('takeoff_completion_vertical_speed_tolerance'),
                value_type=float,
            ),
            'hold_mc_s': ParameterValue(
                LaunchConfiguration('hold_mc_s'), value_type=float),
            'require_transition_stability': True,
            'transition_stability_dwell_s': ParameterValue(
                LaunchConfiguration('transition_stability_dwell'),
                value_type=float,
            ),
            'transition_altitude_tolerance_m': ParameterValue(
                LaunchConfiguration('transition_altitude_tolerance'),
                value_type=float,
            ),
            'transition_vertical_speed_tolerance_mps': ParameterValue(
                LaunchConfiguration('transition_vertical_speed_tolerance'),
                value_type=float,
            ),
            'transition_groundspeed_tolerance_mps': ParameterValue(
                LaunchConfiguration('transition_groundspeed_tolerance'),
                value_type=float,
            ),
            'hold_mc_timeout_s': ParameterValue(
                LaunchConfiguration('hold_mc_timeout'), value_type=float),
            'hold_fw_s': ParameterValue(
                LaunchConfiguration('hold_fw_s'), value_type=float),
            'hold_after_mc_s': 3.0,
            'forward_distance_m': ParameterValue(
                LaunchConfiguration('forward_distance_m'), value_type=float),
            'max_horizontal_speed_mps': ParameterValue(
                LaunchConfiguration('max_horizontal_speed'), value_type=float),
            'max_vertical_speed_mps': ParameterValue(
                LaunchConfiguration('max_vertical_speed'), value_type=float),
            'fw_min_forward_speed_mps': ParameterValue(
                LaunchConfiguration('fw_min_forward_speed'), value_type=float),
            'require_a0_rl_action_ready': ParameterValue(
                LaunchConfiguration('require_a0_rl_action_ready'),
                value_type=bool,
            ),
            'a0_rl_action_ready_timeout_s': ParameterValue(
                LaunchConfiguration('a0_rl_action_ready_timeout'),
                value_type=float,
            ),
            'transition_timeout_s': 25.0,
            'takeoff_timeout_s': 75.0,
            'landing_timeout_s': 90.0,
        }],
    )

    optional_nodes = []
    wing_force_gt_enabled = LaunchConfiguration(
        'wing_force_gt_enabled'
    ).perform(context).strip().lower() in {'1', 'true', 'yes', 'on'}
    if wing_force_gt_enabled:
        optional_nodes.append(Node(
            package='vtol_px4_control',
            executable='gz_wing_force_bridge',
            name='ca_lsc_gz_wing_force_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'left_gz_topic': LaunchConfiguration(
                    'wing_force_left_gz_topic'),
                'right_gz_topic': LaunchConfiguration(
                    'wing_force_right_gz_topic'),
                'elevator_gz_topic': LaunchConfiguration(
                    'elevator_joint_position_gz_topic'),
                'elevator_wrench_gz_topic': LaunchConfiguration(
                    'elevator_wrench_gz_topic'),
                'elevator_effective_angle_gz_topic': LaunchConfiguration(
                    'elevator_effective_angle_gz_topic'),
                'elevator_dither_scale_gz_topic': LaunchConfiguration(
                    'elevator_dither_scale_gz_topic'),
            }],
            additional_env={'GZ_PARTITION': gz_partition},
        ))
    if LaunchConfiguration('schedule_mode').perform(context) == 'gust_target':
        optional_nodes.append(Node(
            package='vtol_px4_control',
            executable='gz_wind_bridge',
            name='ca_lsc_gz_wind_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'gz_topic': f'/world/{world_name}/wind/',
                'command_timeout_s': ParameterValue(
                    LaunchConfiguration('gust_status_timeout'),
                    value_type=float,
                ),
                'publish_rate_hz': 50.0,
            }],
            additional_env={'GZ_PARTITION': gz_partition},
        ))

    return [
        gazebo,
        agent,
        clock_bridge,
        state_monitor,
        recorder,
        *optional_nodes,
        TimerAction(period=3.0, actions=[px4]),
        TimerAction(period=6.0, actions=parameters),
        TimerAction(period=8.0, actions=[command]),
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory('ca_lsc_transition'))
    default_px4 = Path(os.environ.get(
        'PX4_AUTOPILOT_DIR',
        str(Path.home() / 'ca_lsc_td3' / 'PX4-Autopilot'),
    )).expanduser()
    # `make ... BUILD_DIR=build_ca_make` writes the current artifacts directly
    # into build_ca_make.  Do not use the copied, stale
    # build_ca_make/px4_sitl_default tree.
    default_build = default_px4 / 'build_ca_make'
    return LaunchDescription([
        DeclareLaunchArgument('px4_dir', default_value=str(default_px4)),
        DeclareLaunchArgument('px4_build_dir', default_value=str(default_build)),
        DeclareLaunchArgument(
            'world', default_value=str(package_share / 'worlds' / 'nominal_empty.sdf')),
        DeclareLaunchArgument('world_name', default_value='default'),
        DeclareLaunchArgument('model_name', default_value='standard_vtol'),
        DeclareLaunchArgument('gz_partition', default_value='ca_lsc_td3_nominal'),
        DeclareLaunchArgument('extra_gz_resource_path', default_value=''),
        DeclareLaunchArgument('sim_speed_factor', default_value='1.0'),
        DeclareLaunchArgument('px4_sim_speed_factor', default_value='1.0'),
        DeclareLaunchArgument('condition_id', default_value='nominal'),
        DeclareLaunchArgument('wind_model', default_value='none'),
        DeclareLaunchArgument('wind_cmd_e_enu', default_value='0.0'),
        DeclareLaunchArgument('wind_cmd_n_enu', default_value='0.0'),
        DeclareLaunchArgument('wind_cmd_u_enu', default_value='0.0'),
        DeclareLaunchArgument('gust_amplitude', default_value='0.0'),
        DeclareLaunchArgument('gust_delay', default_value='0.5'),
        DeclareLaunchArgument('gust_duration', default_value='2.0'),
        DeclareLaunchArgument('gust_recovery', default_value='2.0'),
        DeclareLaunchArgument('gust_status_timeout', default_value='0.25'),
        DeclareLaunchArgument('airspeed_source', default_value='native'),
        DeclareLaunchArgument('mass_scale', default_value='1.0'),
        DeclareLaunchArgument('nominal_model_mass_kg', default_value='nan'),
        DeclareLaunchArgument('actual_model_mass_kg', default_value='nan'),
        DeclareLaunchArgument('payload_mass_kg', default_value='0.0'),
        DeclareLaunchArgument('wing_force_gt_enabled', default_value='false'),
        DeclareLaunchArgument('wing_force_gt_timeout', default_value='0.25'),
        DeclareLaunchArgument(
            'wing_force_left_gz_topic',
            default_value='/ca_lsc/f3/wing_left/lift',
        ),
        DeclareLaunchArgument(
            'wing_force_right_gz_topic',
            default_value='/ca_lsc/f3/wing_right/lift',
        ),
        DeclareLaunchArgument(
            'elevator_joint_position_gz_topic',
            default_value='/ca_lsc/f3/elevator/joint_position',
        ),
        DeclareLaunchArgument(
            'elevator_wrench_gz_topic',
            default_value='',
        ),
        DeclareLaunchArgument(
            'elevator_effective_angle_gz_topic',
            default_value='',
        ),
        DeclareLaunchArgument(
            'elevator_dither_scale_gz_topic',
            default_value='',
        ),
        DeclareLaunchArgument('eta_l_estimated_mass_kg', default_value='nan'),
        DeclareLaunchArgument(
            'eta_l_desired_vertical_acceleration', default_value='0.0'),
        DeclareLaunchArgument(
            'eta_c_pressure_gate_lower', default_value='15.05125'),
        DeclareLaunchArgument(
            'eta_c_pressure_gate_upper', default_value='60.205'),
        DeclareLaunchArgument('f3b2_test_mode', default_value=''),
        DeclareLaunchArgument(
            'f3b2_shadow_demand_offset', default_value='0.0'),
        DeclareLaunchArgument(
            'f3b2_shadow_demand_for_eta_c', default_value='nan'),
        DeclareLaunchArgument('f3b2_elevator_bias', default_value='0.0'),
        DeclareLaunchArgument('f3b2_stage_index', default_value='-1.0'),
        DeclareLaunchArgument(
            'f3b2_gt_dmoment_ddelta_per_q_m3', default_value='nan'),
        DeclareLaunchArgument('elevator_id_dither_enabled', default_value='false'),
        DeclareLaunchArgument('gz_verbosity', default_value='2'),
        DeclareLaunchArgument('agent_port', default_value='8888'),
        DeclareLaunchArgument('takeoff_altitude', default_value='50.0'),
        DeclareLaunchArgument('takeoff_completion_tolerance', default_value='0.5'),
        DeclareLaunchArgument(
            'takeoff_completion_vertical_speed_tolerance',
            default_value='0.3',
        ),
        DeclareLaunchArgument('hold_mc_s', default_value='5.0'),
        DeclareLaunchArgument('transition_stability_dwell', default_value='2.0'),
        DeclareLaunchArgument('transition_altitude_tolerance', default_value='1.0'),
        DeclareLaunchArgument(
            'transition_vertical_speed_tolerance', default_value='0.2'),
        DeclareLaunchArgument(
            'transition_groundspeed_tolerance', default_value='0.2'),
        DeclareLaunchArgument('hold_mc_timeout', default_value='45.0'),
        DeclareLaunchArgument('hold_fw_s', default_value='8.0'),
        DeclareLaunchArgument('forward_distance_m', default_value='500.0'),
        DeclareLaunchArgument('max_horizontal_speed', default_value='15.0'),
        DeclareLaunchArgument('max_vertical_speed', default_value='1.5'),
        DeclareLaunchArgument('fw_min_forward_speed', default_value='15.0'),
        DeclareLaunchArgument('fw_airspeed_min', default_value='12.0'),
        DeclareLaunchArgument('fw_airspeed_trim', default_value='15.0'),
        DeclareLaunchArgument('fw_airspeed_max', default_value='20.0'),
        DeclareLaunchArgument('fw_throttle_max', default_value='0.45'),
        DeclareLaunchArgument('fw_climb_rate_max', default_value='3.0'),
        DeclareLaunchArgument('vt_blend_airspeed', default_value='8.0'),
        DeclareLaunchArgument('vt_transition_airspeed', default_value='13.0'),
        DeclareLaunchArgument(
            'lambda_attitude_blend_start', default_value='0.1'),
        DeclareLaunchArgument(
            'lambda_attitude_blend_full', default_value='0.9'),
        DeclareLaunchArgument(
            'physics_prior_eta_l_lower', default_value='0.15'),
        DeclareLaunchArgument(
            'physics_prior_eta_l_upper', default_value='0.85'),
        DeclareLaunchArgument(
            'physics_prior_eta_c_lower', default_value='0.20'),
        DeclareLaunchArgument(
            'physics_prior_eta_c_upper', default_value='0.80'),
        DeclareLaunchArgument(
            'physics_prior_hard_eta_l_lower', default_value='0.05'),
        DeclareLaunchArgument(
            'physics_prior_hard_eta_l_upper', default_value='0.35'),
        DeclareLaunchArgument(
            'physics_prior_hard_eta_c_lower', default_value='0.05'),
        DeclareLaunchArgument(
            'physics_prior_hard_eta_c_upper', default_value='0.35'),
        DeclareLaunchArgument(
            'physics_prior_altitude_drop_soft', default_value='1.0'),
        DeclareLaunchArgument(
            'physics_prior_altitude_drop_hard', default_value='4.0'),
        DeclareLaunchArgument(
            'physics_prior_descent_soft', default_value='0.5'),
        DeclareLaunchArgument(
            'physics_prior_descent_hard', default_value='1.5'),
        DeclareLaunchArgument(
            'physics_prior_alpha_soft', default_value='0.28'),
        DeclareLaunchArgument(
            'physics_prior_alpha_hard', default_value='0.42'),
        DeclareLaunchArgument(
            'physics_prior_unloading_rate', default_value='0.25'),
        DeclareLaunchArgument(
            'physics_prior_recovery_rate', default_value='1.0'),
        DeclareLaunchArgument(
            'physics_prior_emergency_recovery_rate', default_value='2.5'),
        DeclareLaunchArgument(
            'physics_prior_release_lambda', default_value='0.95'),
        DeclareLaunchArgument(
            'physics_prior_release_dwell', default_value='2.0'),
        DeclareLaunchArgument('elevator_joint_min_rad', default_value='-0.53'),
        DeclareLaunchArgument('elevator_joint_max_rad', default_value='0.53'),
        DeclareLaunchArgument(
            'vt_forward_transition_throttle', default_value='0.45'),
        DeclareLaunchArgument('schedule_mode', default_value='none'),
        DeclareLaunchArgument('a0_rl_command_timeout', default_value='0.35'),
        DeclareLaunchArgument('require_a0_rl_action_ready', default_value='false'),
        DeclareLaunchArgument('a0_rl_action_ready_timeout', default_value='0.5'),
        DeclareLaunchArgument('a0_rl_handover_lambda', default_value='0.95'),
        DeclareLaunchArgument('a0_rl_handover_dwell', default_value='1.0'),
        DeclareLaunchArgument('lambda_start_airspeed', default_value='8.0'),
        DeclareLaunchArgument('lambda_full_airspeed', default_value='16.0'),
        DeclareLaunchArgument('lambda_maximum', default_value='0.6'),
        DeclareLaunchArgument('lambda_target', default_value='0.5'),
        DeclareLaunchArgument('lambda_target_tolerance', default_value='0.03'),
        DeclareLaunchArgument('lambda_target_dwell', default_value='1.0'),
        DeclareLaunchArgument('va_target', default_value='12.0'),
        DeclareLaunchArgument('va_target_tolerance', default_value='0.3'),
        DeclareLaunchArgument('va_target_dwell', default_value='1.0'),
        DeclareLaunchArgument('va_measurement_s', default_value='3.0'),
        DeclareLaunchArgument(
            'va_measurement_min_band_fraction', default_value='0.9'),
        DeclareLaunchArgument(
            'va_measurement_max_abs_airspeed_error', default_value='0.6'),
        DeclareLaunchArgument('va_velocity_kp', default_value='0.8'),
        DeclareLaunchArgument('va_velocity_min', default_value='8.0'),
        DeclareLaunchArgument('va_velocity_max', default_value='15.0'),
        DeclareLaunchArgument('va_altitude_kp', default_value='0.35'),
        DeclareLaunchArgument(
            'va_altitude_vertical_speed_kd', default_value='0.5'),
        DeclareLaunchArgument(
            'va_altitude_velocity_limit', default_value='1.5'),
        DeclareLaunchArgument('pusher_airspeed_ff', default_value='0.25'),
        DeclareLaunchArgument('pusher_airspeed_kp', default_value='0.08'),
        DeclareLaunchArgument('pusher_airspeed_ki', default_value='0.03'),
        DeclareLaunchArgument('pusher_throttle_min', default_value='0.0'),
        DeclareLaunchArgument('pusher_throttle_max', default_value='0.45'),
        DeclareLaunchArgument(
            'pusher_handover_below_target', default_value='1.0'),
        DeclareLaunchArgument('pusher_throttle_slew_up', default_value='0.5'),
        DeclareLaunchArgument('pusher_throttle_slew_down', default_value='1.5'),
        DeclareLaunchArgument('va_filter_alpha', default_value='0.35'),
        DeclareLaunchArgument('va_runaway_margin', default_value='3.0'),
        DeclareLaunchArgument('va_runaway_dwell', default_value='0.5'),
        DeclareLaunchArgument('va_altitude_abort_error', default_value='5.0'),
        DeclareLaunchArgument('va_vertical_speed_abort', default_value='2.0'),
        DeclareLaunchArgument(
            'va_vertical_abort_min_airspeed', default_value='6.0'),
        DeclareLaunchArgument('alpha_min_airspeed', default_value='5.0'),
        DeclareLaunchArgument('vt_unload_test_enable', default_value='0'),
        DeclareLaunchArgument(
            'vt_unload_altitude_pitch_kp', default_value='2.0'),
        DeclareLaunchArgument(
            'vt_unload_vertical_speed_pitch_kd', default_value='2.0'),
        DeclareLaunchArgument('vt_unload_pitch_min', default_value='-12.0'),
        DeclareLaunchArgument('vt_unload_pitch_max', default_value='10.0'),
        DeclareLaunchArgument(
            'output_csv', default_value='/tmp/ca_lsc_td3/transition.csv'),
        DeclareLaunchArgument(
            'px4_workdir', default_value='/tmp/ca_lsc_td3/px4_workdir'),
        OpaqueFunction(function=_setup),
    ])
