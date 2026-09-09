#!/usr/bin/env python3
"""Start the complete single-vehicle VTOL simulation stack."""

import math
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    Shutdown,
    TimerAction,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def _launch_setup(context):
    bringup_share = Path(
        get_package_share_directory('vtol_bringup')
    )
    simulation_share = Path(
        get_package_share_directory('vtol_simulation')
    )

    px4_dir = Path(
        LaunchConfiguration('px4_dir').perform(context)
    ).expanduser().resolve()
    world_file = Path(
        LaunchConfiguration('world').perform(context)
    ).expanduser().resolve()
    world_name = LaunchConfiguration('world_name').perform(context)
    model_name = LaunchConfiguration('model_name').perform(context)
    spawn_north = float(
        LaunchConfiguration('spawn_north').perform(context)
    )
    spawn_east = float(
        LaunchConfiguration('spawn_east').perform(context)
    )
    spawn_yaw = float(
        LaunchConfiguration('spawn_yaw').perform(context)
    )
    goal_north = float(
        LaunchConfiguration('goal_north').perform(context)
    )
    goal_east = float(
        LaunchConfiguration('goal_east').perform(context)
    )
    goal_down = float(
        LaunchConfiguration('goal_down').perform(context)
    )
    map_metadata = LaunchConfiguration('map_metadata').perform(context)
    safe_strip_depth = float(
        LaunchConfiguration('safe_strip_depth').perform(context)
    )
    agent_port = LaunchConfiguration('agent_port').perform(context)
    agent_verbosity = LaunchConfiguration('agent_verbosity').perform(context)
    gz_partition = LaunchConfiguration('gz_partition').perform(context)
    headless = _as_bool(
        LaunchConfiguration('headless').perform(context)
    )
    start_paused = _as_bool(
        LaunchConfiguration('start_paused').perform(context)
    )
    start_gazebo = _as_bool(
        LaunchConfiguration('start_gazebo').perform(context)
    )
    start_episode_stack = _as_bool(
        LaunchConfiguration('start_episode_stack').perform(context)
    )
    px4_sim_speed_factor = LaunchConfiguration(
        'px4_sim_speed_factor'
    ).perform(context)
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'),
        value_type=bool,
    )

    px4_models = px4_dir / 'Tools' / 'simulation' / 'gz' / 'models'
    px4_worlds = px4_dir / 'Tools' / 'simulation' / 'gz' / 'worlds'
    custom_models = simulation_share / 'models'
    resource_path = ':'.join(
        str(path)
        for path in (custom_models, px4_models, px4_worlds)
    )

    px4_binary = px4_dir / 'build' / 'px4_sitl_default' / 'bin' / 'px4'
    px4_rcs = (
        px4_dir
        / 'build'
        / 'px4_sitl_default'
        / 'etc'
        / 'init.d-posix'
        / 'rcS'
    )
    px4_romfs = px4_dir / 'ROMFS' / 'px4fmu_common'
    px4_workdir = px4_dir / 'build' / 'px4_sitl_default'

    required_paths = (
        world_file,
        px4_binary,
        px4_rcs,
        px4_romfs,
        custom_models,
    )
    missing = [str(path) for path in required_paths if not path.exists()]
    if missing:
        raise RuntimeError(
            'VTOL bringup is missing required paths: ' + ', '.join(missing)
        )

    gz_command = ['gz', 'sim']
    if headless:
        gz_command.append('-s')
    if not start_paused:
        gz_command.append('-r')
    gz_command.extend(
        [
            '-v',
            LaunchConfiguration('gz_verbosity').perform(context),
            str(world_file),
        ]
    )

    clean_plugin_path = os.environ.get('LD_LIBRARY_PATH', '')
    gz_environment = {
        'GZ_PARTITION': gz_partition,
        'GZ_SIM_RESOURCE_PATH': resource_path,
        'GZ_SIM_SYSTEM_PLUGIN_PATH': clean_plugin_path,
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': clean_plugin_path,
    }
    gazebo = ExecuteProcess(
        cmd=gz_command,
        name='vtol_gazebo',
        output='screen',
        emulate_tty=True,
        additional_env=gz_environment,
        on_exit=Shutdown(reason='Gazebo exited'),
    )

    # The aircraft is included in the world so PX4 can attach in standalone
    # mode. Move it to this episode's sampled position before PX4 starts and
    # establishes its local estimator origin.
    relocate_aircraft = ExecuteProcess(
        cmd=[
            'gz',
            'service',
            '-s',
            f'/world/{world_name}/set_pose',
            '--reqtype',
            'gz.msgs.Pose',
            '--reptype',
            'gz.msgs.Boolean',
            '--timeout',
            '5000',
            '--req',
            (
                f'name: "{model_name}", position: '
                f'{{x: {spawn_east}, y: {spawn_north}, z: 0.3}}, '
                'orientation: {'
                f'z: {math.sin(spawn_yaw / 2.0)}, '
                f'w: {math.cos(spawn_yaw / 2.0)}'
                '}'
            ),
        ],
        name='vtol_episode_relocation',
        output='screen',
        additional_env={'GZ_PARTITION': gz_partition},
    )

    relocate_goal = ExecuteProcess(
        cmd=[
            'gz',
            'service',
            '-s',
            f'/world/{world_name}/set_pose',
            '--reqtype',
            'gz.msgs.Pose',
            '--reptype',
            'gz.msgs.Boolean',
            '--timeout',
            '5000',
            '--req',
            (
                # This is only a Gazebo ground cue.  Keeping it at the
                # commanded flight altitude makes the horizontal GPU lidar
                # report the goal as an obstacle.  RViz renders the actual
                # three-dimensional goal independently from goal_down.
                'name: "goal_marker", position: '
                f'{{x: {goal_east}, y: {goal_north}, '
                'z: 0.04}, orientation: {w: 1.0}'
            ),
        ],
        name='vtol_goal_relocation',
        output='screen',
        additional_env={'GZ_PARTITION': gz_partition},
    )

    resume_world = ExecuteProcess(
        cmd=[
            'gz',
            'service',
            '-s',
            f'/world/{world_name}/control',
            '--reqtype',
            'gz.msgs.WorldControl',
            '--reptype',
            'gz.msgs.Boolean',
            '--timeout',
            '5000',
            '--req',
            'pause: false',
        ],
        name='vtol_episode_resume',
        output='screen',
        additional_env={'GZ_PARTITION': gz_partition},
    )

    agent = ExecuteProcess(
        cmd=[
            'MicroXRCEAgent',
            'udp4',
            '-p',
            agent_port,
            '-v',
            agent_verbosity,
        ],
        name='micro_xrce_dds_agent',
        output='screen',
        emulate_tty=True,
    )

    px4_environment = {
        **gz_environment,
        'PX4_GZ_STANDALONE': '1',
        'PX4_GZ_WORLD': world_name,
        'PX4_GZ_MODEL_NAME': model_name,
        'PX4_SYS_AUTOSTART': '4004',
        'PX4_SIM_SPEED_FACTOR': px4_sim_speed_factor,
        'PX4_UXRCE_DDS_SYNCT': '0',
    }
    px4 = ExecuteProcess(
        cmd=[
            str(px4_binary),
            '-d',
            '-s',
            str(px4_rcs),
            str(px4_romfs),
            '-i',
            '0',
            '-w',
            str(px4_workdir),
        ],
        name='px4_sitl',
        output='screen',
        emulate_tty=True,
        additional_env=px4_environment,
    )

    def px4_param_override(name: str, value: str) -> ExecuteProcess:
        return ExecuteProcess(
            cmd=[
                str(px4_workdir / 'bin' / 'px4-param'),
                '--instance',
                '0',
                'set',
                name,
                value,
            ],
            name=f'px4_param_{name.lower()}',
            cwd=str(px4_workdir),
            output='screen',
        )

    # The stock 4004 airframe uses an aggressive 0.75 transition throttle.
    # A repeatable navigation baseline targets 15 m/s and keeps both the
    # transition and TECS throttle envelope conservative.
    px4_speed_parameters = [
        px4_param_override('UXRCE_DDS_SYNCT', '0'),
        px4_param_override('FW_AIRSPD_MIN', '12.0'),
        px4_param_override('FW_AIRSPD_TRIM', '15.0'),
        px4_param_override('FW_AIRSPD_MAX', '20.0'),
        px4_param_override('FW_THR_MAX', '0.45'),
        px4_param_override('FW_T_CLMB_MAX', '3.0'),
        px4_param_override('VT_ARSP_TRANS', '13.0'),
        px4_param_override('VT_F_TRANS_THR', '0.45'),
    ]

    bridge_config = (
        simulation_share / 'config' / 'standard_vtol_depth_bridge.yaml'
    )
    image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='vtol_sensor_bridge',
        output='screen',
        parameters=[{'config_file': str(bridge_config)}],
        additional_env={'GZ_PARTITION': gz_partition},
        condition=IfCondition(LaunchConfiguration('perception')),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='vtol_clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        additional_env={'GZ_PARTITION': gz_partition},
    )

    world_control_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='vtol_world_control_bridge',
        output='screen',
        arguments=[
            (
                f'/world/{world_name}/control'
                '@ros_gz_interfaces/srv/ControlWorld'
            )
        ],
        additional_env={'GZ_PARTITION': gz_partition},
    )

    state_monitor = Node(
        package='vtol_px4_control',
        executable='vtol_state_monitor',
        name='vtol_state_monitor',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
            }
        ],
    )

    command_node = Node(
        package='vtol_px4_control',
        executable='vtol_command_node',
        name='vtol_command_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'auto_start_mission': ParameterValue(
                    LaunchConfiguration('auto_mission'),
                    value_type=bool,
                ),
                'takeoff_altitude_m': ParameterValue(
                    LaunchConfiguration('takeoff_altitude'),
                    value_type=float,
                ),
                'max_horizontal_speed_mps': ParameterValue(
                    LaunchConfiguration('max_horizontal_speed'),
                    value_type=float,
                ),
                'max_vertical_speed_mps': ParameterValue(
                    LaunchConfiguration('max_vertical_speed'),
                    value_type=float,
                ),
                'max_yaw_rate_rps': ParameterValue(
                    LaunchConfiguration('max_yaw_rate'),
                    value_type=float,
                ),
                'fw_min_forward_speed_mps': ParameterValue(
                    LaunchConfiguration('fw_min_forward_speed'),
                    value_type=float,
                ),
                'command_period_ms': ParameterValue(
                    LaunchConfiguration('command_period_ms'),
                    value_type=float,
                ),
            }
        ],
    )

    perception_node = Node(
        package='vtol_perception',
        executable='depth_observation_node',
        name='depth_observation_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'num_sectors': ParameterValue(
                    LaunchConfiguration('depth_sectors'),
                    value_type=int,
                ),
                'max_range_m': ParameterValue(
                    LaunchConfiguration('depth_max_range'),
                    value_type=float,
                ),
            }
        ],
        condition=IfCondition(LaunchConfiguration('depth_processing')),
    )

    training_visualization = Node(
        package='vtol_visualization',
        executable='training_visualization_node',
        name='training_visualization_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'max_depth_m': ParameterValue(
                    LaunchConfiguration('depth_max_range'),
                    value_type=float,
                ),
                'area_length_m': ParameterValue(
                    LaunchConfiguration('area_length'),
                    value_type=float,
                ),
                'area_width_m': ParameterValue(
                    LaunchConfiguration('area_width'),
                    value_type=float,
                ),
                'safe_strip_depth_m': safe_strip_depth,
                'spawn_north_m': spawn_north,
                'spawn_east_m': spawn_east,
                'goal_north_m': goal_north,
                'goal_east_m': goal_east,
                'goal_down_m': goal_down,
                'map_metadata_path': map_metadata,
            }
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='vtol_training_rviz',
        output='screen',
        arguments=[
            '-d',
            str(bringup_share / 'config' / 'training.rviz'),
        ],
        parameters=[
            {
                'use_sim_time': use_sim_time,
            }
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    dashboard = Node(
        package='vtol_perception',
        executable='training_dashboard_node',
        name='training_dashboard_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'max_range_m': ParameterValue(
                    LaunchConfiguration('depth_max_range'),
                    value_type=float,
                ),
                'area_length_m': ParameterValue(
                    LaunchConfiguration('area_length'),
                    value_type=float,
                ),
                'area_width_m': ParameterValue(
                    LaunchConfiguration('area_width'),
                    value_type=float,
                ),
                'spawn_north_m': spawn_north,
                'spawn_east_m': spawn_east,
                'goal_north_m': goal_north,
                'goal_east_m': goal_east,
            }
        ],
        condition=IfCondition(LaunchConfiguration('dashboard')),
    )

    actions = []
    if start_gazebo:
        actions.append(gazebo)
    if start_episode_stack:
        actions.extend([
            agent,
            image_bridge,
            clock_bridge,
            world_control_bridge,
            state_monitor,
            command_node,
            perception_node,
            training_visualization,
            rviz,
            dashboard,
            TimerAction(
                period=2.0,
                actions=[relocate_aircraft, relocate_goal],
            ),
            TimerAction(
                period=3.0,
                actions=[resume_world],
            ),
            TimerAction(
                period=4.0,
                actions=[px4],
            ),
            TimerAction(
                period=8.0,
                actions=px4_speed_parameters,
            ),
        ])
    if not actions:
        raise RuntimeError(
            'at least one of start_gazebo/start_episode_stack must be true'
        )
    return actions


def generate_launch_description():
    simulation_share = Path(
        get_package_share_directory('vtol_simulation')
    )
    default_world = (
        simulation_share / 'worlds' / 'vtol_random_field_preview.sdf'
    )
    default_map_metadata = (
        simulation_share / 'worlds' / 'vtol_random_field_preview.json'
    )
    default_px4 = os.environ.get(
        'PX4_AUTOPILOT_DIR',
        str(Path.home() / 'vtol_sim' / 'PX4-Autopilot'),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'px4_dir',
                default_value=default_px4,
                description='PX4-Autopilot source directory.',
            ),
            DeclareLaunchArgument(
                'world',
                default_value=str(default_world),
                description='Absolute Gazebo world file.',
            ),
            DeclareLaunchArgument(
                'world_name',
                default_value='vtol_training',
                description='World name declared inside the SDF.',
            ),
            DeclareLaunchArgument(
                'model_name',
                default_value='standard_vtol_depth',
                description='Existing Gazebo model to which PX4 attaches.',
            ),
            DeclareLaunchArgument(
                'spawn_north',
                default_value='136.97802734375',
                description='Episode spawn north coordinate in metres.',
            ),
            DeclareLaunchArgument(
                'spawn_east',
                default_value='-480.0',
                description='Episode spawn east coordinate in metres.',
            ),
            DeclareLaunchArgument(
                'spawn_yaw',
                default_value='0.0',
                description='Episode spawn yaw in Gazebo ENU radians.',
            ),
            DeclareLaunchArgument(
                'goal_north',
                default_value='-30.560779571533203',
                description='World-aligned goal north coordinate.',
            ),
            DeclareLaunchArgument(
                'goal_east',
                default_value='480.0',
                description='World-aligned goal east coordinate.',
            ),
            DeclareLaunchArgument(
                'goal_down',
                default_value='-20.0',
                description='World-aligned NED goal down coordinate.',
            ),
            DeclareLaunchArgument(
                'map_metadata',
                default_value=str(default_map_metadata),
                description='Generated episode JSON used only by RViz.',
            ),
            DeclareLaunchArgument(
                'agent_port',
                default_value='8888',
                description='Micro XRCE-DDS UDP port.',
            ),
            DeclareLaunchArgument(
                'agent_verbosity',
                default_value='2',
                description='Micro XRCE-DDS Agent verbosity (0-6).',
            ),
            DeclareLaunchArgument(
                'gz_partition',
                default_value='vtol_navigation',
                description=(
                    'Gazebo Transport partition used to isolate this stack.'
                ),
            ),
            DeclareLaunchArgument(
                'headless',
                default_value='false',
                description='Run only the Gazebo server.',
            ),
            DeclareLaunchArgument(
                'start_paused',
                default_value='false',
                description=(
                    'Start Gazebo paused (PX4 needs an initial clock).'
                ),
            ),
            DeclareLaunchArgument(
                'gz_verbosity',
                default_value='2',
                description='Gazebo console verbosity.',
            ),
            DeclareLaunchArgument(
                'auto_mission',
                default_value='false',
                description='Automatically execute the VTOL demo mission.',
            ),
            DeclareLaunchArgument(
                'takeoff_altitude',
                default_value='20.0',
                description='Takeoff altitude above the local origin.',
            ),
            DeclareLaunchArgument(
                'depth_sectors',
                default_value='36',
                description='Fixed depth observation dimension.',
            ),
            DeclareLaunchArgument(
                'depth_max_range',
                default_value='100.0',
                description='Maximum usable front depth range in metres.',
            ),
            DeclareLaunchArgument(
                'perception',
                default_value='true',
                description=(
                    'Start the Gazebo image bridge and depth observation '
                    'node. Contract 13-A does not consume depth data.'
                ),
            ),
            DeclareLaunchArgument(
                'depth_processing',
                default_value='true',
                description=(
                    'Run depth-image sector processing. Contract 13-B uses '
                    'the bridged horizontal lidar directly.'
                ),
            ),
            DeclareLaunchArgument(
                'area_length',
                default_value='1000.0',
                description='Training area east-west length in metres.',
            ),
            DeclareLaunchArgument(
                'area_width',
                default_value='600.0',
                description='Training area north-south width in metres.',
            ),
            DeclareLaunchArgument(
                'safe_strip_depth',
                default_value='40.0',
                description='Obstacle-free depth at both short ends.',
            ),
            DeclareLaunchArgument(
                'max_horizontal_speed',
                default_value='15.0',
                description='Command-node horizontal velocity limit.',
            ),
            DeclareLaunchArgument(
                'max_vertical_speed',
                default_value='1.5',
                description='Command-node vertical velocity limit.',
            ),
            DeclareLaunchArgument(
                'max_yaw_rate',
                default_value='0.25',
                description='Command-node yaw-rate limit.',
            ),
            DeclareLaunchArgument(
                'fw_min_forward_speed',
                default_value='15.0',
                description='Fixed-wing minimum commanded forward speed.',
            ),
            DeclareLaunchArgument(
                'command_period_ms',
                default_value='50.0',
                description=(
                    'Simulation-time Offboard setpoint publish period. '
                    'Keep this fixed when Gazebo real_time_factor changes.'
                ),
            ),
            DeclareLaunchArgument(
                'px4_sim_speed_factor',
                default_value='1.0',
                description=(
                    'Legacy PX4 timeout scaling factor. Keep 1.0 when ROS '
                    'nodes use Gazebo /clock.'
                ),
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description=(
                    'Use Gazebo /clock for ROS nodes. Required when the '
                    'world real_time_factor is not 1.'
                ),
            ),
            DeclareLaunchArgument(
                'start_gazebo',
                default_value='true',
                description='Start the Gazebo server/GUI in this launch.',
            ),
            DeclareLaunchArgument(
                'start_episode_stack',
                default_value='true',
                description='Start PX4, DDS, bridges and ROS episode nodes.',
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='false',
                description='Start RViz with the VTOL training scene.',
            ),
            DeclareLaunchArgument(
                'dashboard',
                default_value='false',
                description='Start the RGB and depth training dashboard.',
            ),
            UnsetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH'),
            UnsetEnvironmentVariable('GZ_SIM_RESOURCE_PATH'),
            SetEnvironmentVariable('PYTHONNOUSERSITE', '1'),
            OpaqueFunction(function=_launch_setup),
        ]
    )
