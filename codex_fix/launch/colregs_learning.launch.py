from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os

from codex_usv_controller.buoy_course import COLREGS_LEARNING_BUOYS


def _spawn_learning_buoys(context, model_files):
    world_name = LaunchConfiguration('gz_world_name').perform(context)
    spawn_delay = LaunchConfiguration('buoy_spawn_delay')
    enabled = IfCondition(LaunchConfiguration('spawn_learning_buoys'))
    timers = []
    for index, (name, color, x, y) in enumerate(COLREGS_LEARNING_BUOYS):
        spawner = Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_' + name,
            condition=enabled,
            output='log',
            arguments=[
                '-world', world_name,
                '-file', model_files[color],
                '-name', name,
                '-x', str(x),
                '-y', str(y),
                '-z', '0.0',
            ],
        )
        timers.append(TimerAction(
            period=PythonExpression([
                spawn_delay, ' + ', str(index * 0.25),
            ]),
            actions=[spawner],
        ))
    return timers


def generate_launch_description():
    share = get_package_share_directory('codex_usv_controller')
    simulation_launch = os.path.join(share, 'launch', 'simulation.launch.py')
    model_file = os.path.join(
        share, 'models', 'codex_target_vessel', 'model.sdf')
    buoy_model_files = {
        'red': os.path.join(
            share, 'models', 'codex_marker_buoy_red', 'model.sdf'),
        'green': os.path.join(
            share, 'models', 'codex_marker_buoy_green', 'model.sdf'),
    }
    forwarded = {
        name: LaunchConfiguration(name)
        for name in (
            'world', 'config', 'headless', 'rviz', 'rviz_delay', 'rviz_config',
            'timed_competition', 'competition_mode', 'robot_localization',
            'robot_localization_config',
        )
    }
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch),
        launch_arguments=forwarded.items())
    world_name = LaunchConfiguration('gz_world_name')
    start_x = LaunchConfiguration('target_start_x')
    start_y = LaunchConfiguration('target_start_y')
    start_z = LaunchConfiguration('target_start_z')
    target_name = LaunchConfiguration('target_name')
    target_2_name = LaunchConfiguration('target_2_name')
    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='target_pose_service_bridge',
        arguments=[[
            '/world/', world_name,
            '/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ]],
        output='screen')
    spawn_target = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_colregs_target',
        arguments=[
            '-world', world_name,
            '-file', model_file,
            '-name', target_name,
            '-x', start_x,
            '-y', start_y,
            '-z', start_z,
        ],
        output='screen')
    moving_target = Node(
        package='codex_usv_controller',
        executable='moving_target',
        name='moving_target',
        parameters=[{
            'use_sim_time': True,
            'world_name': world_name,
            'entity_name': target_name,
            'start_x': start_x,
            'start_y': start_y,
            'start_z': start_z,
            'velocity_x': LaunchConfiguration('target_velocity_x'),
            'velocity_y': LaunchConfiguration('target_velocity_y'),
            'start_delay': LaunchConfiguration('target_motion_delay'),
            'motion_duration': LaunchConfiguration('target_motion_duration'),
        }],
        output='screen')
    spawn_target_2 = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_colregs_target_2',
        condition=IfCondition(LaunchConfiguration('spawn_second_target')),
        arguments=[
            '-world', world_name,
            '-file', model_file,
            '-name', target_2_name,
            '-x', LaunchConfiguration('target_2_start_x'),
            '-y', LaunchConfiguration('target_2_start_y'),
            '-z', start_z,
        ],
        output='screen')
    moving_target_2 = Node(
        package='codex_usv_controller',
        executable='moving_target',
        name='moving_target_2',
        condition=IfCondition(LaunchConfiguration('spawn_second_target')),
        parameters=[{
            'use_sim_time': True,
            'world_name': world_name,
            'entity_name': target_2_name,
            'start_x': LaunchConfiguration('target_2_start_x'),
            'start_y': LaunchConfiguration('target_2_start_y'),
            'start_z': start_z,
            'velocity_x': LaunchConfiguration('target_2_velocity_x'),
            'velocity_y': LaunchConfiguration('target_2_velocity_y'),
            'start_delay': LaunchConfiguration('target_2_motion_delay'),
            'motion_duration': LaunchConfiguration('target_2_motion_duration'),
        }],
        output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='wayfinding_task'),
        DeclareLaunchArgument('gz_world_name', default_value='wayfinding_task'),
        DeclareLaunchArgument(
            'config', default_value=os.path.join(share, 'config', 'controller.yaml')),
        DeclareLaunchArgument('headless', default_value='False'),
        DeclareLaunchArgument('rviz', default_value='True'),
        DeclareLaunchArgument('rviz_delay', default_value='6.0'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(share, 'config', 'pointcloud.rviz')),
        DeclareLaunchArgument('timed_competition', default_value='False'),
        DeclareLaunchArgument('competition_mode', default_value='False'),
        DeclareLaunchArgument('robot_localization', default_value='True'),
        DeclareLaunchArgument(
            'robot_localization_config',
            default_value=os.path.join(share, 'config', 'robot_localization.yaml')),
        DeclareLaunchArgument('target_name', default_value='codex_target_vessel'),
        DeclareLaunchArgument('target_start_x', default_value='-500.0'),
        DeclareLaunchArgument('target_start_y', default_value='205.0'),
        DeclareLaunchArgument('target_start_z', default_value='0.7'),
        DeclareLaunchArgument('target_velocity_x', default_value='-1.0'),
        DeclareLaunchArgument('target_velocity_y', default_value='0.0'),
        DeclareLaunchArgument('target_motion_delay', default_value='15.0'),
        DeclareLaunchArgument('target_motion_duration', default_value='90.0'),
        DeclareLaunchArgument('spawn_second_target', default_value='True'),
        DeclareLaunchArgument(
            'target_2_name', default_value='codex_target_vessel_southbound'),
        DeclareLaunchArgument('target_2_start_x', default_value='-505.0'),
        DeclareLaunchArgument('target_2_start_y', default_value='255.0'),
        DeclareLaunchArgument('target_2_velocity_x', default_value='0.0'),
        DeclareLaunchArgument('target_2_velocity_y', default_value='-0.7'),
        DeclareLaunchArgument('target_2_motion_delay', default_value='40.0'),
        DeclareLaunchArgument('target_2_motion_duration', default_value='120.0'),
        DeclareLaunchArgument('spawn_learning_buoys', default_value='True'),
        DeclareLaunchArgument('buoy_spawn_delay', default_value='6.5'),
        simulation,
        pose_bridge,
        TimerAction(period=5.0, actions=[spawn_target]),
        TimerAction(period=5.5, actions=[spawn_target_2]),
        TimerAction(period=7.0, actions=[moving_target]),
        TimerAction(period=7.5, actions=[moving_target_2]),
        OpaqueFunction(
            function=_spawn_learning_buoys,
            kwargs={'model_files': buoy_model_files},
        ),
    ])
