from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os

from codex_usv_controller.launch_helpers import resolve_gz_world_name
from codex_usv_controller.random_course import generate_random_buoy_layout


def _spawn_layout(context, model_files):
    seed = int(LaunchConfiguration('scenario_seed').perform(context))
    specs = generate_random_buoy_layout(seed)
    requested_world = LaunchConfiguration('world').perform(context)
    override = LaunchConfiguration('gz_world_name').perform(context)
    world_name = resolve_gz_world_name(requested_world, override)
    spawn_delay = LaunchConfiguration('spawn_delay')
    actions = []
    for index, (name, color, x, y) in enumerate(specs):
        actions.append(TimerAction(
            period=PythonExpression([spawn_delay, ' + ', str(index * 0.25)]),
            actions=[Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_' + name,
                output='log',
                arguments=[
                    '-world', world_name,
                    '-file', model_files[color],
                    '-name', name,
                    '-x', str(x),
                    '-y', str(y),
                    '-z', '0.0',
                ],
            )],
        ))
    return actions


def generate_launch_description():
    share = get_package_share_directory('codex_usv_controller')
    simulation_launch = os.path.join(share, 'launch', 'simulation.launch.py')
    model_files = {
        'red': os.path.join(
            share, 'models', 'codex_marker_buoy_red', 'model.sdf'),
        'green': os.path.join(
            share, 'models', 'codex_marker_buoy_green', 'model.sdf'),
        'orange': os.path.join(
            share, 'models', 'codex_round_buoy_orange', 'model.sdf'),
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
    return LaunchDescription([
        DeclareLaunchArgument('scenario_seed', default_value='1000'),
        DeclareLaunchArgument('world', default_value='wayfinding_task'),
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
        DeclareLaunchArgument('gz_world_name', default_value=''),
        DeclareLaunchArgument('spawn_delay', default_value='5.0'),
        simulation,
        OpaqueFunction(
            function=_spawn_layout, kwargs={'model_files': model_files}),
    ])
