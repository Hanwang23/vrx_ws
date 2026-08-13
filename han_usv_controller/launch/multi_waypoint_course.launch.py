from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from pathlib import Path
import shutil
import tempfile

from han_usv_controller.multi_waypoint_course import write_course_world


_generated_world_directory = None


def _remove_generated_world(_event, _context):
    global _generated_world_directory
    if _generated_world_directory is None:
        return
    try:
        shutil.rmtree(_generated_world_directory, ignore_errors=True)
    finally:
        _generated_world_directory = None


def _launch_setup(context):
    global _generated_world_directory
    share = get_package_share_directory('han_usv_controller')
    template_world = Path(
        LaunchConfiguration('template_world').perform(context))
    _generated_world_directory = tempfile.mkdtemp(
        prefix='han_multi_waypoint_')
    generated_world = Path(_generated_world_directory) / 'wayfinding_task.sdf'
    write_course_world(template_world, generated_world)

    simulation_launch = os.path.join(
        share, 'launch', 'simulation.launch.py')
    forwarded = {
        name: LaunchConfiguration(name)
        for name in (
            'config', 'headless', 'rviz', 'rviz_delay', 'rviz_config',
            'timed_competition', 'competition_mode', 'robot_localization',
            'robot_localization_config',
        )
    }
    forwarded['world'] = str(generated_world)
    forwarded['preserve_waypoint_order'] = 'True'
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch),
        launch_arguments=forwarded.items(),
    )
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=_remove_generated_world)),
        simulation,
    ]


def generate_launch_description():
    share = get_package_share_directory('han_usv_controller')
    return LaunchDescription([
        DeclareLaunchArgument(
            'template_world',
            default_value=os.path.join(share, 'worlds', 'wayfinding_task.sdf')),
        DeclareLaunchArgument(
            'config',
            default_value=os.path.join(share, 'config', 'controller.yaml')),
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
            default_value=os.path.join(
                share, 'config', 'robot_localization.yaml')),
        OpaqueFunction(function=_launch_setup),
    ])
