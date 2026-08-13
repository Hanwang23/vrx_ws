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

from han_usv_controller.buoy_course import LATTICE_STRESS_SPECS
from han_usv_controller.launch_helpers import resolve_gz_world_name


def _spawn_stress_barrier(context, model_file):
    requested_world = LaunchConfiguration('world').perform(context)
    override = LaunchConfiguration('gz_world_name').perform(context)
    gz_world_name = resolve_gz_world_name(requested_world, override)
    delay = LaunchConfiguration('stress_spawn_delay')
    actions = []
    for index, (name, _color, x, y) in enumerate(LATTICE_STRESS_SPECS):
        actions.append(TimerAction(
            period=PythonExpression([delay, ' + ', str(index * 0.35)]),
            actions=[Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_' + name,
                output='log',
                arguments=[
                    '-world', gz_world_name,
                    '-file', model_file,
                    '-name', name,
                    '-x', str(x),
                    '-y', str(y),
                    '-z', '0.0',
                ],
            )],
        ))
    return actions


def generate_launch_description():
    share = get_package_share_directory('han_usv_controller')
    buoy_launch = os.path.join(share, 'launch', 'buoy_course.launch.py')
    orange_model = os.path.join(
        share, 'models', 'han_round_buoy_orange', 'model.sdf')

    arguments = {
        name: LaunchConfiguration(name)
        for name in (
            'world', 'config', 'timed_competition', 'gz_world_name',
            'competition_mode', 'headless', 'rviz', 'rviz_delay',
            'rviz_config',
            'spawn_delay',
        )
    }
    course = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(buoy_launch),
        launch_arguments=arguments.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='wayfinding_task'),
        DeclareLaunchArgument(
            'config',
            default_value=os.path.join(share, 'config', 'controller.yaml')),
        DeclareLaunchArgument('timed_competition', default_value='False'),
        DeclareLaunchArgument('gz_world_name', default_value=''),
        DeclareLaunchArgument('competition_mode', default_value='False'),
        DeclareLaunchArgument('headless', default_value='False'),
        DeclareLaunchArgument('rviz', default_value='True'),
        DeclareLaunchArgument('rviz_delay', default_value='6.0'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(share, 'config', 'pointcloud.rviz')),
        DeclareLaunchArgument('spawn_delay', default_value='5.0'),
        DeclareLaunchArgument(
            'stress_spawn_delay',
            default_value='9.5',
            description='Delay the centerline barrier until the base course exists'),
        course,
        OpaqueFunction(
            function=_spawn_stress_barrier,
            kwargs={'model_file': orange_model},
        ),
    ])
