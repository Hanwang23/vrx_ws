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

from codex_usv_controller.buoy_course import BUOY_SPECS
from codex_usv_controller.launch_helpers import resolve_gz_world_name


def _spawn_buoys(context, model_files):
    requested_world = LaunchConfiguration('world').perform(context)
    override = LaunchConfiguration('gz_world_name').perform(context)
    gz_world_name = resolve_gz_world_name(requested_world, override)
    spawn_delay = LaunchConfiguration('spawn_delay')

    spawn_timers = []
    for index, (name, color, x, y) in enumerate(BUOY_SPECS):
        spawner = Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_' + name,
            output='log',
            arguments=[
                '-world', gz_world_name,
                '-file', model_files[color],
                '-name', name,
                '-x', str(x),
                '-y', str(y),
                '-z', '0.0',
            ],
        )
        spawn_timers.append(TimerAction(
            period=PythonExpression([
                spawn_delay, ' + ', str(index * 0.25),
            ]),
            actions=[spawner],
        ))
    return spawn_timers


def generate_launch_description():
    controller_share = get_package_share_directory('codex_usv_controller')
    simulation_launch = os.path.join(
        controller_share, 'launch', 'simulation.launch.py')
    model_files = {
        'red': os.path.join(
            controller_share, 'models', 'codex_marker_buoy_red', 'model.sdf'),
        'green': os.path.join(
            controller_share, 'models', 'codex_marker_buoy_green', 'model.sdf'),
        'orange': os.path.join(
            controller_share, 'models', 'codex_round_buoy_orange', 'model.sdf'),
    }

    world = LaunchConfiguration('world')
    config = LaunchConfiguration('config')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    rviz_delay = LaunchConfiguration('rviz_delay')
    rviz_config = LaunchConfiguration('rviz_config')
    timed_competition = LaunchConfiguration('timed_competition')
    competition_mode = LaunchConfiguration('competition_mode')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch),
        launch_arguments={
            'world': world,
            'config': config,
            'headless': headless,
            'rviz': rviz,
            'rviz_delay': rviz_delay,
            'rviz_config': rviz_config,
            'timed_competition': timed_competition,
            'competition_mode': competition_mode,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='wayfinding_task',
            description='VRX world name; the buoy layout is designed for Wayfinding',
        ),
        DeclareLaunchArgument(
            'config',
            default_value=os.path.join(
                controller_share, 'config', 'controller.yaml'),
            description='Controller parameter YAML file',
        ),
        DeclareLaunchArgument(
            'timed_competition',
            default_value='False',
            description='Restore the original 300 s VRX Wayfinding timeout',
        ),
        DeclareLaunchArgument(
            'gz_world_name',
            default_value='',
            description=(
                'Override the internal Gazebo world name; by default it is '
                'derived from world'),
        ),
        DeclareLaunchArgument(
            'competition_mode',
            default_value='False',
            description='Disable VRX debug bridges like a real competition',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='False',
            description='Run Gazebo without its graphical client',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='True',
            description='Open the configured WAM-V point-cloud RViz view',
        ),
        DeclareLaunchArgument(
            'rviz_delay',
            default_value='6.0',
            description='Seconds to wait before opening RViz',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                controller_share, 'config', 'pointcloud.rviz'),
            description='RViz configuration shared by all learning courses',
        ),
        DeclareLaunchArgument(
            'spawn_delay',
            default_value='5.0',
            description='Seconds to wait for Gazebo before spawning extra buoys',
        ),
        simulation,
        OpaqueFunction(
            function=_spawn_buoys,
            kwargs={'model_files': model_files},
        ),
    ])
