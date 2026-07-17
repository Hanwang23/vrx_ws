from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

from codex_usv_controller.launch_helpers import (
    acquire_simulation_lock,
    launch_flag,
    reject_running_gazebo,
    release_simulation_lock,
    resolve_simulation_world,
    simulation_lock_path,
)


_simulation_lock = None


def _release_launch_lock(_event, _context):
    global _simulation_lock
    if _simulation_lock is not None:
        release_simulation_lock(_simulation_lock)
        _simulation_lock = None


def _launch_setup(context):
    global _simulation_lock
    controller_share = get_package_share_directory('codex_usv_controller')
    if _simulation_lock is None:
        reject_running_gazebo()
        _simulation_lock = acquire_simulation_lock(
            simulation_lock_path(controller_share))
    vrx_share = get_package_share_directory('vrx_gz')
    requested_world = LaunchConfiguration('world').perform(context)
    timed_competition = LaunchConfiguration(
        'timed_competition').perform(context)
    simulation_world = resolve_simulation_world(
        controller_share, requested_world, timed_competition)
    config = LaunchConfiguration('config')
    robot_localization = LaunchConfiguration('robot_localization')
    robot_localization_config = LaunchConfiguration(
        'robot_localization_config')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    rviz_delay = LaunchConfiguration('rviz_delay')
    rviz_config = LaunchConfiguration('rviz_config')
    competition_mode = (
        'True'
        if launch_flag(LaunchConfiguration('competition_mode').perform(context))
        else 'False'
    )
    preserve_waypoint_order = launch_flag(
        LaunchConfiguration('preserve_waypoint_order').perform(context))

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vrx_share, 'launch', 'competition.launch.py')),
        launch_arguments={
            'world': simulation_world,
            'headless': headless,
            'competition_mode': competition_mode,
        }.items(),
    )
    controller = Node(
        package='codex_usv_controller',
        executable='autonomous_usv',
        name='autonomous_usv',
        output='screen',
        parameters=[
            config,
            {
                'navigation.nearest_neighbor_order': (
                    not preserve_waypoint_order),
            },
        ],
        emulate_tty=True,
        on_exit=Shutdown(reason='Autonomous controller exited'),
    )
    gnss_adapter = Node(
        package='codex_usv_controller',
        executable='gnss_odometry_adapter',
        name='gnss_odometry_adapter',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(robot_localization),
    )
    localization_filter = Node(
        package='robot_localization',
        executable='ekf_node',
        name='codex_ekf_filter',
        output='screen',
        parameters=[robot_localization_config],
        remappings=[('odometry/filtered', '/odometry/filtered')],
        condition=IfCondition(robot_localization),
    )
    pointcloud_viewer = Node(
        package='rviz2',
        executable='rviz2',
        name='usv_pointcloud_viewer',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(rviz),
    )

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=_release_launch_lock)),
        simulation,
        gnss_adapter,
        localization_filter,
        controller,
        TimerAction(period=rviz_delay, actions=[pointcloud_viewer]),
    ]


def generate_launch_description():
    controller_share = get_package_share_directory('codex_usv_controller')
    default_config = os.path.join(controller_share, 'config', 'controller.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='wayfinding_task',
            description='VRX world name',
        ),
        DeclareLaunchArgument(
            'timed_competition',
            default_value='False',
            description=(
                'Use the original 300 s Wayfinding competition world instead '
                'of the effectively unlimited learning world'),
        ),
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Controller parameter YAML file',
        ),
        DeclareLaunchArgument(
            'robot_localization',
            default_value='True',
            description='Start robot_localization with the GNSS adapter',
        ),
        DeclareLaunchArgument(
            'robot_localization_config',
            default_value=os.path.join(
                controller_share, 'config', 'robot_localization.yaml'),
            description='robot_localization EKF parameter YAML file',
        ),
        DeclareLaunchArgument(
            'competition_mode',
            default_value='False',
            description='Disable VRX debug bridges like a real competition',
        ),
        DeclareLaunchArgument(
            'preserve_waypoint_order',
            default_value='False',
            description=(
                'Follow Wayfinding goals in their published order instead of '
                'nearest-neighbor order'),
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
            description='RViz configuration file',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
