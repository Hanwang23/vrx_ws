from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    package_share = get_package_share_directory('han_usv_controller')
    default_config = os.path.join(package_share, 'config', 'controller.yaml')
    config = LaunchConfiguration('config')
    robot_localization = LaunchConfiguration('robot_localization')
    robot_localization_config = LaunchConfiguration(
        'robot_localization_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Controller parameter YAML file',
        ),
        DeclareLaunchArgument(
            'robot_localization',
            default_value='True',
            description='Start the external robot_localization EKF and adapter',
        ),
        DeclareLaunchArgument(
            'robot_localization_config',
            default_value=os.path.join(
                package_share, 'config', 'robot_localization.yaml'),
            description='robot_localization EKF parameter YAML file',
        ),
        Node(
            package='han_usv_controller',
            executable='gnss_odometry_adapter',
            name='gnss_odometry_adapter',
            output='screen',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(robot_localization),
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='han_usv_ekf_filter',
            output='screen',
            parameters=[robot_localization_config],
            remappings=[('odometry/filtered', '/odometry/filtered')],
            condition=IfCondition(robot_localization),
        ),
        Node(
            package='han_usv_controller',
            executable='autonomous_usv',
            name='autonomous_usv',
            output='screen',
            parameters=[config],
            emulate_tty=True,
        ),
    ])
