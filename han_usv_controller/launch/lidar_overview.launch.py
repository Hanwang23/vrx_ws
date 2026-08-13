from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory('han_usv_controller')
    config = os.path.join(share, 'config', 'lidar_overview.rviz')
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='usv_lidar_overview',
            arguments=['-d', config],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
