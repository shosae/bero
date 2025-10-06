import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ekf_config = os.path.join(
        get_package_share_directory('bero_localization'),
        'config',
        'ekf_odom.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_odom',
        output='screen',
        parameters=[ekf_config]
    )

    return LaunchDescription([
        ekf_node,
    ])
