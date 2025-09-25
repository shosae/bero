import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    urdf_file_path = os.path.join(
        get_package_share_directory('bero_description'),
        'urdf',
        'bero.urdf'
    )

    with open(urdf_file_path, 'r') as urdf_file:
        urdf_desc = urdf_file.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_desc}]
    )

    return LaunchDescription([
        robot_state_publisher_node,
    ])
