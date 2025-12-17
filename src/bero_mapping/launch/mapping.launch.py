import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_bero_mapping = get_package_share_directory('bero_mapping')
    pkg_bero_bringup = get_package_share_directory('bero_bringup')

    mapping_config = os.path.join(
        pkg_bero_mapping,
        'config',
        'mapper_params_online_async.yaml',
    )

    bero_bringup_launch_file = os.path.join(
        pkg_bero_bringup,
        'launch',
        'bero_bringup.launch.py',
    )

    bero_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bero_bringup_launch_file])
    )

    async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[mapping_config],
    )

    return LaunchDescription([
        bero_bringup_launch,
        async_slam_toolbox_node,
    ])
