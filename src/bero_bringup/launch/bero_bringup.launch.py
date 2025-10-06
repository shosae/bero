import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    imu_publisher_node = Node(
        package='imu',
        executable='imu_publisher',
        name='imu_publisher',
        output='screen'
    )

    joint_state_publisher_node = Node(
        package='encoder',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    odometry_publisher_node = Node(
        package='bero_localization',
        executable='odometry_publisher',
        name='odometry_publisher',
        output='screen'
    )

    ekf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bero_localization'),
                'launch',
                'ekf_odom.launch.py'
            )
        )
    )

    rplidar_s3_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_s3_launch.py'
            )
        )
    )

    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bero_description'),
                'launch',
                'bero_description.launch.py'
            )
        )
    )

    return LaunchDescription([
        imu_publisher_node,
        joint_state_publisher_node,
        odometry_publisher_node,
        ekf_node,
        rplidar_s3_node,
        robot_state_publisher_node,
    ])
