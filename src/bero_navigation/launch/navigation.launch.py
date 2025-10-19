import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_bero_nav = get_package_share_directory('bero_navigation')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    map_file = os.path.join(pkg_bero_nav, 'maps', 'home_0.025_2.yaml')
    nav_config = os.path.join(pkg_bero_nav, 'config', 'nav2_params.yaml')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=map_file
    )
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav_config
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_nav2,
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'autostart': 'true',  # activate all nav2 nodes, 없으면 하나하나 activate 필요
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
        }.items()
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        nav2_bringup_launch,
    ])
