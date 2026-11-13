"""Nav2 and map-server-only entry point.

Localization and sensor/control bringup live in go2_nav_bringup. This file is
kept for users who want to start Nav2 independently against an already-running
Voxel-SLAM TF tree.
"""

from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('go2_navigation2'))
    nav2_share = Path(get_package_share_directory('nav2_bringup'))
    map_path = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_path,
            'use_sim_time': use_sim_time,
            'topic_name': 'map',
        }],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(nav2_share / 'launch/navigation_launch.py')),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=str(
                Path(os.environ.get(
                    'GO2_NAV_WORKSPACE', '/home/gzq/GO2_nav_ws'))
                / 'maps/zoulang1111_sc_map/processed/map.yaml')),
        DeclareLaunchArgument(
            'params_file',
            default_value=str(package_share / 'config/nav2_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        map_server,
        lifecycle_manager,
        navigation,
    ])
