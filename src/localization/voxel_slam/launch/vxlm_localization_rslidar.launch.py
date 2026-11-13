"""Native RoboSense localization against the zoulang1111 v4 scan map."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os
from pathlib import Path


def generate_launch_description():
    share = FindPackageShare('voxel_slam')
    map_path = LaunchConfiguration('map_path')
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)
    common = PathJoinSubstitution([
        share, 'config', 'ros2_localization_common.yaml'])
    rviz_config = PathJoinSubstitution([
        share, 'rviz_cfg', 'ros2_localization_rslidar.rviz'])
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'map_path',
            default_value=str(
                Path(os.environ.get(
                    'GO2_NAV_WORKSPACE', '/home/gzq/GO2_nav_ws'))
                / 'maps/zoulang1111_sc_map'),
            description='v4 map directory containing manifest.yaml and scans.yaml.'),
        DeclareLaunchArgument(
            'map_cloud_path',
            default_value=PathJoinSubstitution([
                map_path, 'processed', 'map_full.pcd']),
            description='Display-only PCD, already expressed in map coordinates.'),
        DeclareLaunchArgument(
            'config', default_value=PathJoinSubstitution([
                share, 'config', 'ros2_localization_rslidar.yaml']),
            description='RoboSense input, extrinsics and LIO configuration.'),
        DeclareLaunchArgument(
            'relocalization_config', default_value=PathJoinSubstitution([
                share, 'config', 'ros2_scancontext_rslidar_relocalization.yaml']),
            description='RoboSense Scan Context / Quatro / Nano-GICP configuration.'),
        Node(
            package='voxel_slam', executable='voxelslam', name='voxelslam',
            output='screen', parameters=[
                LaunchConfiguration('config'), common,
                LaunchConfiguration('relocalization_config'), {
                    'use_sim_time': use_sim_time,
                    'General.mode': 'localization',
                    'General.map_export_path': '',
                    'Localization.map_path': ParameterValue(map_path, value_type=str),
                    'Localization.visualization.map_cloud_path': ParameterValue(
                        LaunchConfiguration('map_cloud_path'), value_type=str),
                }]),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
