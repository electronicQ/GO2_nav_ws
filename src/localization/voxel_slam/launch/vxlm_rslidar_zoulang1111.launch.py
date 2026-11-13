"""Offline mapping of zoulang1111 using native RoboSense PointCloud2."""

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('voxel_slam')
    config = LaunchConfiguration('config')
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    workspace = Path(os.environ.get(
        'GO2_NAV_WORKSPACE', '/home/gzq/GO2_nav_ws'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution(
            [share, 'config', 'ros2_rslidar_slam.yaml'])),
        DeclareLaunchArgument(
            'map_export_path',
            default_value=str(workspace / 'maps/zoulang1111_sc_map_rerun')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        LogInfo(msg='RoboSense input is native (lidar_type=4). Check IMU units '
                    'and LiDAR-to-IMU extrinsics in the YAML before evaluating accuracy.'),
        Node(package='voxel_slam', executable='voxelslam', name='voxelslam',
             output='screen', parameters=[config, {
                 'use_sim_time': sim_time,
                 'General.map_export_path':
                     LaunchConfiguration('map_export_path'),
             }]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             parameters=[{'use_sim_time': sim_time}],
             arguments=['-d', PathJoinSubstitution([share, 'rviz_cfg', 'ros2.rviz'])],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
