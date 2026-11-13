"""Live or bag mapping with external IMU, RS16, and Voxel-SLAM."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from go2_nav_bringup.launch_utils import (
    DEFAULT_BAG,
    delayed_finish_action,
    load_calibration,
    mapping_legacy_directory,
    parse_auto_bool,
    sensor_nodes,
    validate_auto_finish_mode,
    voxel_extrinsic_parameters,
    workspace_root,
)


def _setup(context):
    args = {
        name: LaunchConfiguration(name).perform(context)
        for name in (
            'sensor_source', 'bag_path', 'map_name', 'calibration_file',
            'voxel_config', 'lidar_config', 'auto_finish',
            'auto_finish_mode', 'bag_start_paused', 'bag_rate',
            'sensor_serial_port', 'sensor_serial_baud', 'finish_delay_sec',
            'rviz', 'use_sim_time', 'legacy_save_root',
        )
    }
    sensor_source = args['sensor_source']
    if sensor_source not in ('live', 'bag'):
        raise RuntimeError('sensor_source must be live or bag')
    automatic_sim = sensor_source == 'bag'
    use_sim_time = parse_auto_bool(
        args['use_sim_time'], automatic=automatic_sim, name='use_sim_time')
    auto_finish = parse_auto_bool(
        args['auto_finish'], automatic=automatic_sim, name='auto_finish')
    auto_finish_mode = validate_auto_finish_mode(args['auto_finish_mode'])
    bag_start_paused = parse_auto_bool(
        args['bag_start_paused'], automatic=False, name='bag_start_paused')
    show_rviz = parse_auto_bool(
        args['rviz'], automatic=True, name='rviz')

    ws = workspace_root()
    map_name = args['map_name'].strip()
    if not map_name or '/' in map_name:
        raise RuntimeError('map_name must be a single directory name')
    map_export_path = ws / 'maps' / map_name
    if map_export_path.exists():
        raise RuntimeError(
            f'Map export target already exists: {map_export_path}. '
            'Choose a new map_name; Voxel-SLAM does not overwrite maps.')

    legacy_directory = mapping_legacy_directory(
        ws, map_name, args['legacy_save_root'])

    calibration_arg = args['calibration_file'].strip()
    if calibration_arg in ('', 'auto'):
        calibration_path = ws / 'config/calibration/current_robot.yaml'
    else:
        calibration_path = Path(calibration_arg)
    calibration = load_calibration(calibration_path)

    sensors, bag_action = sensor_nodes(
        sensor_source=sensor_source,
        bag_path=Path(args['bag_path']),
        use_sim_time=use_sim_time,
        serial_port=args['sensor_serial_port'],
        serial_baud=int(args['sensor_serial_baud']),
        lidar_config=args['lidar_config'],
        bag_start_paused=bag_start_paused,
        bag_rate=float(args['bag_rate']),
    )

    parameters = [
        args['voxel_config'],
        {
            'use_sim_time': use_sim_time,
            'General.mode': 'mapping',
            'General.map_export_path': str(map_export_path),
            'General.is_save_map': 0,
            'General.save_path': f'{legacy_directory}{os.sep}',
            'General.bagname': map_name,
            **voxel_extrinsic_parameters(calibration),
        },
    ]
    voxel = Node(
        package='voxel_slam',
        executable='voxelslam',
        name='voxelslam',
        output='screen',
        parameters=parameters,
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(
            Path(get_package_share_directory('go2_nav_bringup'))
            / 'config/rviz/mapping.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(args['rviz']),
    )

    actions = [
        *(
            [
                UnsetEnvironmentVariable('CYCLONEDDS_URI'),
                SetEnvironmentVariable(
                    'RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
                SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
            ]
            if sensor_source == 'bag' else []
        ),
        *sensors,
        voxel,
        rviz,
    ]
    if bag_action is not None:
        actions.append(bag_action)
        if auto_finish:
            actions.append(delayed_finish_action(
                bag_action,
                delay_sec=float(args['finish_delay_sec']),
                finish_parameter=auto_finish_mode,
            ))
    return actions


def generate_launch_description():
    ws = workspace_root()
    default_bag = str(DEFAULT_BAG)
    default_voxel_config = (
        Path(get_package_share_directory('voxel_slam'))
        / 'config/ros2_rslidar_slam.yaml')
    default_lidar_config = (
        Path(get_package_share_directory('rslidar_sdk'))
        / 'config/config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('sensor_source', default_value='live'),
        DeclareLaunchArgument('bag_path', default_value=default_bag),
        DeclareLaunchArgument('map_name', default_value='new_sc_map'),
        DeclareLaunchArgument(
            'calibration_file', default_value='auto'),
        DeclareLaunchArgument(
            'voxel_config', default_value=str(default_voxel_config)),
        DeclareLaunchArgument(
            'lidar_config', default_value=str(default_lidar_config)),
        DeclareLaunchArgument('auto_finish', default_value='auto'),
        DeclareLaunchArgument(
            'auto_finish_mode', default_value='finish',
            description='Automatic bag finish: finish or finish_no_ba.'),
        DeclareLaunchArgument('bag_start_paused', default_value='false'),
        DeclareLaunchArgument('bag_rate', default_value='1.0'),
        DeclareLaunchArgument(
            'sensor_serial_port', default_value='/dev/imu_usb'),
        DeclareLaunchArgument('sensor_serial_baud', default_value='115200'),
        DeclareLaunchArgument('finish_delay_sec', default_value='5.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='auto'),
        DeclareLaunchArgument(
            'legacy_save_root',
            default_value=str(ws / '.mapping_legacy'),
            description='Workspace scratch root for Voxel-SLAM legacy output.'),
        OpaqueFunction(function=_setup),
    ])
