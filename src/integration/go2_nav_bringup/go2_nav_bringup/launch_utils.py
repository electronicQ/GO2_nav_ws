"""Shared validation and launch helpers for GO2 Voxel-SLAM navigation."""

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Any

import yaml
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


DEFAULT_WORKSPACE = Path('/home/gzq/GO2_nav_ws')
DEFAULT_BAG = Path(
    '/home/gzq/voxel_slam_ws/bag/ros2bag/zoulang1111')


def workspace_root() -> Path:
    return Path(os.environ.get('GO2_NAV_WORKSPACE', DEFAULT_WORKSPACE))


def parse_auto_bool(value: str, *, automatic: bool, name: str) -> bool:
    text = value.strip().lower()
    if text == 'auto':
        return automatic
    if text in ('true', '1', 'yes', 'on'):
        return True
    if text in ('false', '0', 'no', 'off'):
        return False
    raise RuntimeError(f'{name} must be auto, true, or false (got {value!r})')


def validate_auto_finish_mode(value: str) -> str:
    mode = value.strip()
    if mode not in ('finish', 'finish_no_ba'):
        raise RuntimeError(
            'auto_finish_mode must be finish or finish_no_ba '
            f'(got {value!r})')
    return mode


def auto_finish_command(finish_parameter: str) -> list[str]:
    parameter = validate_auto_finish_mode(finish_parameter)
    return ['ros2', 'param', 'set', '/voxelslam', parameter, 'true']


def load_calibration(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise RuntimeError(f'Calibration file does not exist: {path}')
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise RuntimeError(f'Calibration YAML must contain a mapping: {path}')

    required_frames = ('imu_frame', 'base_frame', 'lidar_frame')
    for key in required_frames:
        if not isinstance(data.get(key), str) or not data[key]:
            raise RuntimeError(f'Calibration is missing a valid {key}: {path}')

    for key in ('imu_to_base', 'imu_to_lidar'):
        transform = data.get(key)
        if not isinstance(transform, dict):
            raise RuntimeError(f'Calibration is missing {key}: {path}')
        translation = transform.get('translation')
        quaternion = transform.get('rotation_xyzw')
        if not isinstance(translation, list) or len(translation) != 3:
            raise RuntimeError(f'{key}.translation must have three values')
        if not isinstance(quaternion, list) or len(quaternion) != 4:
            raise RuntimeError(f'{key}.rotation_xyzw must have four values')
        values = [float(v) for v in translation + quaternion]
        if not all(math.isfinite(v) for v in values):
            raise RuntimeError(f'{key} contains a non-finite value')
        norm = math.sqrt(sum(v * v for v in quaternion))
        if norm < 1e-12:
            raise RuntimeError(f'{key}.rotation_xyzw has zero norm')

    data['imu_to_base'] = {
        'translation': [float(v) for v in data['imu_to_base']['translation']],
        'rotation_xyzw': [
            float(v) for v in data['imu_to_base']['rotation_xyzw']],
    }
    data['imu_to_lidar'] = {
        'translation': [float(v) for v in data['imu_to_lidar']['translation']],
        'rotation_xyzw': [
            float(v) for v in data['imu_to_lidar']['rotation_xyzw']],
    }
    return data


def quaternion_to_matrix(quaternion: list[float]) -> list[list[float]]:
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return [
        [1.0 - 2.0 * (y * y + z * z),
         2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w),
         1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w),
         2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ]


def static_tf_node(
    name: str,
    parent: str,
    child: str,
    transform: dict[str, Any],
    use_sim_time: bool,
) -> Node:
    x, y, z = transform['translation']
    qx, qy, qz, qw = transform['rotation_xyzw']
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        output='screen',
        arguments=[
            '--x', f'{x:.12g}', '--y', f'{y:.12g}', '--z', f'{z:.12g}',
            '--qx', f'{qx:.12g}', '--qy', f'{qy:.12g}',
            '--qz', f'{qz:.12g}', '--qw', f'{qw:.12g}',
            '--frame-id', parent, '--child-frame-id', child,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )


def voxel_extrinsic_parameters(calibration: dict[str, Any]) -> dict[str, Any]:
    transform = calibration['imu_to_lidar']
    matrix = quaternion_to_matrix(transform['rotation_xyzw'])
    return {
        'General.extrinsic_tran': transform['translation'],
        'General.extrinsic_rota': [value for row in matrix for value in row],
    }


def sensor_nodes(
    *,
    sensor_source: str,
    bag_path: Path,
    use_sim_time: bool,
    serial_port: str,
    serial_baud: int,
    lidar_config: str,
    bag_start_paused: bool,
    bag_rate: float,
) -> tuple[list[Any], Any | None]:
    if sensor_source == 'live':
        return [
            Node(
                package='rslidar_sdk',
                executable='rslidar_sdk_node',
                name='rslidar_sdk_node',
                output='screen',
                parameters=[{'config_path': lidar_config}],
            ),
            Node(
                package='wit_ros2_imu',
                executable='wit_ros2_imu',
                name='wit_imu_driver',
                output='screen',
                parameters=[{
                    'port': serial_port,
                    'baud': serial_baud,
                    'frame_id': 'imu_link',
                }],
            ),
        ], None

    if sensor_source != 'bag':
        raise RuntimeError(
            f'sensor_source must be live or bag (got {sensor_source!r})')
    if not bag_path.is_dir():
        raise RuntimeError(f'ROS 2 bag directory does not exist: {bag_path}')
    if (bag_path / 'metadata.yaml').is_file() is False:
        raise RuntimeError(f'ROS 2 bag has no metadata.yaml: {bag_path}')

    command = [
        'ros2', 'bag', 'play', str(bag_path), '--clock',
        '--topics', '/imu/data_raw', '/rslidar_points',
        '--rate', f'{bag_rate:.6g}',
    ]
    if bag_start_paused:
        command.append('--start-paused')
    bag_action = ExecuteProcess(cmd=command, output='screen')
    return [], bag_action


def delayed_finish_action(
    bag_action: Any,
    *,
    delay_sec: float,
    finish_parameter: str = 'finish',
) -> RegisterEventHandler:
    command = auto_finish_command(finish_parameter)
    return RegisterEventHandler(
        OnProcessExit(
            target_action=bag_action,
            on_exit=[
                TimerAction(
                    period=delay_sec,
                    actions=[
                        ExecuteProcess(
                            cmd=command,
                            output='screen',
                        ),
                    ],
                ),
            ],
        ),
    )


def resolve_map_yaml(map_path: Path) -> tuple[Path, Path]:
    """Return (v4 map directory, Nav2 map YAML)."""
    if map_path.is_dir():
        map_dir = map_path
        map_yaml = map_dir / 'processed' / 'map.yaml'
    elif map_path.suffix in ('.yaml', '.yml'):
        map_yaml = map_path
        map_dir = map_path.parent.parent
    else:
        raise RuntimeError(f'Invalid map path: {map_path}')
    if not (map_dir / 'manifest.yaml').is_file():
        raise RuntimeError(
            f'Map path is not a v4 Voxel-SLAM package: {map_dir}')
    if not map_yaml.is_file():
        raise RuntimeError(
            f'Processed Nav2 map YAML does not exist: {map_yaml}')
    return map_dir, map_yaml
