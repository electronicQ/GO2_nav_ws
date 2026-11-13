"""Run the proprietary map processor with workspace-owned configuration."""

from pathlib import Path
import os
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import ExecutableInPackage


def _workspace():
    return Path(os.environ.get(
        'GO2_NAV_WORKSPACE', str(Path.home() / 'GO2_nav_ws')))


def _process(context):
    workspace = _workspace()
    config_path = LaunchConfiguration('config').perform(context)
    input_map = LaunchConfiguration('input_map').perform(context).strip()
    output_dir = LaunchConfiguration('output_dir').perform(context).strip()
    calibration_file = LaunchConfiguration(
        'calibration_file').perform(context).strip()

    config_file = Path(config_path)
    config = yaml.safe_load(config_file.read_text(encoding='utf-8'))
    if input_map:
        config['input_map'] = input_map
    elif not str(config.get('input_map', '')).strip():
        config['input_map'] = str(
            workspace / 'maps' / 'zoulang1111_sc_map')
    if output_dir:
        config['output_dir'] = output_dir
    if calibration_file:
        calibration = yaml.safe_load(
            Path(calibration_file).read_text(encoding='utf-8'))
        translation = calibration.get('imu_to_lidar', {}).get('translation')
        if not isinstance(translation, list) or len(translation) != 3:
            raise RuntimeError(
                f'Invalid imu_to_lidar.translation in {calibration_file}')
        config.setdefault('dynamic_removal', {})[
            'sensor_origin_imu_m'] = [
                float(value) for value in translation]
    generated = tempfile.NamedTemporaryFile(
        mode='w', encoding='utf-8', dir=config_file.parent,
        prefix='go2_process_map_', suffix='.yaml', delete=False)
    with generated:
        yaml.safe_dump(config, generated, sort_keys=False)
    config_path = generated.name

    command = [
        ExecutableInPackage(
            package='voxel_map_tools', executable='process_map'),
        '--config', config_path,
    ]
    if IfCondition(LaunchConfiguration('overwrite')).evaluate(context):
        command.append('--overwrite')
    return [ExecuteProcess(cmd=command, output='screen')]


def generate_launch_description():
    workspace = _workspace()
    default_config = str(
        workspace / 'src' / 'localization' / 'voxel_map_tools_runtime'
        / 'config' / 'default.yaml')
    default_calibration = str(
        workspace / 'config' / 'calibration' / 'zoulang1111_legacy.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='Workspace-owned map processing YAML file.'),
        DeclareLaunchArgument(
            'input_map', default_value='',
            description='Optional v4 map package; overrides config input_map.'),
        DeclareLaunchArgument(
            'output_dir', default_value='',
            description='Optional output directory; defaults to input_map/processed.'),
        DeclareLaunchArgument(
            'calibration_file', default_value=default_calibration,
            description='Calibration profile used to set the LiDAR ray origin.'),
        DeclareLaunchArgument(
            'overwrite', default_value='false',
            description='Replace existing outputs owned by this tool.'),
        OpaqueFunction(function=_process),
    ])
