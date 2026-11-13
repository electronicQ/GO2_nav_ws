from pathlib import Path

import pytest
import yaml

from go2_nav_bringup.launch_utils import (
    auto_finish_command,
    load_calibration,
    mapping_legacy_directory,
    quaternion_to_matrix,
    resolve_map_yaml,
    validate_auto_finish_mode,
    workspace_root,
)


def test_identity_quaternion_to_matrix():
    assert quaternion_to_matrix([0.0, 0.0, 0.0, 1.0]) == [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def test_auto_finish_mode_validation():
    assert validate_auto_finish_mode('finish') == 'finish'
    assert validate_auto_finish_mode('finish_no_ba') == 'finish_no_ba'
    with pytest.raises(RuntimeError):
        validate_auto_finish_mode('invalid')
    assert auto_finish_command('finish') == [
        'ros2', 'param', 'set', '/voxelslam', 'finish', 'true']
    assert auto_finish_command('finish_no_ba') == [
        'ros2', 'param', 'set', '/voxelslam', 'finish_no_ba', 'true']


def test_mapping_legacy_directory_is_workspace_local(tmp_path):
    directory = mapping_legacy_directory(tmp_path, 'test_scene')
    assert directory.is_dir()
    assert directory.parent == tmp_path / '.mapping_legacy'
    assert directory.name.startswith('test_scene_legacy_')

    override = mapping_legacy_directory(
        tmp_path, 'override_scene', 'legacy_override')
    assert override.is_dir()
    assert override.parent == tmp_path / 'legacy_override'


def test_legacy_calibration_is_consistent():
    calibration = load_calibration(
        workspace_root() / 'config/calibration/zoulang1111_legacy.yaml')
    assert calibration['imu_frame'] == 'imu_link'
    assert calibration['base_frame'] == 'base_link'
    assert calibration['lidar_frame'] == 'rslidar'
    assert len(calibration['imu_to_lidar']['translation']) == 3
    assert len(calibration['imu_to_lidar']['rotation_xyzw']) == 4
    assert calibration['robot_motion_authorized'] is False


def test_copied_v4_map_resolves():
    map_dir, map_yaml = resolve_map_yaml(
        workspace_root() / 'maps/zoulang1111_sc_map')
    assert (map_dir / 'manifest.yaml').is_file()
    assert map_yaml.name == 'map.yaml'
    assert map_yaml.is_file()


def test_nav2_uses_voxel_and_rslidar_without_amcl():
    from ament_index_python.packages import get_package_share_directory

    path = (
        Path(get_package_share_directory('go2_navigation2'))
        / 'config/nav2_params.yaml')
    params = yaml.safe_load(path.read_text(encoding='utf-8'))
    assert 'amcl' not in params
    assert params['bt_navigator']['ros__parameters']['odom_topic'] == \
        '/LIO/odom_imu'
    local = params['local_costmap']['local_costmap']['ros__parameters']
    assert local['voxel_layer']['rslidar']['topic'] == '/rslidar_points'
    assert local['voxel_layer']['rslidar']['data_type'] == 'PointCloud2'
    assert local['voxel_layer']['z_voxels'] <= 16
    assert local['robot_base_frame'] == 'base_link'
