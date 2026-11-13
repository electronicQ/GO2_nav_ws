"""Run Voxel-SLAM localization and Nav2 with one RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from go2_nav_bringup.launch_utils import (
    DEFAULT_BAG,
    load_calibration,
    parse_auto_bool,
    resolve_map_yaml,
    sensor_nodes,
    static_tf_node,
    voxel_extrinsic_parameters,
    workspace_root,
)


def _setup(context):
    names = (
        'sensor_source', 'bag_path', 'map_path', 'calibration_file',
        'voxel_config', 'relocalization_config', 'params_file',
        'lidar_config', 'control_robot', 'bag_start_paused', 'bag_rate',
        'sensor_serial_port', 'sensor_serial_baud', 'rviz', 'use_sim_time',
    )
    args = {
        name: LaunchConfiguration(name).perform(context)
        for name in names
    }
    sensor_source = args['sensor_source']
    if sensor_source not in ('live', 'bag'):
        raise RuntimeError('sensor_source must be live or bag')
    use_sim_time = parse_auto_bool(
        args['use_sim_time'],
        automatic=sensor_source == 'bag',
        name='use_sim_time',
    )
    control_robot = parse_auto_bool(
        args['control_robot'], automatic=False, name='control_robot')
    bag_start_paused = parse_auto_bool(
        args['bag_start_paused'], automatic=False, name='bag_start_paused')
    show_rviz = parse_auto_bool(
        args['rviz'], automatic=True, name='rviz')

    calibration_path = Path(args['calibration_file'])
    calibration = load_calibration(calibration_path)
    if control_robot and calibration.get('robot_motion_authorized') is not True:
        raise RuntimeError(
            'control_robot=true requires robot_motion_authorized: true in '
            f'{calibration_path}. The current calibration is not approved '
            'for real-robot motion.')

    map_dir, map_yaml = resolve_map_yaml(Path(args['map_path']))
    map_cloud = map_dir / 'processed/map_full.pcd'
    if not map_cloud.is_file():
        raise RuntimeError(f'Map display cloud is missing: {map_cloud}')

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

    imu_frame = calibration['imu_frame']
    base_frame = calibration['base_frame']
    lidar_frame = calibration['lidar_frame']
    tf_nodes = [
        static_tf_node(
            'imu_to_base_tf', imu_frame, base_frame,
            calibration['imu_to_base'], use_sim_time),
        static_tf_node(
            'imu_to_lidar_tf', imu_frame, lidar_frame,
            calibration['imu_to_lidar'], use_sim_time),
    ]

    voxel = Node(
        package='voxel_slam',
        executable='voxelslam',
        name='voxelslam',
        output='screen',
        parameters=[
            args['voxel_config'],
            args['relocalization_config'],
            {
                'use_sim_time': use_sim_time,
                'General.mode': 'localization',
                'General.map_export_path': '',
                'General.is_save_map': 0,
                'General.lid_topic': '/rslidar_points',
                'General.imu_topic': '/imu/data_raw',
                'Localization.map_path': str(map_dir),
                'Localization.visualization.publish_map_cloud': True,
                'Localization.visualization.map_cloud_path': str(map_cloud),
                **voxel_extrinsic_parameters(calibration),
            },
        ],
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': str(map_yaml),
            'use_sim_time': use_sim_time,
            'topic_name': 'map',
        }],
    )
    localization_manager = Node(
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
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('nav2_bringup'))
            / 'launch/navigation_launch.py')),
        launch_arguments={
            'params_file': args['params_file'],
            'use_sim_time': str(use_sim_time).lower(),
        }.items(),
    )

    twist_bridge = Node(
        package='go2_twist_bridge',
        executable='twist_bridge',
        name='twist_bridge_node_cpp',
        output='screen',
        parameters=[{
            'require_localization_valid': True,
            'localization_valid_topic': '/localization/valid',
            'localization_state_topic': '/localization/state',
            'required_localization_states':
                'IN_MAP_TRACKING,GLOBAL_TRACKING',
            'localization_timeout_sec': 1.0,
            'cmd_vel_timeout_sec': 0.5,
            'stop_on_gate_close': True,
        }],
        condition=IfCondition(args['control_robot']),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(
            Path(get_package_share_directory('go2_nav_bringup'))
            / 'config/rviz/localization_nav.rviz')],
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
        *tf_nodes,
        voxel,
        map_server,
        localization_manager,
        nav2,
        twist_bridge,
        rviz,
    ]
    if bag_action is not None:
        actions.append(bag_action)
    return actions


def generate_launch_description():
    ws = workspace_root()
    voxel_share = Path(get_package_share_directory('voxel_slam'))
    nav_share = Path(get_package_share_directory('go2_navigation2'))
    lidar_share = Path(get_package_share_directory('rslidar_sdk'))
    default_map = ws / 'maps/zoulang1111_sc_map'
    default_calibration = (
        ws / 'config/calibration/zoulang1111_legacy.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('sensor_source', default_value='live'),
        DeclareLaunchArgument('bag_path', default_value=str(DEFAULT_BAG)),
        DeclareLaunchArgument('map_path', default_value=str(default_map)),
        DeclareLaunchArgument(
            'calibration_file', default_value=str(default_calibration)),
        DeclareLaunchArgument(
            'voxel_config',
            default_value=str(
                voxel_share / 'config/ros2_localization_rslidar.yaml')),
        DeclareLaunchArgument(
            'relocalization_config',
            default_value=str(
                voxel_share
                / 'config/ros2_scancontext_rslidar_relocalization.yaml')),
        DeclareLaunchArgument(
            'params_file',
            default_value=str(
                nav_share / 'config/nav2_params.yaml')),
        DeclareLaunchArgument(
            'lidar_config',
            default_value=str(lidar_share / 'config/config.yaml')),
        DeclareLaunchArgument('control_robot', default_value='false'),
        DeclareLaunchArgument('bag_start_paused', default_value='false'),
        DeclareLaunchArgument('bag_rate', default_value='1.0'),
        DeclareLaunchArgument(
            'sensor_serial_port', default_value='/dev/imu_usb'),
        DeclareLaunchArgument('sensor_serial_baud', default_value='115200'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='auto'),
        OpaqueFunction(function=_setup),
    ])
