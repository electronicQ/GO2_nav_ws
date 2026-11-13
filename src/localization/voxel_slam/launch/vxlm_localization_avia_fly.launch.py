from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def localization_node(context, config, common, profile, use_sim_time, map_path, map_cloud_path):
    overrides = {'use_sim_time': use_sim_time, 'Localization.map_path': map_path,
                 'General.mode': 'localization', 'General.map_export_path': ''}
    display_path = map_cloud_path.perform(context)
    if display_path:
        overrides['Localization.visualization.map_cloud_path'] = display_path
    return [Node(package='voxel_slam', executable='voxelslam', name='voxelslam',
                 output='screen', parameters=[config, common, profile, overrides])]


def generate_launch_description():

    # 变量定位阶段
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_path = LaunchConfiguration('map_path')
    map_cloud_path = LaunchConfiguration('map_cloud_path')
    profile = LaunchConfiguration('relocalization_config')
    common = PathJoinSubstitution([
        FindPackageShare('voxel_slam'), 'config', 'ros2_localization_common.yaml'])
    config = PathJoinSubstitution([
        FindPackageShare('voxel_slam'), 'config', 'ros2_localization_avia_fly.yaml'])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('voxel_slam'), 'rviz_cfg', 'ros2_localization_avia.rviz'])
    
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map_path', default_value=''),
        DeclareLaunchArgument('map_cloud_path', default_value='',
                              description='Optional full display PCD path; otherwise use the YAML setting.'),
        DeclareLaunchArgument('relocalization_config', default_value=PathJoinSubstitution([
            FindPackageShare('voxel_slam'), 'config', 'ros2_scancontext_relocalization.yaml'])),
        OpaqueFunction(function=localization_node,
                       args=[config, common, profile, use_sim_time, map_path, map_cloud_path]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             parameters=[{'use_sim_time': use_sim_time}],
             arguments=['-d', rviz_config], condition=IfCondition(rviz)),
    ])
