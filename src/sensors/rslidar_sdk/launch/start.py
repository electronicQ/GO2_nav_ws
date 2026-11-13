from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('rslidar_sdk'),
        'config',
        'config.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=default_config,
            description='Absolute path to the RoboSense driver YAML.'),
        Node(
            package='rslidar_sdk',
            executable='rslidar_sdk_node',
            name='rslidar_sdk_node',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('config_path'),
            }]),
    ])
