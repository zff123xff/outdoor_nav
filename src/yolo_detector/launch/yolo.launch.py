from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('yolo_detector'), 'config', 'yolo.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        DeclareLaunchArgument('config_file', default_value=default_config),
        Node(
            package='yolo_detector', executable='yolo_detector',
            name='yolo_detector', namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('config_file')], output='screen'),
    ])
