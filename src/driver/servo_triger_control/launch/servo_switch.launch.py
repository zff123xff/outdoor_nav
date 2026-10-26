from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(
                get_package_share_directory('servo_triger_control'), 'config', 'servo.yaml')),
        Node(
            package='servo_triger_control', executable='servo_triger_node',
            name='servo_triger_node', output='screen',
            parameters=[LaunchConfiguration('params_file')]),
    ])
