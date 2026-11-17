from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
import shlex
import sys


def _create_detector(context):
    namespace = LaunchConfiguration('namespace').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context).strip()
    if not log_level:
        log_level = 'ERROR'
    parameters = [LaunchConfiguration('config_file'), {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
    }]
    targets_file = LaunchConfiguration('targets_file').perform(context)
    if targets_file:
        parameters.append({'targets_file': targets_file})
    return [Node(
        package='yolo_detector', executable='yolo_detector', name='yolo_detector',
        namespace=namespace,
        prefix=LaunchConfiguration('python_executable'),
        arguments=['--ros-args', '--log-level', log_level],
        parameters=parameters, output='screen')]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('yolo_detector'), 'config', 'yolo.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        DeclareLaunchArgument('log_level', default_value='',
                              description='Override ROS log severity; default is WARN for robot_2 and INFO otherwise'),
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('python_executable', default_value=shlex.quote(sys.executable)),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('targets_file', default_value=''),
        OpaqueFunction(function=_create_detector),
    ])
