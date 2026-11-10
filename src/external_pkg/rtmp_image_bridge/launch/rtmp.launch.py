import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _create_bridge(context):
    overrides = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    # Empty arguments preserve YAML configuration, including its stream URL.
    for key in ('rtmp_url', 'mode', 'image_topic', 'decoder'):
        value = LaunchConfiguration(key, default='').perform(context)
        if value:
            overrides[key] = ParameterValue(value, value_type=str)
    publish_fps = LaunchConfiguration('publish_fps', default='').perform(context)
    if publish_fps:
        overrides['publish_fps'] = ParameterValue(float(publish_fps), value_type=float)
    return [Node(package='rtmp_image_bridge', executable='rtmp_image_bridge',
                 namespace=LaunchConfiguration('namespace'), output='screen',
                 arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                 parameters=[LaunchConfiguration('config_file'), overrides])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        DeclareLaunchArgument('rtmp_url', default_value='',
                              description='Override YAML stream URL; required in either YAML or argument'),
        DeclareLaunchArgument('mode', default_value='', description='pull or listen; empty uses YAML'),
        DeclareLaunchArgument('image_topic', default_value='', description='Empty uses YAML'),
        DeclareLaunchArgument('decoder', default_value='',
                              description='auto|ffmpeg|ffmpeg-hw|gstreamer; empty uses YAML'),
        DeclareLaunchArgument('publish_fps', default_value='',
                              description='Publish rate limit; empty uses YAML'),
        DeclareLaunchArgument('log_level', default_value='ERROR',
                              description='RTMP bridge ROS log severity; WARN hides routine statistics'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config_file', default_value=os.path.join(
            get_package_share_directory('rtmp_image_bridge'), 'config', 'rtmp.yaml')),
        OpaqueFunction(function=_create_bridge),
    ])
