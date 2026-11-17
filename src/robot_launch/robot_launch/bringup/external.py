"""Navigation against external odometry and pointcloud inputs; no hardware drivers."""
from robot_launch.bringup.source import ComponentSource
from robot_launch.bringup import nav2


import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def _odom_relay(context):
    namespace = LaunchConfiguration("robot_namespace").perform(context).strip('/')
    with open(LaunchConfiguration("params_file").perform(context), encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    bridge = config.get('/' + namespace + '/nav2_input_bridge',
                        config.get('nav2_input_bridge', {})).get('ros__parameters', {})
    names = ('reference_latitude', 'reference_longitude', 'reference_altitude')
    if any(name not in bridge for name in names):
        raise RuntimeError('Simulation GPS requires nav2_input_bridge reference_latitude/longitude/altitude in params_file')
    return [Node(
        package="mavros_odom_relay", executable="odom_relay",
        name="mavros_odom_relay", namespace=namespace, output="screen",
        parameters=[{
            "input_topic": "mavros/odom", "output_topic": "odom",
            "frame_id_rewrite": "", "publish_gps": True,
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            **{name: float(bridge[name]) for name in names},
        }],
    )]


def generate_launch_description():
    share = get_package_share_directory("robot_launch")
    ns = LaunchConfiguration("robot_namespace")
    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value="robot_1"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_foxglove", default_value="false"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(share, "config", "go2w_nav2.yaml"),
            description="Navigation YAML; GPS reference must match the simulator.",
        ),
        SetParameter(name="use_sim_time", value=sim_time),
        IncludeLaunchDescription(
            ComponentSource(nav2.generate_launch_description),
            launch_arguments={
                "robot_namespace": ns,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "params_file": LaunchConfiguration("params_file"),
                "use_foxglove": LaunchConfiguration("use_foxglove"),
                "use_mid360": "false",
            }.items(),
        ),
        OpaqueFunction(function=_odom_relay),
        Node(
            package="twist_mux", executable="twist_mux", name="twist_mux",
            namespace=ns, output="screen",
            parameters=[{
                "use_sim_time": sim_time,
                "topics": {
                    "nav": {"topic": "cmd_vel_nav", "timeout": 0.5, "priority": 10},
                    "remote": {"topic": "cmd_vel_remote", "timeout": 0.5, "priority": 20},
                },
            }],
            remappings=[("cmd_vel_out", "cmd_vel")],
        ),
    ])
