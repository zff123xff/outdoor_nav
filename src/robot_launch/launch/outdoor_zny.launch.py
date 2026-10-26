"""Beijing ZNY three-topic navigation contract, independent of simulation code."""
import os
import re
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, GroupAction,
                            OpaqueFunction, RegisterEventHandler, EmitEvent)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def bringup(context):
    share = get_package_share_directory("robot_launch")
    with open(LaunchConfiguration("params_file").perform(context)) as stream:
        config = yaml.safe_load(stream) or {}
    ns = (LaunchConfiguration("robot_namespace").perform(context) or
          config.get("launch_config", {}).get("ros__parameters", {}).get("robot_namespace", "robot_1"))
    if not isinstance(ns, str):
        raise RuntimeError("robot_namespace must be a string")
    ns = ns.strip("/")
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", ns):
        raise RuntimeError("robot_namespace must be one non-empty ROS name")
    managed = ["controller_server", "straight_planner"]
    watchdog = Node(package="robot_launch", executable="lifecycle_startup_watchdog",
                    namespace=ns, output="screen")
    relay = Node(package="robot_launch", executable="lifecycle_service_relay", namespace=ns,
                 parameters=[{"node_names": managed}], output="screen")
    def failed(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="ZNY navigation startup failed"))]
        return []
    return [
        RegisterEventHandler(OnProcessExit(target_action=watchdog, on_exit=failed)),
        RegisterEventHandler(OnProcessExit(target_action=relay, on_exit=failed)),
        GroupAction(actions=[
            *[SetRemap(src="lifecycle_manager_navigation:/"+ns+"/"+name+"/"+service,
                       dst="/"+ns+"/lifecycle_relay/"+name+"/"+service)
              for name in managed for service in ("change_state", "get_state")],
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, "launch", "outdoor_hil.launch.py")),
                launch_arguments={
                    "robot_namespace": ns,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "params_file": LaunchConfiguration("params_file"),
                    "use_foxglove": LaunchConfiguration("use_foxglove"),
                }.items(),
            ),
        ]),
        relay,
        watchdog,
    ]


def generate_launch_description():
    share = get_package_share_directory("robot_launch")
    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value="", description="Override namespace in params_file launch_config; fallback robot_1"),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Default: wall clock; true requires simulator /clock."),
        DeclareLaunchArgument("use_foxglove", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "zny_hil.yaml")),
        OpaqueFunction(function=bringup),
    ])
