"""External sensor navigation and optional transport bridges."""
from robot_launch.bringup.source import ComponentSource
from robot_launch.bringup import external, pipeline, visualization

import os
import re
import sys
import yaml
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, GroupAction,
                            OpaqueFunction, RegisterEventHandler, EmitEvent, ExecuteProcess,
                            SetLaunchConfiguration)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def mavros_bridge(context):

    if LaunchConfiguration(
        "use_mavros_bridge"
    ).perform(context).lower() != "true":
        return []

    bridge_dir = os.path.expanduser(
        LaunchConfiguration(
            "mavros_bridge_dir"
        ).perform(context)
    )

    enable = LaunchConfiguration(
        "mavros_bridge_enable"
    ).perform(context)

    supervisor_path = os.path.join(
        bridge_dir,
        "mavros_bridge_supervisor.py"
    )

    if not os.path.isfile(supervisor_path):
        print(
            f"[WARN] MavrosBridge supervisor not found: "
            f"{supervisor_path}, skipping."
        )

        return []

    return [
        ExecuteProcess(
            cmd=[
                sys.executable,
                supervisor_path,
                "--bridge-dir",
                bridge_dir,
                "--enable",
                enable,
            ],

            name="mavros_bridge_supervisor",

            output="screen",

            # supervisor 自己如果意外挂掉，
            # launch 再把 supervisor 拉起来
            respawn=True,

            respawn_delay=2.0,
        )
    ]

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
    startup = config.get("lifecycle_startup_watchdog", {}).get("ros__parameters", {})
    startup_timeout = startup.get("startup_timeout", 180.0)
    watchdog = Node(package="robot_launch", executable="lifecycle_startup_watchdog",
                    namespace=ns, parameters=[startup], output="screen")
    relay = Node(package="robot_launch", executable="lifecycle_service_relay", namespace=ns,
                 parameters=[{"node_names": managed, "activation_timeout": startup_timeout}], output="screen")
    def failed(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="Navigation startup failed"))]
        return []
    return [
        # Persist the resolved fallback outside the scoped navigation group.
        # Pipeline and RViz are siblings and must see the very same namespace.
        SetLaunchConfiguration("robot_namespace", ns),
        RegisterEventHandler(OnProcessExit(target_action=watchdog, on_exit=failed)),
        RegisterEventHandler(OnProcessExit(target_action=relay, on_exit=failed)),
        GroupAction(actions=[
            *[SetRemap(src="lifecycle_manager_navigation:/"+ns+"/"+name+"/"+service,
                       dst="/"+ns+"/lifecycle_relay/"+name+"/"+service)
              for name in managed for service in ("change_state", "get_state")],
            IncludeLaunchDescription(
                ComponentSource(external.generate_launch_description),
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
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "go2w_nav2.yaml")),
        DeclareLaunchArgument("use_mavros_bridge", default_value="true",
                              description="Start the UE<->ROS UDP bridges from mavros_bridge_dir."),
        DeclareLaunchArgument("mavros_bridge_dir", default_value="~/MavrosBridge",
                              description="Directory holding ue_to_ros_bridge.py / ros_to_ue_bridge.py."),
        DeclareLaunchArgument("mavros_bridge_enable", default_value="dog",
                              description="Enable the dog transport in MavrosBridge."),
        DeclareLaunchArgument("mqtt", default_value="false"),
        DeclareLaunchArgument("address", default_value="tcp://127.0.0.1:1883"),
        DeclareLaunchArgument("mqtt_username", default_value=""),
        DeclareLaunchArgument("mqtt_passwd", default_value=""),
        # Bridges first: a missing script aborts the launch before the
        # navigation stack comes up.
        OpaqueFunction(function=mavros_bridge),
        OpaqueFunction(function=bringup),
        OpaqueFunction(function=visualization.bringup),
        IncludeLaunchDescription(ComponentSource(pipeline.generate_launch_description),
            launch_arguments={"robot_namespace": LaunchConfiguration("robot_namespace"),
                              "patrol_nav2_params_yaml": LaunchConfiguration("params_file"),
                              "use_sim_time": LaunchConfiguration("use_sim_time"),
                              "mqtt": LaunchConfiguration("mqtt"),
                              "simulation_targets": "true",
                              "address": LaunchConfiguration("address"),
                              "mqtt_username": LaunchConfiguration("mqtt_username"),
                              "mqtt_passwd": LaunchConfiguration("mqtt_passwd")}.items()),
    ])
