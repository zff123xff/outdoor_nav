# Copyright 2026 outdoor_nav contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Launch the patrol_node.

The patrol node walks back and forth along the line segment between two GPS
points. It dispatches the endpoints to /{ns}/gp_goal (sensor_msgs/NavSatFix),
which nav2_input_bridge converts to /{ns}/goal_pose for straight_planner.

The ENU origin (reference_lat/lon/alt) is auto-loaded from `go2w_nav2.yaml`
so the arrival check always agrees with nav2_input_bridge. Override via
`nav2_params_yaml:=...` for the wheeled variant, or pass CLI args to
override individual reference_* values per-session.

Multi-robot: pass robot_namespace:=robot_a so the node publishes
/{robot_a}/gp_goal (consumed by nav2_input_bridge under the same namespace)
and subscribes to /{robot_a}/odom.
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


LOCAL_CONFIG = os.path.join(
    get_package_share_directory("patrol_node"), "config", "params.yaml"
)
DEFAULT_NAV2_PARAMS = os.path.join(
    get_package_share_directory("robot_launch"), "config", "go2w_nav2.yaml"
)


def _read_reference(yaml_path: str):
    """
    Extract nav2_input_bridge.ros__parameters.reference_* from a YAML file.

    Commented-out lines (`# ...`) are skipped so users can keep alternate
    sites as reminders in the same file.
    """
    with open(yaml_path, "r") as f:
        text = f.read()
    # Strip comments and leading whitespace so PyYAML doesn't treat them as
    # plain strings. Keep newlines so line numbers stay sane on errors.
    stripped = "\n".join(
        line.split("#", 1)[0] for line in text.splitlines()
    )
    params = yaml.safe_load(stripped) or {}
    bridge = params.get("nav2_input_bridge", {}).get("ros__parameters", {})
    return (
        float(bridge.get("reference_latitude", 0.0)),
        float(bridge.get("reference_longitude", 0.0)),
        float(bridge.get("reference_altitude", 0.0)),
    )


def generate_launch_description():
    ref_lat, ref_lon, ref_alt = _read_reference(DEFAULT_NAV2_PARAMS)

    ns = LaunchConfiguration("robot_namespace")

    args = [
        DeclareLaunchArgument(
            "robot_namespace",
            default_value="",
            description="Per-robot namespace. When set, prefixes the topic "
                        "names with /{robot_namespace}/.",
        ),
        DeclareLaunchArgument(
            "nav2_params_yaml",
            default_value=DEFAULT_NAV2_PARAMS,
            description=(
                "Path to a nav2 params yaml to auto-load reference_*. "
                "For the wheeled variant use gp_goal_nx_nav2.yaml."
            ),
        ),
        DeclareLaunchArgument("reference_latitude", default_value=str(ref_lat)),
        DeclareLaunchArgument("reference_longitude", default_value=str(ref_lon)),
        DeclareLaunchArgument("reference_altitude", default_value=str(ref_alt)),
        DeclareLaunchArgument("reach_threshold_m", default_value="0.5"),
        DeclareLaunchArgument("consecutive_required", default_value="5"),
        DeclareLaunchArgument("check_period_s", default_value="0.1"),
        DeclareLaunchArgument("feedback_period_s", default_value="0.5"),
    ]

    overrides = {
        "robot_namespace": ns,
        # RELATIVE topic names so that when this node sits in namespace
        # /{ns}/ the ROS resolver turns them into /{ns}/gp_goal, /{ns}/odom
        # and /{ns}/stop_planner.
        "goal_output_topic": "gp_goal",
        "odom_topic": "odom",
        "stop_topic": "stop_planner",
        "reference_latitude": LaunchConfiguration("reference_latitude"),
        "reference_longitude": LaunchConfiguration("reference_longitude"),
        "reference_altitude": LaunchConfiguration("reference_altitude"),
        "reach_threshold_m": LaunchConfiguration("reach_threshold_m"),
        "consecutive_required": LaunchConfiguration("consecutive_required"),
        "check_period_s": LaunchConfiguration("check_period_s"),
        "feedback_period_s": LaunchConfiguration("feedback_period_s"),
    }

    node = Node(
        package="patrol_node",
        executable="patrol_node",
        name="patrol_node",
        namespace=ns,
        output="screen",
        parameters=[LOCAL_CONFIG, overrides],
    )

    return LaunchDescription(args + [node])
