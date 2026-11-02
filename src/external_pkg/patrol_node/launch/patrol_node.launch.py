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

"""Launch GPS point/patrol executors using correlated NavigateToPose results."""

import os

from multi_goal_to_gp_goal.navigation_client import navigation_parameters

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_NAV2_PARAMS = os.path.join(
    get_package_share_directory("robot_launch"), "config", "go2w_nav2.yaml"
)


def _create_executors(context):
    nav_yaml = LaunchConfiguration("nav2_params_yaml").perform(context)
    ns = LaunchConfiguration("robot_namespace")
    common = {
        "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
        **navigation_parameters(nav_yaml),
        "feedback_period_s": LaunchConfiguration("feedback_period_s"),
    }
    node = Node(package="patrol_node", executable="patrol_node", name="patrol_node",
                namespace=ns, output="screen", parameters=[{
                    "robot_namespace": ns, "action_name": "patrol",

                    "odom_topic": "odom",  **common,
                }])
    point_goal_node = Node(package="patrol_node", executable="point_goal_node",
                           name="point_goal_executor", namespace=ns, output="screen",
                           parameters=[{
                               "action_name": "point_goal",

                               "odom_topic": "odom",
                               **common,
                           }])
    return [node, point_goal_node]


def generate_launch_description():
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
                "Use the same navigation YAML as the active Nav2 stack."
            ),
        ),
        DeclareLaunchArgument("feedback_period_s", default_value="0.5"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_create_executors)])
