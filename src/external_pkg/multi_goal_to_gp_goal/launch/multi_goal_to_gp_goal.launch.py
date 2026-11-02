"""Launch the sequential GPS queue using navigation Action completion."""

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


def _create_bridge(context):
    nav_yaml = LaunchConfiguration("nav2_params_yaml").perform(context)
    overrides = {
        "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
        "robot_namespace": LaunchConfiguration("robot_namespace"),
        "input_topic": "waypoint_goals",
        **navigation_parameters(nav_yaml),
    }
    return [Node(package="multi_goal_to_gp_goal", executable="multi_goal_to_gp_goal",
                 name="multi_goal_to_gp_goal", namespace=LaunchConfiguration("robot_namespace"),
                 output="screen", parameters=[overrides])]
def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "robot_namespace",
            default_value="",
            description="Per-robot namespace. When set, prefixes input_topic, "
                        "output_topic, and odom_topic with /{robot_namespace}/.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "nav2_params_yaml",
            default_value=DEFAULT_NAV2_PARAMS,
            description=(
                "Path to a nav2 params yaml to auto-load reference_*. "
                "Use the same navigation YAML as the active Nav2 stack."
            ),
        ),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_create_bridge)])
