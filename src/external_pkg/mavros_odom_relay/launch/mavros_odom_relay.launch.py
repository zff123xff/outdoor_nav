"""Launch the mavros_odom_relay node.

Bridges /{ns}/mavros/odom (BEST_EFFORT, from mavros local_position plugin) to
/{ns}/odom (RELIABLE) so that Nav2 controller_server can subscribe with its
default RELIABLE QoS. Without this relay Nav2 never receives odom and never
publishes /cmd_vel.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_robot_namespace = DeclareLaunchArgument(
        "robot_namespace",
        default_value="robot",
        description="Namespace for this robot instance.",
    )

    ns = LaunchConfiguration("robot_namespace")

    # Topic defaults are RELATIVE so the node's own namespace (set below)
    # prefixes them to /{robot_namespace}/mavros/odom and
    # /{robot_namespace}/odom. Using (ns, "/x") tuples would yield literal
    # /{ns}/... paths that get double-prefixed to /{ns}/{ns}/x by the
    # per-node namespace, so the relative form is required.
    args = [
        declare_robot_namespace,
        DeclareLaunchArgument(
            "input_topic",
            default_value="mavros/odom",
            description="Source odom topic (BEST_EFFORT). ROS 2 mavros's "
                        "local_position plugin publishes ~/odom (relative to "
                        "the mavros namespace), not ~/local_position/odom.",
        ),
        DeclareLaunchArgument(
            "output_topic",
            default_value="odom",
            description="Destination odom topic (RELIABLE) consumed by Nav2.",
        ),
        DeclareLaunchArgument(
            "frame_id_rewrite",
            default_value="odom",
            description="Rewrite odom header.frame_id so TF stays inside the namespace.",
        ),
    ]

    overrides = {
        "input_topic": LaunchConfiguration("input_topic"),
        "output_topic": LaunchConfiguration("output_topic"),
        "frame_id_rewrite": LaunchConfiguration("frame_id_rewrite"),
    }

    node = Node(
        package="mavros_odom_relay",
        executable="odom_relay",
        name="mavros_odom_relay",
        namespace=ns,
        output="screen",
        parameters=[overrides],
    )

    return LaunchDescription(args + [node])
