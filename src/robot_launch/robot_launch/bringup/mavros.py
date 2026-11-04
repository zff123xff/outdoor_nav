"""Custom MAVROS PX4 launch that supports a per-robot namespace."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    mavros_share = get_package_share_directory("mavros")
    robot_launch_share = get_package_share_directory("robot_launch")

    default_pluginlists = os.path.join(
        robot_launch_share, "config", "mavros_pluginlists.yaml"
    )
    default_config = os.path.join(mavros_share, "launch", "px4_config.yaml")

    declare_robot_namespace = DeclareLaunchArgument(
        "robot_namespace",
        default_value="",
        description="Namespace for this robot instance. MAVROS node lives at "
                    "/{robot_namespace}/mavros. Empty default keeps the legacy "
                    "single-robot (root) behavior for real.launch.py.",
    )
    declare_fcu_url = DeclareLaunchArgument(
        "fcu_url", default_value="/dev/ttyACM0:230400"
    )
    declare_gcs_url = DeclareLaunchArgument(
        "gcs_url", default_value=""
    )
    declare_pluginlists_yaml = DeclareLaunchArgument(
        "pluginlists_yaml", default_value=default_pluginlists
    )
    declare_config_yaml = DeclareLaunchArgument(
        "config_yaml", default_value=default_config
    )

    ns = LaunchConfiguration("robot_namespace")
    fcu_url = LaunchConfiguration("fcu_url")
    gcs_url = LaunchConfiguration("gcs_url")
    pluginlists_yaml = LaunchConfiguration("pluginlists_yaml")
    config_yaml = LaunchConfiguration("config_yaml")

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        name="mavros",
        namespace=ns,
        output="screen",
        remappings=[("mavros/global", "mavros/global_position/global")],
        parameters=[
            {"fcu_url": fcu_url},
            {"gcs_url": gcs_url},
            {"tgt_system": 1},
            {"tgt_component": 1},
            {"fcu_protocol": "v2.0"},
            # === EKF2 TF frame_id override ===
            # MAVROS local_position plugin reads `local_position.tf.frame_id`
            # and `local_position.tf.child_frame_id` as flat dotted keys
            # (per px4_config.yaml layout). The values are literal frame
            # strings — ROS 2 does not namespace-prefix them — so we build
            # {robot_namespace}/odom explicitly (slash-less; tf2 rejects
            # leading '/' in lookups). With an empty namespace (legacy
            # single-robot launches) this falls back to odom/base_link.
            #
            # NOTE: `local_position.tf.send` is intentionally left false.
            # The installed mavros C++ plugin DOES support broadcasting the
            # odom->base_link TF when tf.send=true (publish_tf() exists in
            # local_position.cpp / libmavros_plugins.so), but this stack
            # broadcasts /{ns}/odom -> /{ns}/base_link from nav2_input_bridge
            # instead (it already consumes the same odom and its broadcast is
            # independent of mavros build/version quirks). Enabling both would
            # publish the same transform twice — harmless but redundant.
            {
                "local_position.tf.frame_id": [ns, TextSubstitution(text='/odom')],
                "local_position.tf.child_frame_id": [ns, TextSubstitution(text='/base_link')],
            },
            pluginlists_yaml,
            config_yaml,
        ],
    )

    return LaunchDescription(
        [
            declare_robot_namespace,
            declare_fcu_url,
            declare_gcs_url,
            declare_pluginlists_yaml,
            declare_config_yaml,
            mavros_node,
            Node(package="robot_launch", executable="mavros_global_rate",
                 namespace=ns, output="screen"),
        ]
    )
