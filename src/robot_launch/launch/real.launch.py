"""One-click launch for Go2W outdoor nav: go2w_driver + PX4 + nav2 + auto gp_origin setup.

Multi-robot aware: pass robot_namespace:=robot_a (or robot_b) on the command line.
"""
from robot_launch.bringup.source import ComponentSource
from robot_launch.bringup import mavros, nav2, pipeline


import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_ref_from_yaml(yaml_path):
    """Load reference lat/lon/alt from nav2_input_bridge params."""
    with open(yaml_path, "r") as f:
        params = yaml.safe_load(f)
    bridge = params.get("nav2_input_bridge", {}).get("ros__parameters", {})
    return (
        bridge.get("reference_latitude", 30.8135718),
        bridge.get("reference_longitude", 120.8338169),
        bridge.get("reference_altitude", 12.318126322268082),
    )


def generate_launch_description():
    robot_launch_share = get_package_share_directory("robot_launch")
    nav2_params_file = LaunchConfiguration("params_file")
    declare_params_file = DeclareLaunchArgument("params_file", default_value=os.path.join(robot_launch_share, "config", "go2w_nav2.yaml"))

    declare_robot_namespace = DeclareLaunchArgument(
        "robot_namespace",
        default_value="robot_1",
        description="Per-robot namespace (e.g. robot_a, robot_b). All topics, "
                    "TFs, action servers and lifecycle nodes live under this.",
    )
    declare_use_auto_origin = DeclareLaunchArgument(
        "use_auto_origin",
        default_value="true",
        description="Whether to run the auto_set_gp_origin script on startup.",
    )
    declare_fcu_url = DeclareLaunchArgument(
        "fcu_url",
        default_value="/dev/ttyACM0:230400",
        description="MAVROS FCU URL.",
    )
    declare_use_mid360 = DeclareLaunchArgument(
        "use_mid360",
        default_value="true",
        description="Launch Livox MID360 driver.",
    )
    declare_use_foxglove = DeclareLaunchArgument(
        "use_foxglove",
        default_value="true",
        description="Start foxglove_bridge for Foxglove Studio.",
    )
    declare_use_go2w_driver = DeclareLaunchArgument(
        "use_go2w_driver",
        default_value="true",
        description="Launch go2w_driver (connects to the real dog). Set false "
                    "for HIL simulation — the simulator consumes /{ns}/cmd_vel "
                    "directly.",
    )
    declare_use_mavros = DeclareLaunchArgument(
        "use_mavros",
        default_value="true",
        description="Launch mavros (connects to the real FCU). Set false for "
                    "HIL simulation — the simulator publishes "
                    "/{ns}/mavros/odom itself.",
    )
    declare_mqtt = DeclareLaunchArgument(
        "mqtt", default_value="false",
        description="Enable MQTT task input together with the task pipeline.",
    )
    declare_address = DeclareLaunchArgument("address", default_value="tcp://127.0.0.1:1883")
    declare_mqtt_username = DeclareLaunchArgument("mqtt_username", default_value="")
    declare_mqtt_passwd = DeclareLaunchArgument("mqtt_passwd", default_value="")

    ns = LaunchConfiguration("robot_namespace")
    fcu_url = LaunchConfiguration("fcu_url")
    use_auto_origin = LaunchConfiguration("use_auto_origin")
    use_mid360 = LaunchConfiguration("use_mid360")
    use_foxglove = LaunchConfiguration("use_foxglove")
    use_go2w_driver = LaunchConfiguration("use_go2w_driver")
    use_mavros = LaunchConfiguration("use_mavros")
    mqtt = LaunchConfiguration("mqtt")

    # go2w_driver launch (namespaced). Skipped in HIL simulation: the
    # simulator reads /{ns}/cmd_vel directly and feeds back odom/pointcloud,
    # so the real-dog bridge is not needed.
    go2w_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("go2w_driver"), "launch", "go2w_driver.launch.py")
        ),
        launch_arguments={"robot_namespace": ns}.items(),
        condition=IfCondition(use_go2w_driver),
    )

    mavros_pluginlists_yaml = os.path.join(robot_launch_share, "config", "mavros_pluginlists.yaml")

    mavros_launch = IncludeLaunchDescription(
        ComponentSource(mavros.generate_launch_description),
        launch_arguments={
            "robot_namespace": ns,
            "fcu_url": fcu_url,
            "pluginlists_yaml": mavros_pluginlists_yaml,
        }.items(),
        # Skipped in HIL simulation: the simulator publishes /{ns}/mavros/odom
        # (and optionally the mavros global topics) itself.
        condition=IfCondition(use_mavros),
    )

    # odom QoS relay: mavros publishes /{ns}/mavros/odom (the local_position
    # plugin's ~/odom) as BEST_EFFORT, but Nav2 controller_server subscribes
    # RELIABLE -> nothing gets through. This relay bridges the two QoS regimes
    # by re-publishing to /{ns}/odom (RELIABLE).
    odom_relay_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mavros_odom_relay"),
                "launch",
                "mavros_odom_relay.launch.py",
            )
        ),
        launch_arguments={"robot_namespace": ns}.items(),
    )

    nav2_launch = GroupAction(actions=[IncludeLaunchDescription(
        ComponentSource(nav2.generate_launch_description),
        launch_arguments={
            "robot_namespace": ns,
            "params_file": nav2_params_file,
            "use_mid360": use_mid360,
            "use_foxglove": use_foxglove,
        }.items(),
    )])

    task_pipeline_launch = IncludeLaunchDescription(
        ComponentSource(pipeline.generate_launch_description),
        launch_arguments={
            "robot_namespace": ns,
            "patrol_nav2_params_yaml": nav2_params_file,
            "use_sim_time": "false",
            "mqtt": mqtt,
            "address": LaunchConfiguration("address"),
            "mqtt_username": LaunchConfiguration("mqtt_username"),
            "mqtt_passwd": LaunchConfiguration("mqtt_passwd"),
        }.items(),
    )

    # twist_mux routes cmd_vel_nav (priority 10) and cmd_vel_remote (priority 20)
    # to a single cmd_vel consumed by go2w_driver. Remote override wins.
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        namespace=ns,
        parameters=[{
            "topics": {
                "nav":    {"topic": "cmd_vel_nav",    "timeout": 0.5, "priority": 10},
                "remote": {"topic": "cmd_vel_remote", "timeout": 0.5, "priority": 20},
            }
        }],
        remappings=[
            # twist_mux publishes cmd_vel_out in its own namespace; remap to
            # the relative cmd_vel so it resolves to /{ns}/cmd_vel.
            ("cmd_vel_out", "cmd_vel"),
        ],
    )

    remote_override_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("remote_override"),
                "launch",
                "remote_override.launch.py",
            )
        ),
        launch_arguments={"robot_namespace": ns}.items(),
    )

    def auto_origin(context):
        ref_lat, ref_lon, ref_alt = load_ref_from_yaml(nav2_params_file.perform(context))
        return [Node(
            package="robot_launch",
            executable="auto_set_gp_origin",
            name="auto_set_gp_origin",
            namespace=ns,
            output="screen",
            parameters=[
                {
                    "latitude": ref_lat,
                    "longitude": ref_lon,
                    "altitude": ref_alt,
                    "startup_delay_s": 10.0,
                    "verify_tolerance_deg": 1e-6,
                    "max_retries": 3,
                    "verify_timeout_s": 5.0,
                    # MAVROS node lives at /{ns}/mavros. The node builds topic
                    # paths from mavros_ns and publishes to BOTH /{ns}/mavros/
                    # set_gp_origin and /{ns}/mavros/global_position/set_gp_origin
                    # (the allowlist loading drops the global_position/ prefix).
                    "mavros_ns": ns,
                }
            ],
            # auto_set_gp_origin 需要 mavros（set_gp_origin/gp_origin 话题）；
            # 仿真时（use_mavros=false）记得同时传 use_auto_origin:=false，
            # 否则它会因发不到话题空转重试后退出（无害，但刷日志）。
            condition=IfCondition(use_auto_origin),
        )]

    auto_set_origin_node = OpaqueFunction(function=auto_origin)

    return LaunchDescription(
        [
            declare_robot_namespace,
            declare_params_file,
            declare_use_auto_origin,
            declare_fcu_url,
            declare_use_mid360,
            declare_use_foxglove,
            declare_use_go2w_driver,
            declare_use_mavros,
            declare_mqtt,
            declare_address,
            declare_mqtt_username,
            declare_mqtt_passwd,
            go2w_driver_launch,
            mavros_launch,
            odom_relay_launch,
            nav2_launch,
            task_pipeline_launch,
            twist_mux_node,
            remote_override_launch,
            auto_set_origin_node,
        ]
    )
