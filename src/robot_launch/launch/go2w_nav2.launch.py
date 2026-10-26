"""Go2W Nav2 stack: nav2_input_bridge + controller_server + straight_planner + Livox MID360."""

import os
from tempfile import NamedTemporaryFile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# Param names whose values are TF frame_ids. TF frame strings are literal —
# ROS 2 never namespace-prefixes them, and tf2 REJECTS frame ids that start
# with '/' in canTransform/lookupTransform (the slash is only stripped when
# broadcasting, so lookups must use the slash-less form). A bare value like
# `map` must therefore be rewritten to {robot_namespace}/map, otherwise the
# costmap/planner look up `map`/`base_link` in a TF tree that only contains
# robot_a/map etc.
FRAME_PARAM_NAMES = {"global_frame", "robot_base_frame", "goal_output_frame"}


def _rewrite_param_value(param_name, value, prefix):
    """Namespace the VALUE of a param where the value is not resolved by ROS.

    - TF frame params (see FRAME_PARAM_NAMES): bare values get the {ns}/
      prefix WITHOUT a leading slash (tf2 rejects "/robot_a/map" in lookups;
      only "robot_a/map" works). Absolute values are left alone.
    - Root-level topic values starting with /livox/: the Livox driver is
      launched inside /{ns}/ (see mid360_driver below), so its topics move
      from /livox/lidar to /{ns}/livox/lidar and every consumer must follow.
      Topics keep their leading slash — absolute topic names are correct.
    - Everything else (relative topic names etc.) is left untouched: relative
      topics are namespace-resolved by ROS itself.
    """
    if not isinstance(value, str) or not value:
        return value
    if value.startswith("/"):
        if value.startswith("/livox/"):
            return prefix + value
        return value
    if param_name in FRAME_PARAM_NAMES:
        return prefix.lstrip("/") + "/" + value
    return value


def _rewrite_params_values(node, prefix, dotted=""):
    """Recursively rewrite param values; `dotted` tracks the param name path
    (reset at `ros__parameters`, which is not part of the param name)."""
    if isinstance(node, dict):
        out = {}
        for key, value in node.items():
            if key == "ros__parameters":
                out[key] = _rewrite_params_values(value, prefix, "")
            else:
                child_dotted = f"{dotted}.{key}" if dotted else key
                out[key] = _rewrite_params_values(value, prefix, child_dotted)
        return out
    return _rewrite_param_value(dotted, node, prefix)


def _rewrite_params_with_namespace(context, params_path_sub, ns_sub):
    """Rewrite a YAML params file so each top-level node key is prefixed
    with the namespace.

    ROS 2 launch generates per-Node override files whose top-level key is the
    full `/<ns>/<node>` path (see launch_ros/actions/node.py —
    `_create_params_file_from_dict`). User-authored YAML files normally use
    the bare node name, which doesn't match the per-Node override key, so
    every `declare_parameter(...)` in user code falls back to its C++ default
    (e.g. `node_names` triggers ParameterUninitializedException, and
    `reference_latitude` stays 0). This helper rewrites the YAML so both
    styles converge on the namespaced form, matching what launch already does.

    It also namespaces the VALUES of TF-frame params and of root-level
    /livox/ topics (see _rewrite_param_value) so the local_costmap and the
    planner see robot_a/map / robot_a/base_link / robot_a/livox/lidar instead
    of the bare single-robot names.

    Returns the path to the rewritten file. The caller doesn't need to clean
    it up — the OS reclaims the temp file on reboot.
    """
    src = params_path_sub.perform(context)
    with open(src, "r") as f:
        data = yaml.safe_load(f) or {}
    # Costmap subnodes load their own YAML; a parent-node override is not enough.
    use_clock = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    def rewrite_clock(value):
        if isinstance(value, dict):
            return {key: use_clock if key == "use_sim_time" else rewrite_clock(child)
                    for key, child in value.items()}
        if isinstance(value, list):
            return [rewrite_clock(child) for child in value]
        return value
    data = rewrite_clock(data)
    ns_str = ns_sub.perform(context) if ns_sub is not None else ""
    prefix = "/" + ns_str.lstrip("/") if ns_str else ""
    rewritten = {}
    for key, value in data.items():
        new_key = f"{prefix}/{key}" if prefix and not key.startswith("/") else key
        rewritten[new_key] = _rewrite_params_values(value, prefix) if prefix else value
    tmp = NamedTemporaryFile(mode="w", prefix="nav2_params_", suffix=".yaml", delete=False)
    yaml.dump(rewritten, tmp, default_flow_style=False)
    tmp.close()
    return tmp.name


def _make_namespaced_params(context, *args, **kwargs):
    """OpaqueFunction entry point: produce a namespaced params-file path and
    inject it back into launch_configurations as ``params_file``.
    """
    rewritten = _rewrite_params_with_namespace(
        context,
        LaunchConfiguration("params_file"),
        LaunchConfiguration("robot_namespace"),
    )
    return [SetLaunchConfiguration("params_file", rewritten)]


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_launch")
    default_params = os.path.join(pkg_share, "config", "go2w_nav2.yaml")

    # Resolve only when the real driver is enabled (HIL needs no Livox package).
    mid360_config = PathJoinSubstitution([
        FindPackageShare("livox_ros_driver2"), "config", "MID360_config.json"
    ])

    declare_robot_namespace = DeclareLaunchArgument(
        "robot_namespace",
        default_value="robot",
        description="Namespace for this robot instance. All TF frames and topics "
                    "are namespaced under this prefix.",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Real vehicle: false. Simulation: pass use_sim_time:=true and run /clock.",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Nav2 + nav2_input_bridge multi-node YAML (go2w_nav2.yaml).",
    )
    declare_use_mid360 = DeclareLaunchArgument(
        "use_mid360",
        default_value="true",
        description="Launch Livox MID360 driver (namespaced under /{robot_namespace}/).",
    )
    declare_use_foxglove = DeclareLaunchArgument(
        "use_foxglove",
        default_value="true",
        description="Start foxglove_bridge for Foxglove Studio.",
    )

    ns = LaunchConfiguration("robot_namespace")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    use_mid360 = LaunchConfiguration("use_mid360")
    use_foxglove = LaunchConfiguration("use_foxglove")

    sim_time_param = {"use_sim_time": use_sim_time}

    # Livox MID360 driver. Launched DIRECTLY (not via the vendored
    # msg_MID360_launch.py) so it runs inside the per-robot namespace: the
    # pointcloud moves from /livox/lidar to /{ns}/livox/lidar and its frame_id
    # becomes robot_a/livox_frame (slash-less) — matching the static
    # base_link->livox_frame TF below. Without this, the obstacle layer's
    # pointcloud transform fails ("livox_frame" not in the TF tree) and the
    # costmap never sees obstacles.
    mid360_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        namespace=ns,
        output="screen",
        parameters=[
            {
                "xfer_format": 0,          # 0-PointCloud2(PointXYZRTL)
                "multi_topic": 0,          # 0-All LiDARs share the same topic
                "data_src": 0,             # 0-lidar
                "publish_freq": 10.0,
                "output_data_type": 0,
                "frame_id": [ns, TextSubstitution(text='/livox_frame')],
                "lvx_file_path": "/home/livox/livox_test.lvx",
                "user_config_path": mid360_config,
                "cmdline_input_bd_code": "livox0000000001",
            },
        ],
        condition=IfCondition(use_mid360),
    )

    # 18° tilt: "0.0", "0.15643", "0.0", "0.98769"
    # base_to_livox_xyz_ypr: [0.18, 0.0, 0.53, 0.0, 0.3142, 0.0]
    # Frame ids are slash-less ("robot_a/base_link") on purpose: tf2 rejects
    # leading slashes in canTransform/lookupTransform.
    static_tf_base_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_lidar_tf",
        namespace=ns,
        arguments=[
            "0.18", "0", "0.53",
            "0.0", "0.15643", "0.0", "0.98769",
            [ns, TextSubstitution(text='/base_link')],
            [ns, TextSubstitution(text='/livox_frame')],
        ],
        parameters=[params_file, sim_time_param],
        output="screen",
    )

    # map -> odom identity TF. Both frames namespaced so they don't collide
    # across robots on the same DDS domain.
    static_tf_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_odom_tf",
        namespace=ns,
        arguments=[
            "0", "0", "0",
            "0", "0", "0", "1",
            [ns, TextSubstitution(text='/map')],
            [ns, TextSubstitution(text='/odom')],
        ],
        parameters=[params_file, sim_time_param],
        output="screen",
    )

    input_bridge_node = Node(
        package="nav2_input_bridge",
        executable="input_bridge_node",
        name="nav2_input_bridge",
        namespace=ns,
        output="screen",
        parameters=[
            params_file,
            sim_time_param,
            # GPS-converted goal PoseStamped header.frame_id must match the
            # namespaced map frame so straight_planner can transform it.
            # TF frame_id is a literal string (no ROS namespace prefixing),
            # so we construct the slash-less {robot_namespace}/map here
            # rather than passing a relative name (tf2 rejects leading '/').
            {
                "goal_output_frame": [ns, TextSubstitution(text='/map')],
            },
        ],
    )

    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        namespace=ns,
        output="screen",
        parameters=[
            params_file,
            sim_time_param,
            {
                # odom_topic is a topic name; ROS does prefix it with the
                # controller_server's namespace /{ns}/, so a RELATIVE name
                # resolves to /{ns}/odom (RELIABLE, from mavros_odom_relay).
                # Costmap2DROS spawns its own sub-node
                # (/robot_a/local_costmap/local_costmap) and reads its frames
                # from the (rewritten) params file, where
                # _rewrite_params_with_namespace prefixes them to robot_a/map
                # and robot_a/base_link. Dotted keys on THIS node never reach
                # that sub-node, so they must not be set here.
                "odom_topic": "odom",
            },
        ],
        remappings=[
            # controller_server publishes "cmd_vel" in its own namespace by
            # default; remap to cmd_vel_nav so twist_mux can pick it up.
            ("cmd_vel", "cmd_vel_nav"),
        ],
    )

    straight_planner_node = Node(
        package="nav2_straight_planner",
        executable="straight_planner_node",
        name="straight_planner",
        namespace=ns,
        output="screen",
        parameters=[
            params_file,
            sim_time_param,
            {
                # TF frame_ids — slash-less, with explicit namespace prefix
                # (tf2 rejects leading '/' in lookups).
                "global_frame": [ns, TextSubstitution(text='/map')],
                "robot_base_frame": [ns, TextSubstitution(text='/base_link')],
                # controller_server creates the action under the relative name
                # "follow_path", which resolves to /{ns}/follow_path inside
                # this namespace.
                "follow_path_action": "follow_path",
            },
        ],
        remappings=[
            # Subscribe to relative "goal_pose" so it resolves to /{ns}/goal_pose.
            ("goal_pose", "goal_pose"),
        ],
    )

    # foxglove_bridge for Foxglove Studio. Stays at the ROOT namespace; the
    # topic_whitelist is built per-robot from the launch namespace, so only
    # /{robot_namespace}/... topics are forwarded (same set as the previous
    # single-robot whitelist, just namespaced).
    #
    # IMPORTANT: the whitelist must be a LIST OF LISTS OF SUBSTITUTIONS, e.g.
    # [TextSubstitution("^/"), ns, TextSubstitution("/tf$")]. launch_ros
    # evaluates that shape into a string_array at runtime. Do NOT use an
    # f-string on `ns` (it serializes the LaunchConfiguration object, not its
    # value), and do NOT pass an empty [] (humble launch_ros normalizes an
    # empty list to () and raises "Expected 'value' ... but got '()'").
    #
    # NOTE: the regex must start with "^/" — foxglove matches against the
    # fully-qualified topic name INCLUDING its leading slash (e.g. the odom
    # topic is "/robot_a/odom"), so "^robot_a/odom$" (missing the slash)
    # matches NOTHING and the bridge silently forwards no topics.
    # (Exception: /tf and /tf_static stay at ROOT — see topic_whitelist.)
    def _wl(pattern):
        return [TextSubstitution(text='^/'), ns, TextSubstitution(text=pattern)]

    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "port": 8765,
                "address": "0.0.0.0",
                "topic_whitelist": [
                    # /tf and /tf_static are published on the ROOT topics:
                    # tf2_ros broadcasters hardcode the ABSOLUTE topic name
                    # ("/tf", "/tf_static"), independent of the node
                    # namespace — only the FRAMES inside are per-robot
                    # (robot_a/map, robot_a/odom, ...). So these two must be
                    # whitelisted at root ("^/tf$"), NOT under /{ns}/.
                    # Namespacing them (as my earlier ^/robot_a/ version did)
                    # matches nothing and foxglove silently gets no TF at all.
                    "^/tf$",
                    "^/tf_static$",
                    _wl('/odom$'),
                    _wl('/local_costmap/costmap$'),
                    _wl('/received_global_plan$'),
                    _wl('/plan$'),
                    _wl('/lookahead_point$'),
                    _wl('/lookahead_collision_arc$'),
                    _wl('/goal_pose$'),
                    _wl('/cmd_vel$'),
                ],
                "service_whitelist": ["(?!)"],
                "client_topic_whitelist": ["(?!)"],
                "param_whitelist": ["(?!)"],
                "capabilities": ["connectionGraph"],
                "tls": False,
            }
        ],
        condition=IfCondition(use_foxglove),
    )

    lc_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        namespace=ns,
        output="screen",
        parameters=[params_file, sim_time_param],
    )

    return LaunchDescription(
        [
            declare_robot_namespace,
            declare_use_sim_time,
            declare_params_file,
            declare_use_mid360,
            declare_use_foxglove,
            # Rewrite params_file so each top-level key carries the
            # /{robot_namespace}/ prefix that matches the per-Node override
            # files generated by launch_ros. Without this, lifecycle_manager
            # reports `node_names` as uninitialized and nav2_input_bridge
            # falls back to lat=0/lon=0 because the YAML top-level keys
            # (`lifecycle_manager_navigation`, `nav2_input_bridge`) don't
            # match the namespaced keys launch uses.
            OpaqueFunction(function=_make_namespaced_params),
            mid360_driver,
            static_tf_base_lidar,
            static_tf_map_odom,
            input_bridge_node,
            controller_server_node,
            straight_planner_node,
            lc_manager_node,
            foxglove_bridge_node,
        ]
    )
