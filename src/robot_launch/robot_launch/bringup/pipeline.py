"""Internal task pipeline composed by the two navigation entries."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _area_search_executor(context):
    config = LaunchConfiguration("area_search_config").perform(context)
    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    return [Node(package="task_pipeline", executable="area_search", name="area_search",
                 namespace=LaunchConfiguration("robot_namespace"), output="screen",
                 parameters=[{"config_file": config, "use_sim_time": sim_time}])]


def generate_launch_description():
    ns = LaunchConfiguration("robot_namespace")
    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    declare_robot_namespace = DeclareLaunchArgument(
        "robot_namespace",
        default_value="",
        description="Per-robot namespace. Pass the same value used for "
                    "system launch (e.g. robot_a) so topics align.",
    )

    # ===== MQTT 指控链路（blocks 3/4）开关与参数 =====
    # 单个 mqtt 开关同时启用 swarms_backend 与 multi_task_bridge；
    # 默认 false（两个块不启动），mqtt:=true 一起启动，无需改本文件。
    declare_mqtt = DeclareLaunchArgument(
        "mqtt",
        default_value="false",
        description="Enable the MQTT command chain: swarms_backend (backend_nodes) "
                    "and multi_task_bridge together.",
    )
    declare_usv_sn = DeclareLaunchArgument(
        "usv_sn",
        default_value=LaunchConfiguration("robot_namespace"),
        description="swarms_backend SN; must equal robot_namespace (and keep "
                    "prefix empty) so it self-namespaces to /{ns}/.",
    )
    declare_prefix = DeclareLaunchArgument(
        "prefix",
        default_value="",
        description="swarms_backend topic prefix; keep empty so the namespace "
                    "is exactly /{usv_sn}/.",
    )
    declare_vehicle_id = DeclareLaunchArgument(
        "vehicle_id",
        default_value="1",
        description="swarms_backend traditional-mode ID; must match the "
                    "drones_id in MQTT messages.",
    )
    declare_address = DeclareLaunchArgument(
        "address",
        default_value="tcp://127.0.0.1:1883",
        description="MQTT broker URL.",
    )
    declare_mqtt_username = DeclareLaunchArgument(
        "mqtt_username",
        default_value="admin",
        description="MQTT username.",
    )
    declare_mqtt_passwd = DeclareLaunchArgument(
        "mqtt_passwd",
        default_value="nanhu@2020",
        description="MQTT password.",
    )
    declare_drone_name = DeclareLaunchArgument(
        "drone_name",
        default_value=LaunchConfiguration("robot_namespace"),
        description="multi_task_bridge robot name; must equal robot_namespace "
                    "so it calls /agent/{ns}/execute_task.",
    )
    declare_drone_id = DeclareLaunchArgument(
        "drone_id",
        default_value="1",
        description="multi_task_bridge numeric drone id; must match the "
                    "drones_id in MQTT messages.",
    )

    # Navigation owns task ingress, dispatch and all primitive executors.
    dispatcher = Node(package="task_pipeline", executable="pipeline_dispatcher",
                      namespace=ns, output="screen",
                      parameters=[{"robot_namespace": ns, "agent_id": ns,
                                   "use_sim_time": sim_time}])
    primitive_mqtt = Node(
        package="task_pipeline", executable="mqtt_task_gateway", namespace=ns,
        output="screen", condition=IfCondition(LaunchConfiguration("mqtt")),
        parameters=[{"robot_namespace": ns,
                     "vehicle_id": ParameterValue(LaunchConfiguration("vehicle_id"), value_type=int),
                     "address": LaunchConfiguration("address"),
                     "mqtt_username": LaunchConfiguration("mqtt_username"),
                     "mqtt_passwd": LaunchConfiguration("mqtt_passwd")}])

    # multi_goal_to_gp_goal: /waypoint_goals -> /gp_goal NavSatFix
    multi_goal_to_gp_goal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("multi_goal_to_gp_goal"),
                "launch",
                "multi_goal_to_gp_goal.launch.py",
            )
        ),
        launch_arguments={
            "robot_namespace": ns,
            "nav2_params_yaml": LaunchConfiguration("patrol_nav2_params_yaml"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    # Primitive executors expose Actions to the pipeline dispatcher; each emits
    # one-point MultiGoal messages which join the shared navigation adapter.
    patrol_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("patrol_node"),
                "launch",
                "patrol_node.launch.py",
            )
        ),
        launch_arguments={
            "robot_namespace": ns,
            "nav2_params_yaml": LaunchConfiguration("patrol_nav2_params_yaml"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    # ===================== optional: MQTT command chain =====================
    # Both blocks are gated by the single mqtt switch (mqtt:=true to enable);
    # execute_task can still be called manually when mqtt:=false.
    # NOTE: neither package accepts an external ROS namespace; both must run at
    # the ROOT namespace and self-namespace via their ID params. The IDs must
    # match robot_namespace (robot_a), and multi_task_bridge's drone_id must
    # correspond to the numeric drones_id in MQTT messages (default 1 = robot_a).

    # 3. swarms_backend (ament package name: backend_nodes): MQTT ->
    #    /{ns}/swarms_backend/global/multitask
    #    !!! build first: colcon build --packages-select backend_nodes
    #    usv_sn:=robot_a + prefix:="" -> swarms_backend self-namespace = /robot_a
    #    由 mqtt:=true 启用。
    swarms_backend_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("backend_nodes"),
                "launch",
                "backend_nodes.launch.py",
            )
        ),
        launch_arguments={
            "usv_sn": LaunchConfiguration("usv_sn"),
            "prefix": LaunchConfiguration("prefix"),
            "vehicle_id": LaunchConfiguration("vehicle_id"),
            "address": LaunchConfiguration("address"),
            "mqtt_username": LaunchConfiguration("mqtt_username"),
            "mqtt_passwd": LaunchConfiguration("mqtt_passwd"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("mqtt")),
    )

    # 4. multi_task_bridge: /{ns}/swarms_backend/global/multitask ->
    #    calls /agent/{ns}/execute_task
    #    由 mqtt:=true 启用。
    multi_task_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("multi_task_bridge"),
                "launch",
                "multi_task_bridge.launch.py",
            )
        ),
        launch_arguments={
            "drone_name": LaunchConfiguration("drone_name"),
            "drone_id": LaunchConfiguration("drone_id"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("mqtt")),
    )

    return LaunchDescription([
        declare_robot_namespace,
        DeclareLaunchArgument("area_search_config", default_value=os.path.join(
            get_package_share_directory("task_pipeline"), "config", "area_search.yaml")),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("patrol_nav2_params_yaml", default_value=os.path.join(
            get_package_share_directory("robot_launch"), "config", "go2w_nav2.yaml")),
        OpaqueFunction(function=_area_search_executor),
        declare_mqtt,
        declare_usv_sn,
        declare_prefix,
        declare_vehicle_id,
        declare_address,
        declare_mqtt_username,
        declare_mqtt_passwd,
        declare_drone_name,
        declare_drone_id,
        dispatcher,
        primitive_mqtt,
        patrol_launch,
        multi_goal_to_gp_goal_launch,
        # blocks 4/5 由 mqtt:=true 一起启用（默认关闭）
        swarms_backend_launch,
        multi_task_bridge_launch,
    ])
