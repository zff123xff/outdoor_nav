"""Top-level launch for the task pipeline: ExecuteTask service -> /gp_goal.

Includes sub-launches under a single per-robot namespace so the topic chain is
end-to-end namespaced:

    1. execute_task_server/execute_task.launch.py    - ExecuteTask service -> /waypoint_goals
    2. multi_goal_to_gp_goal/multi_goal_to_gp_goal.launch.py
                                                     - /waypoint_goals -> /gp_goal NavSatFix
    3. patrol_node/patrol_node.launch.py             - /patrol action -> /gp_goal NavSatFix
                                                     (ping-pong patrol between two GPS points)
    4. swarms_backend (backend_nodes)   [optional]   - MQTT -> /{ns}/swarms_backend/global/multitask
    5. multi_task_bridge                [optional]   - /{ns}/swarms_backend/global/multitask
                                                       -> calls /agent/{ns}/execute_task

The namespaced topics chain together:

    /agent/{ns}/execute_task    (service - ABSOLUTE path, agent_id = namespace)
    /{ns}/waypoint_goals        (MultiGoal)
    /{ns}/gp_goal               (NavSatFix, one waypoint at a time)
    /{ns}/odom                  (input to multi_goal_to_gp_goal)
    /{ns}/patrol                (action - patrol_node/action/Patrol)

NOTE: patrol_node and multi_goal_to_gp_goal both publish to /{ns}/gp_goal.
Do not run a waypoint mission while a patrol is active — the two would
fight over the robot's goal.

4/5 are the optional MQTT command chain, gated by the single mqtt switch: they
are OFF by default (mqtt:=false) and both start together with mqtt:=true, so
no file editing is needed:

    ros2 launch robot_launch task_pipeline.launch.py robot_namespace:=robot_a \\
        mqtt:=true address:=tcp://127.0.0.1:1883 mqtt_username:=admin mqtt_passwd:=nanhu@2020

Topic-name consistency rule: usv_sn / drone_name default to robot_namespace and
must stay equal to it (keep prefix empty), so the whole chain self-namespaces
under the same /{ns}/:

    MQTT -> /{ns}/swarms_backend/global/multitask -> /agent/{ns}/execute_task
         -> /{ns}/waypoint_goals -> /{ns}/gp_goal

swarms_backend's ament package name is `backend_nodes`; build it first with
`colcon build --packages-select backend_nodes`. Both packages do NOT accept an
external ROS namespace - they self-namespace via their ID params and must run
at the root namespace.

Usage:
    ros2 launch robot_launch task_pipeline.launch.py robot_namespace:=robot_a

This script must be invoked with the same robot_namespace as launch_all_go2w.sh
so the downstream /{ns}/gp_goal is consumed by the nav2 stack under the same
namespace.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ns = LaunchConfiguration("robot_namespace")

    declare_robot_namespace = DeclareLaunchArgument(
        "robot_namespace",
        default_value="",
        description="Per-robot namespace. Pass the same value used for "
                    "launch_all_go2w.sh (e.g. robot_a) so topics align.",
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

    # 1. execute_task_server: ExecuteTask service -> MultiGoal on /waypoint_goals
    # The launch file forwards robot_namespace so the service and topic live
    # under /{ns}/agent/{ns}/execute_task and /{ns}/waypoint_goals, and sets
    # the agent_id parameter to the namespace value so the JSON-level
    # agent_id check rejects tasks meant for a different robot.
    execute_task_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("execute_task_server"),
                "launch",
                "execute_task.launch.py",
            )
        ),
        launch_arguments={
            "robot_namespace": ns,
            "agent_id": ns,
        }.items(),
    )

    # 2. multi_goal_to_gp_goal: /waypoint_goals -> /gp_goal NavSatFix
    multi_goal_to_gp_goal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("multi_goal_to_gp_goal"),
                "launch",
                "multi_goal_to_gp_goal.launch.py",
            )
        ),
        launch_arguments={"robot_namespace": ns}.items(),
    )

    # 3. patrol_node: /patrol action -> /gp_goal NavSatFix
    # Ping-pong patrol between two GPS points. Idle until a Patrol goal
    # arrives, so it is safe to always start it.
    patrol_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("patrol_node"),
                "launch",
                "patrol_node.launch.py",
            )
        ),
        launch_arguments={"robot_namespace": ns}.items(),
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
        declare_mqtt,
        declare_usv_sn,
        declare_prefix,
        declare_vehicle_id,
        declare_address,
        declare_mqtt_username,
        declare_mqtt_passwd,
        declare_drone_name,
        declare_drone_id,
        execute_task_launch,
        multi_goal_to_gp_goal_launch,
        patrol_node_launch,
        # blocks 4/5 由 mqtt:=true 一起启用（默认关闭）
        swarms_backend_launch,
        multi_task_bridge_launch,
    ])
