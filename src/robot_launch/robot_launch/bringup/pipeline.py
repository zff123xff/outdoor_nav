"""Internal task pipeline composed by the two navigation entries."""

import os
import yaml
from yolo_detector.targets import DEFAULT_TARGETS_FILE

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
    with open(config, encoding="utf-8") as stream:
        controller = (yaml.safe_load(stream) or {}).get("search_controller", "co_search")
    if controller != "co_search":
        raise ValueError("area_search only supports search_controller: co_search")
    return [Node(package="co_search_nav", executable="co_search_executor", name="area_search_co_search",
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

    mqtt_arguments = [
        DeclareLaunchArgument("mqtt", default_value="false"),
        DeclareLaunchArgument("address", default_value="tcp://127.0.0.1:1883"),
        DeclareLaunchArgument("mqtt_username", default_value=""),
        DeclareLaunchArgument("mqtt_passwd", default_value=""),
    ]

    # Navigation owns task ingress, dispatch and all primitive executors.
    dispatcher = Node(package="task_pipeline", executable="pipeline_dispatcher",
                      namespace=ns, output="screen",
                      parameters=[{"robot_namespace": ns,
                                   "inspection_search_mode": ParameterValue(
                                       LaunchConfiguration("inspection_search_mode"), value_type=bool),
                                   "yolo_targets_file": ParameterValue(
                                       LaunchConfiguration("yolo_targets_file"), value_type=str),
                                   "simulation_targets": ParameterValue(
                                       LaunchConfiguration("simulation_targets"), value_type=bool),
                                   "target_interaction_enabled": ParameterValue(
                                       LaunchConfiguration("target_interaction_enabled"), value_type=bool),
                                   "use_sim_time": sim_time}])
    simulation_marker = Node(package="yolo_detector", executable="simulation_target_marker",
        name="simulation_target_marker", namespace=ns, output="screen",
        condition=IfCondition(LaunchConfiguration("simulation_targets")),
        parameters=[os.path.join(get_package_share_directory("yolo_detector"),
            "config", "target_marker.yaml"), {"use_sim_time": sim_time,
            "targets_file": LaunchConfiguration("yolo_targets_file")}])
    primitive_mqtt = Node(
        package="task_pipeline", executable="mqtt_task_gateway", namespace=ns,
        output="screen", condition=IfCondition(LaunchConfiguration("mqtt")),
        parameters=[{"robot_namespace": ns,
                     "address": LaunchConfiguration("address"),
                     "mqtt_username": LaunchConfiguration("mqtt_username"),
                     "mqtt_passwd": LaunchConfiguration("mqtt_passwd")}])

    multi_point = Node(package="task_pipeline", executable="multi_point_executor",
                       namespace=ns, output="screen", parameters=[{"use_sim_time": sim_time}])

    # GPS point/patrol executors issue correlated NavigateToPose actions.
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

    return LaunchDescription([
        declare_robot_namespace,
        DeclareLaunchArgument("simulation_targets", default_value="false"),
        DeclareLaunchArgument("target_interaction_enabled", default_value="false"),
        DeclareLaunchArgument("inspection_search_mode", default_value="false",
                              description="143: search and observe markers; false keeps assigned-target visits"),
        DeclareLaunchArgument("yolo_targets_file", default_value=DEFAULT_TARGETS_FILE,
                              description="Shared class file for dispatcher and YOLO; empty uses the common default"),
        DeclareLaunchArgument("inspection_config", default_value=os.path.join(
            get_package_share_directory("task_pipeline"), "config", "inspection.yaml")),
        DeclareLaunchArgument("area_search_config", default_value=os.path.join(
            get_package_share_directory("co_search_nav"), "config", "area_search.yaml")),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("patrol_nav2_params_yaml", default_value=os.path.join(
            get_package_share_directory("robot_launch"), "config", "go2w_nav2.yaml")),
        OpaqueFunction(function=_area_search_executor),
        *mqtt_arguments,
        dispatcher,
        simulation_marker,
        primitive_mqtt,
        patrol_launch,
        multi_point,
        Node(package="task_pipeline", executable="target_inspection_executor", namespace=ns,
             output="screen", parameters=[LaunchConfiguration("inspection_config"),
                                          {"use_sim_time": sim_time}]),
        Node(package="task_pipeline", executable="target_inspection_executor", name="search_target_visitor",
             namespace=ns, output="screen", condition=IfCondition(LaunchConfiguration("inspection_search_mode")),
             parameters=[LaunchConfiguration("inspection_config"), {"use_sim_time": sim_time,
                         "action_name": "search_target_inspection", "assigned_topic": "targets/search_assigned",
                         "require_fresh_assignment": True}]),
        Node(package="task_pipeline", executable="search_inspection_executor", namespace=ns,
             output="screen", condition=IfCondition(LaunchConfiguration("inspection_search_mode")),
             parameters=[LaunchConfiguration("inspection_config"), {"use_sim_time": sim_time}]),
    ])
