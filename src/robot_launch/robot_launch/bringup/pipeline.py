"""Internal task pipeline composed by the two navigation entries."""

import os
import yaml

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
                                   "simulation_targets": ParameterValue(LaunchConfiguration("simulation_targets"), value_type=bool),
                                   "use_sim_time": sim_time}])
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
        DeclareLaunchArgument("area_search_config", default_value=os.path.join(
            get_package_share_directory("co_search_nav"), "config", "area_search.yaml")),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("patrol_nav2_params_yaml", default_value=os.path.join(
            get_package_share_directory("robot_launch"), "config", "go2w_nav2.yaml")),
        OpaqueFunction(function=_area_search_executor),
        *mqtt_arguments,
        dispatcher,
        primitive_mqtt,
        patrol_launch,
        multi_point,
    ])
