cmake_minimum_required(VERSION 3.8)
project(task_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SearchArea.msg" "msg/Task.msg" "msg/TaskStatus.msg"
  "msg/TargetInfo.msg" "msg/TargetArray.msg" "msg/TargetEvent.msg" "msg/InspectionArrival.msg"
  "action/MultiPoint.action" "action/AreaSearch.action" "action/InspectTargets.action"
  "action/ApproachTarget.action"
  DEPENDENCIES sensor_msgs std_msgs geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
