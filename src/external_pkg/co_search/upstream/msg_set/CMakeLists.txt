cmake_minimum_required(VERSION 3.8)
project(msg_set)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# TODO: 需要安装 ros-humble-mavros-msgs
# find_package(mavros_msgs REQUIRED)

# Generate custom messages
set(msg_files
  "msg/DroneState.msg"
  "msg/CarControl.msg"
  "msg/CarState.msg"
  "msg/KinematicState.msg"
  "msg/Polynome.msg"
  "msg/SimulatedCarState.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs nav_msgs builtin_interfaces
  # TODO: 需要安装 ros-humble-mavros-msgs
  # mavros_msgs
)

# Include directories
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install msg files
install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
