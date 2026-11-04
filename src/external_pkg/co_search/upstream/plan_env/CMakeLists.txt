cmake_minimum_required(VERSION 3.8)
project(plan_env)

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
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(msg_set REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Generate custom messages
set(msg_files
  "msg/IdxList.msg"
  "msg/ChunkStamps.msg"
  "msg/ChunkData.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)

# Include directories
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

# Build library (rename to avoid conflict with rosidl target)
# Note: Some source files need more ROS1->ROS2 migration, so we exclude them for now
add_library(plan_env_lib
  src/raycast.cpp
  src/edt_environment.cpp
  src/obj_predictor.cpp
  src/sdf_map.cpp
  src/map_ros.cpp
  src/multi_map_manager.cpp
)
target_include_directories(plan_env_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(plan_env_lib PUBLIC cxx_std_14)
ament_target_dependencies(plan_env_lib
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  cv_bridge
  message_filters
  msg_set
  pcl_ros
  pcl_conversions
  OpenCV
  Eigen3
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(plan_env_lib ${PCL_LIBRARIES})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(plan_env_lib "${cpp_typesupport_target}")

# Upstream ground-station test lives outside the vendored single-robot packages.
# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install library
install(TARGETS plan_env_lib
  EXPORT plan_env_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install msg files
install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_libraries(plan_env_lib)
ament_export_targets(plan_env_lib HAS_LIBRARY_TARGET)

ament_package()
