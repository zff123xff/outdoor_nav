cmake_minimum_required(VERSION 3.8)
project(bspline)

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
find_package(sensor_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(msg_set REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)

# Generate custom messages
set(msg_files
  "msg/Bspline.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

# Include directories
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${plan_env_INCLUDE_DIRS}
)

# Build library
add_library(bspline_lib
  src/non_uniform_bspline.cpp
)
target_include_directories(bspline_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bspline_lib
  rclcpp
  std_msgs
  sensor_msgs
  plan_env
  msg_set
  cv_bridge
  visualization_msgs
  Eigen3
  pcl_ros
  pcl_conversions
  pcl_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(bspline_lib "${cpp_typesupport_target}")

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install library
install(TARGETS bspline_lib
  EXPORT export_bspline
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
ament_export_libraries(bspline_lib)
ament_export_targets(export_bspline)

ament_package()
