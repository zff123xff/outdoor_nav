cmake_minimum_required(VERSION 3.8)
project(active_perception)

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
find_package(plan_env REQUIRED)
find_package(bspline REQUIRED)
find_package(path_searching REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(msg_set REQUIRED)
find_package(cv_bridge REQUIRED)

# Include directories
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${plan_env_INCLUDE_DIRS}
  ${path_searching_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${bspline_INCLUDE_DIRS}
)

# Build library
add_library(active_perception
  src/frontier_finder.cpp
  src/graph_node.cpp
  src/heading_planner.cpp
  src/hgrid.cpp
  src/perception_utils.cpp
  src/traj_visibility.cpp
  src/uniform_grid.cpp
)
target_include_directories(active_perception PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(active_perception
  rclcpp
  std_msgs
  plan_env
  path_searching
  bspline
  Eigen3
  pcl_ros
  pcl_conversions
  visualization_msgs
  msg_set
  cv_bridge
)
target_link_libraries(active_perception ${PCL_LIBRARIES})

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install library
install(TARGETS active_perception
  EXPORT export_active_perception
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_libraries(active_perception)
ament_export_targets(export_active_perception)

ament_package()
