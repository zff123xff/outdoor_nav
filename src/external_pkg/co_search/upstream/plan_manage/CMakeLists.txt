cmake_minimum_required(VERSION 3.8)
project(plan_manage)

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
find_package(quadrotor_msgs REQUIRED)
find_package(poly_traj REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(active_perception REQUIRED)
find_package(traj_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(msg_set REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${poly_traj_INCLUDE_DIRS}
  ${plan_env_INCLUDE_DIRS}
  ${path_searching_INCLUDE_DIRS}
  ${bspline_INCLUDE_DIRS}
  ${bspline_opt_INCLUDE_DIRS}
  ${active_perception_INCLUDE_DIRS}
  ${traj_utils_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Build library
add_library(plan_manage
  src/planner_manager.cpp
  src/planner_manager_dev.cpp
)
target_include_directories(plan_manage PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(plan_manage
  rclcpp
  std_msgs
  quadrotor_msgs
  poly_traj
  plan_env
  path_searching
  bspline
  bspline_opt
  active_perception
  traj_utils
  Eigen3
  pcl_ros
  pcl_conversions
  msg_set
  cv_bridge
  visualization_msgs
)
target_link_libraries(plan_manage ${PCL_LIBRARIES})

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install the original planning library without flight trajectory executables.
install(TARGETS plan_manage
  EXPORT export_plan_manage
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(plan_manage)
ament_export_targets(export_plan_manage)

ament_package()
