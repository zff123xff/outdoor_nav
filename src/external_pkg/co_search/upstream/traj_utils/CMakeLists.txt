cmake_minimum_required(VERSION 3.8)
project(traj_utils)

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
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(path_searching REQUIRED)
find_package(poly_traj REQUIRED)
find_package(plan_env REQUIRED)
find_package(active_perception REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(msg_set REQUIRED)
find_package(cv_bridge REQUIRED)

# Include directories
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${bspline_INCLUDE_DIRS}
  ${bspline_opt_INCLUDE_DIRS}
  ${path_searching_INCLUDE_DIRS}
  ${poly_traj_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Build library
add_library(traj_utils
  src/planning_visualization.cpp
)
target_include_directories(traj_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(traj_utils
  rclcpp
  std_msgs
  bspline
  bspline_opt
  path_searching
  poly_traj
  plan_env
  active_perception
  Eigen3
  pcl_ros
  visualization_msgs
  msg_set
  cv_bridge
)
target_link_libraries(traj_utils ${PCL_LIBRARIES})

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install library
install(TARGETS traj_utils
  EXPORT export_traj_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(traj_utils)
ament_export_targets(export_traj_utils)

ament_package()
