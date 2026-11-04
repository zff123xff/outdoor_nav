cmake_minimum_required(VERSION 3.8)
project(path_searching)

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
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(msg_set REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

# Include directories
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${plan_env_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Build library
add_library(path_searching
  src/astar2.cpp
  src/kinodynamic_astar.cpp
  src/snn_path.cpp
  src/topo_prm.cpp
)
target_include_directories(path_searching PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(path_searching
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  plan_env
  msg_set
  cv_bridge
  Eigen3
  visualization_msgs
  pcl_ros
  pcl_conversions
)

if(BUILD_TESTING)
  add_executable(wavefront_planner_test
    test/wavefront_planner_test.cpp
  )
  target_link_libraries(wavefront_planner_test path_searching)
  ament_target_dependencies(wavefront_planner_test rclcpp Eigen3)
  add_test(NAME wavefront_planner_test COMMAND wavefront_planner_test)
endif()

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install library
install(TARGETS path_searching
  EXPORT export_path_searching
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(path_searching)
ament_export_targets(export_path_searching)

ament_package()
