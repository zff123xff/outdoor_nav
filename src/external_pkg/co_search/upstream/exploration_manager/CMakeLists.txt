cmake_minimum_required(VERSION 3.8)
project(exploration_manager)

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
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(active_perception REQUIRED)
find_package(traj_utils REQUIRED)
find_package(lkh_tsp_solver REQUIRED)
find_package(lkh_mtsp_solver REQUIRED)
find_library(LKH_MTSP_SOLVER_LIB NAMES lkh_mtsp_solver_lib
  PATHS "${lkh_mtsp_solver_DIR}/../../../lib"
  NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED
)
find_package(plan_manage REQUIRED)
find_package(ppu_opt REQUIRED)
find_package(snn_region REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(msg_set REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(poly_traj REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Generate custom messages
set(msg_files
  "msg/HGrid.msg"
  "msg/GridIds.msg"
  "msg/GridTour.msg"
  "msg/DeletedGoals.msg"
  "msg/SentGoals.msg"
  "msg/PairOpt.msg"
  "msg/PairOptResponse.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

# Original algorithm is compiled by co_search_nav/search_engine.
# Export only the interface/header dependency; do not build flight/cluster nodes.
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_package()
