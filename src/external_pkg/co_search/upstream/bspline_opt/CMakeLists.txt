cmake_minimum_required(VERSION 3.8)
project(bspline_opt)

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
find_package(active_perception REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(msg_set REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)

find_path(NLOPT_INCLUDE_DIR nlopt.hpp REQUIRED)  
find_library(NLOPT_LIBRARY NAMES nlopt REQUIRED)  

# Include directories
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${plan_env_INCLUDE_DIRS}
  ${active_perception_INCLUDE_DIRS}
  ${path_searching_INCLUDE_DIRS}
  ${bspline_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIR}  
)

# Build library
add_library(bspline_opt
  src/bspline_optimizer.cpp
)
target_include_directories(bspline_opt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bspline_opt
  rclcpp
  std_msgs
  plan_env
  active_perception
  path_searching
  bspline
  Eigen3
  pcl_ros
  pcl_conversions
  msg_set
  cv_bridge
  visualization_msgs
)
target_link_libraries(bspline_opt
  ${PCL_LIBRARIES}
  ${NLOPT_LIBRARY}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install library
install(TARGETS bspline_opt
  EXPORT export_bspline_opt
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(bspline_opt ${NLOPT_LIBRARY})
ament_export_targets(export_bspline_opt)

ament_package()
