cmake_minimum_required(VERSION 3.8)
project(poly_traj)

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
find_package(Eigen3 REQUIRED)
# TODO: 需要移植 swarmtal_msgs - traj_generator.cpp 依赖它，暂不编译

# Include directories
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)

# Build library (排除 traj_generator.cpp 因为它依赖 swarmtal_msgs)
add_library(poly_traj
  src/polynomial_traj.cpp
)
target_include_directories(poly_traj PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(poly_traj
  rclcpp
  std_msgs
  Eigen3
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install library
install(TARGETS poly_traj
  EXPORT export_poly_traj
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(poly_traj)
ament_export_targets(export_poly_traj)

ament_package()
