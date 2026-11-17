cmake_minimum_required(VERSION 3.8)
project(quadrotor_msgs)

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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Include directories
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)

# Generate custom messages
set(msg_files
  "msg/AuxCommand.msg"
  "msg/Corrections.msg"
  "msg/Gains.msg"
  "msg/LQRTrajectory.msg"
  "msg/Odometry.msg"
  "msg/OutputData.msg"
  "msg/PolynomialTrajectory.msg"
  "msg/PositionCommand.msg"
  "msg/PPROutputData.msg"
  "msg/Serial.msg"
  "msg/SO3Command.msg"
  "msg/StatusData.msg"
  "msg/TRPYCommand.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs nav_msgs
)

# Build libraries
add_library(decode_msgs src/decode_msgs.cpp)
target_include_directories(decode_msgs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(decode_msgs rclcpp geometry_msgs nav_msgs Eigen3)

add_library(encode_msgs src/encode_msgs.cpp)
target_include_directories(encode_msgs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(encode_msgs rclcpp geometry_msgs nav_msgs Eigen3)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(TARGETS decode_msgs encode_msgs
  EXPORT export_quadrotor_msgs
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
ament_export_libraries(decode_msgs encode_msgs)
ament_export_targets(export_quadrotor_msgs)

rosidl_target_interfaces(decode_msgs
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
rosidl_target_interfaces(encode_msgs
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

ament_package()
