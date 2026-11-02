cmake_minimum_required(VERSION 3.8)
project(remote_override)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

add_executable(remote_override_node src/remote_override_node.cpp)

ament_target_dependencies(remote_override_node
  rclcpp
  geometry_msgs
  mavros_msgs
)

install(TARGETS remote_override_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
