cmake_minimum_required(VERSION 3.8)
project(lkh_mtsp_solver)

# Compile options
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3 -Wall")

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Service definition
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SolveMTSP.srv"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
)

# Source files
file(GLOB SRCS src/*.c)

# =========================
# Library
# =========================
add_library(lkh_mtsp_solver_lib STATIC
  ${SRCS}
  src2/lkh3_interface.cpp
)

target_include_directories(lkh_mtsp_solver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(lkh_mtsp_solver_lib PRIVATE -w)
target_link_libraries(lkh_mtsp_solver_lib
  m
)

# =========================
# Node executable
# =========================
add_executable(mtsp_node
  src2/mtsp_node.cpp
)

target_link_libraries(mtsp_node
  lkh_mtsp_solver_lib
  m
)

ament_target_dependencies(mtsp_node
  rclcpp
  std_msgs
)

# Ensure the node sees generated srv headers
rosidl_target_interfaces(mtsp_node
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

# =========================
# Install
# =========================
install(TARGETS lkh_mtsp_solver_lib
  EXPORT export_lkh_mtsp_solver_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS mtsp_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

# =========================
# Export
# =========================
ament_export_include_directories(include)
ament_export_libraries(lkh_mtsp_solver_lib)
ament_export_targets(export_lkh_mtsp_solver_lib)

ament_package()