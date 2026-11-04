cmake_minimum_required(VERSION 3.8)
project(ppu_opt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(path_searching REQUIRED)
find_package(snn_region REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${path_searching_INCLUDE_DIRS}
  ${snn_region_INCLUDE_DIRS}
)

# ── 协议 & 客户端（不含 ROS 依赖的纯 TCP） ──
add_library(ppu_protocol
  src/ppu_path_client.cpp
  src/ppu_region_client.cpp
)
target_include_directories(ppu_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ppu_protocol)

# ── 完整 ppu_opt 库（含抽象接口 + 工厂 + 本地适配器） ──
add_library(ppu_opt
  src/ppu_path_client.cpp
  src/ppu_region_client.cpp
  src/snn_path_interface.cpp
  src/snn_region_interface.cpp
)
target_include_directories(ppu_opt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(ppu_opt
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
)
target_link_libraries(ppu_opt
  path_searching
  snn_region
)

# ── Install targets ──
install(TARGETS
  ppu_protocol
  ppu_opt
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(ppu_opt)
ament_package()