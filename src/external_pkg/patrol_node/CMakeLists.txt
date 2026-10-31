cmake_minimum_required(VERSION 3.8)
project(patrol_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# --- Action interface -------------------------------------------------------
# The patrol action is defined by this package: the node is its only user, so
# there is no separate interface package. The generated type is
# patrol_node/action/Patrol.
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Patrol.action"
  DEPENDENCIES geometry_msgs sensor_msgs
)

# --- Python node ------------------------------------------------------------
# The Python module is installed next to the generated action, i.e. both
# patrol_node.patrol_node and patrol_node.action end up in the same
# site-packages/patrol_node package.
# ament_python_install_package() cannot be used for this: rosidl_generator_py
# already installs a Python package with this name and the macro refuses to
# install the same name twice.
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/patrol_node/
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# Console entry point (the ament_python console_scripts entry point).
install(PROGRAMS
  scripts/patrol_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Launch / config / package index ----------------------------------------
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # PYTEST_DISABLE_PLUGIN_AUTOLOAD sidesteps a pre-existing clash between the
  # pytest version installed here and ROS' launch_testing plugin.
  ament_add_pytest_test(test_ping_pong_queue test/test_ping_pong_queue.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1")
endif()

ament_package()
