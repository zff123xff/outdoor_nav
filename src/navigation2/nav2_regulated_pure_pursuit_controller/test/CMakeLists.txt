# tests for regulated PP
ament_add_gtest(test_regulated_pp
  test_regulated_pp.cpp
  path_utils/path_utils.cpp
)
ament_target_dependencies(test_regulated_pp
  ${dependencies}
)
target_link_libraries(test_regulated_pp
  ${library_name}
)

# Path utils test
ament_add_gtest(test_local_plan test_local_plan.cpp)
ament_target_dependencies(test_local_plan ${dependencies})
target_link_libraries(test_local_plan ${library_name})

ament_add_gtest(test_path_utils path_utils/test_path_utils.cpp path_utils/path_utils.cpp)
ament_target_dependencies(test_path_utils nav_msgs geometry_msgs tf2_geometry_msgs)

ament_add_gtest(test_path_hysteresis test_path_hysteresis.cpp)
ament_target_dependencies(test_path_hysteresis ${dependencies})
target_link_libraries(test_path_hysteresis ${library_name})
