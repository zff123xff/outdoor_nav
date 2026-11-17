cmake_minimum_required(VERSION 3.8)
project(lkh_tsp_solver)

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
find_package(rosidl_default_generators REQUIRED)

# Generate custom services
set(srv_files
  "srv/SolveTSP.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
)

# Compile C source files (LKH solver)
# Note: LKH uses global variables shared across all files
set(LKH_SRCS
  src/Activate.c
  src/AddCandidate.c
  src/AddExtraCandidates.c
  src/AddTourCandidates.c
  src/AdjustCandidateSet.c
  src/AdjustClusters.c
  src/AllocateStructures.c
  src/Ascent.c
  src/Best2OptMove.c
  src/Best3OptMove.c
  src/Best4OptMove.c
  src/Best5OptMove.c
  src/BestKOptMove.c
  src/Between_SL.c
  src/Between_SSL.c
  src/Between.c
  src/BridgeGain.c
  src/BuildKDTree.c
  src/C.c
  src/CandidateReport.c
  src/ChooseInitialTour.c
  src/Connect.c
  src/Create_POPMUSIC_CandidateSet.c
  src/CreateCandidateSet.c
  src/CreateDelaunayCandidateSet.c
  src/CreateNNCandidateSet.c
  src/CreateQuadrantCandidateSet.c
  src/Delaunay.c
  src/Distance_SPECIAL.c
  src/Distance.c
  src/eprintf.c
  src/ERXT.c
  src/Excludable.c
  src/Exclude.c
  src/FindTour.c
  src/FixedOrCommonCandidates.c
  src/Flip_SL.c
  src/Flip_SSL.c
  src/Flip.c
  src/Forbidden.c
  src/FreeStructures.c
  src/fscanint.c
  src/Gain23.c
  src/GenerateCandidates.c
  src/Genetic.c
  src/GeoConversion.c
  src/GetTime.c
  src/gpx.c
  src/GreedyTour.c
  src/Hashing.c
  src/Heap.c
  src/IsBackboneCandidate.c
  src/IsCandidate.c
  src/IsCommonEdge.c
  src/IsPossibleCandidate.c
  src/KSwapKick.c
  src/LinKernighan.c
  src/Make2OptMove.c
  src/Make3OptMove.c
  src/Make4OptMove.c
  src/Make5OptMove.c
  src/MakeKOptMove.c
  src/MergeTourWithBestTour.c
  src/MergeWithTourGPX2.c
  src/MergeWithTourIPT.c
  src/Minimum1TreeCost.c
  src/MinimumSpanningTree.c
  src/NormalizeNodeList.c
  src/NormalizeSegmentList.c
  src/OrderCandidateSet.c
  src/PatchCycles.c
  src/printff.c
  src/PrintParameters.c
  src/Random.c
  src/ReadCandidates.c
  src/ReadEdges.c
  src/ReadLine.c
  src/ReadParameters.c
  src/ReadPenalties.c
  src/ReadProblem.c
  src/RecordBestTour.c
  src/RecordBetterTour.c
  src/RemoveFirstActive.c
  src/ResetCandidateSet.c
  src/RestoreTour.c
  src/SegmentSize.c
  src/Sequence.c
  src/SFCTour.c
  src/SolveCompressedSubproblem.c
  src/SolveDelaunaySubproblems.c
  src/SolveKarpSubproblems.c
  src/SolveKCenterSubproblems.c
  src/SolveKMeansSubproblems.c
  src/SolveRoheSubproblems.c
  src/SolveSFCSubproblems.c
  src/SolveSubproblem.c
  src/SolveSubproblemBorderProblems.c
  src/SolveTourSegmentSubproblems.c
  src/Statistics.c
  src/StoreTour.c
  src/SymmetrizeCandidateSet.c
  src/TrimCandidateSet.c
  src/WriteCandidates.c
  src/WritePenalties.c
  src/WriteTour.c
)

# Build library as STATIC to avoid symbol clashes with other packages
add_library(lkh_tsp_solver_lib STATIC
  ${LKH_SRCS}
  src/lkh_interface.cpp
)
target_include_directories(lkh_tsp_solver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(lkh_tsp_solver_lib PRIVATE -w)
target_link_libraries(lkh_tsp_solver_lib -lm)
ament_target_dependencies(lkh_tsp_solver_lib rclcpp std_msgs)

# Note: tsp_node executable disabled due to complex LKH library dependencies
# The library can be used directly by other packages

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install library
install(TARGETS lkh_tsp_solver_lib
  EXPORT export_lkh_tsp_solver_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install srv files
install(DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_libraries(lkh_tsp_solver_lib)
ament_export_targets(export_lkh_tsp_solver_lib)

rosidl_target_interfaces(lkh_tsp_solver_lib
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

ament_package()
