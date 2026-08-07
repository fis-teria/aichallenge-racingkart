cmake_minimum_required(VERSION 3.8)
project(overtake_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_core SHARED
  src/aw2_plan_sample_contract.cpp
  src/behavior_state_machine.cpp
  src/blocked_risk_analyzer.cpp
  src/candidate_builder.cpp
  src/cartesian_trackability_evaluator.cpp
  src/frenet_frame.cpp
  src/future_side_by_side_risk_analyzer.cpp
  src/overtake_planner_core.cpp
  src/overtake_supervisor_v2.cpp
  src/planner_output_builder.cpp
  src/planner_publication_state.cpp
  src/pp_core_exact_snapshot.cpp
  src/safety_evaluator.cpp
  src/safety_constraint_authority.cpp
  src/state_lattice_shadow_adapter.cpp
  src/v2_localized_pass_profile_builder.cpp
)

ament_auto_add_executable(overtake_planner_node
  src/overtake_planner_node.cpp
)
target_link_libraries(overtake_planner_node ${PROJECT_NAME}_core)

install(
  PROGRAMS scripts/resolve_overtake_control_route
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frenet_frame test/test_frenet_frame.cpp)
  target_link_libraries(test_frenet_frame ${PROJECT_NAME}_core)

  ament_add_gtest(test_cartesian_trackability_evaluator
    test/test_cartesian_trackability_evaluator.cpp)
  target_link_libraries(test_cartesian_trackability_evaluator ${PROJECT_NAME}_core)

  ament_add_gtest(test_pp_core_exact_snapshot
    test/test_pp_core_exact_snapshot.cpp)
  target_link_libraries(test_pp_core_exact_snapshot ${PROJECT_NAME}_core)

  ament_add_gtest(test_safety_evaluator test/test_safety_evaluator.cpp)
  target_link_libraries(test_safety_evaluator ${PROJECT_NAME}_core)
  target_compile_definitions(test_safety_evaluator PRIVATE
    OVERTAKE_PLANNER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

  ament_add_gtest(test_state_machine test/test_state_machine.cpp)
  target_link_libraries(test_state_machine ${PROJECT_NAME}_core)

  ament_add_gtest(test_safety_constraint_authority
    test/test_safety_constraint_authority.cpp)
  target_link_libraries(test_safety_constraint_authority ${PROJECT_NAME}_core)

  ament_add_gtest(test_planner_publication_state
    test/test_planner_publication_state.cpp)
  target_link_libraries(test_planner_publication_state ${PROJECT_NAME}_core)

  ament_add_gtest(test_overtake_planner_core test/test_overtake_planner_core.cpp)
  target_link_libraries(test_overtake_planner_core ${PROJECT_NAME}_core)
  target_compile_definitions(test_overtake_planner_core PRIVATE
    OVERTAKE_PLANNER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

  ament_add_gtest(test_overtake_supervisor_v2
    test/test_overtake_supervisor_v2.cpp)
  target_link_libraries(test_overtake_supervisor_v2 ${PROJECT_NAME}_core)
  target_compile_definitions(test_overtake_supervisor_v2 PRIVATE
    OVERTAKE_PLANNER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

  ament_add_gtest(test_aw2_plan_sample_contract
    test/test_aw2_plan_sample_contract.cpp)
  target_link_libraries(test_aw2_plan_sample_contract ${PROJECT_NAME}_core)

  ament_add_gtest(test_state_lattice_shadow_adapter
    test/test_state_lattice_shadow_adapter.cpp)
  target_link_libraries(test_state_lattice_shadow_adapter ${PROJECT_NAME}_core)
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  docs
  launch
)
