cmake_minimum_required(VERSION 3.8)
project(state_lattice_overtake_planner)

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
  src/c002ay0_shadow_proposal.cpp
  src/c002ay0_shadow_proposal_capture.cpp
  src/c002ay0_shadow_proposal_record.cpp
  src/c002ay0_shadow_proposal_worker.cpp
  src/config.cpp
  src/cost_model.cpp
  src/frenet_frame.cpp
  src/grid_map.cpp
  src/instant_controller.cpp
  src/lattice_planner.cpp
  src/reference_override_contract.cpp
  src/state_lattice_authority_projection.cpp
  src/shadow_geometry_generator.cpp
)

# Bind the shadow proposal's implementation provenance to the exact planner
# sources that generate and safety-evaluate its Cartesian payload.
set(AY0_IMPLEMENTATION_FINGERPRINT_INPUT "")
foreach(AY0_SOURCE
    include/state_lattice_overtake_planner/config.hpp
    include/state_lattice_overtake_planner/cost_model.hpp
    include/state_lattice_overtake_planner/frenet_frame.hpp
    include/state_lattice_overtake_planner/grid_map.hpp
    include/state_lattice_overtake_planner/lattice_planner.hpp
    include/state_lattice_overtake_planner/types.hpp
    include/state_lattice_overtake_planner/c002ay0_shadow_proposal_capture.hpp
    include/state_lattice_overtake_planner/c002ay0_shadow_proposal_record.hpp
    include/state_lattice_overtake_planner/c002ay0_shadow_proposal_worker.hpp
    src/c002ay0_shadow_proposal.cpp
    src/c002ay0_shadow_proposal_capture.cpp
    src/c002ay0_shadow_proposal_record.cpp
    src/c002ay0_shadow_proposal_worker.cpp
    src/config.cpp
    src/cost_model.cpp
    src/frenet_frame.cpp
    src/grid_map.cpp
    src/lattice_planner.cpp)
  file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/${AY0_SOURCE}" AY0_SOURCE_SHA256)
  string(APPEND AY0_IMPLEMENTATION_FINGERPRINT_INPUT
    "${AY0_SOURCE}:${AY0_SOURCE_SHA256}\n")
endforeach()
string(SHA256 AY0_IMPLEMENTATION_FINGERPRINT
  "${AY0_IMPLEMENTATION_FINGERPRINT_INPUT}")
target_compile_definitions(${PROJECT_NAME}_core PRIVATE
  STATE_LATTICE_AY0_IMPLEMENTATION_FINGERPRINT="${AY0_IMPLEMENTATION_FINGERPRINT}")

ament_auto_add_executable(state_lattice_overtake_planner_node
  src/state_lattice_overtake_planner_node.cpp
)
target_link_libraries(state_lattice_overtake_planner_node ${PROJECT_NAME}_core)

ament_auto_add_executable(c002ay0_state_lattice_shadow_worker
  src/c002ay0_state_lattice_shadow_worker.cpp
)
target_link_libraries(c002ay0_state_lattice_shadow_worker ${PROJECT_NAME}_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_config_and_cost test/test_config_and_cost.cpp)
  target_link_libraries(test_config_and_cost ${PROJECT_NAME}_core)

  ament_add_gtest(test_frenet_and_lattice test/test_frenet_and_lattice.cpp)
  target_link_libraries(test_frenet_and_lattice ${PROJECT_NAME}_core)
  target_compile_definitions(test_frenet_and_lattice PRIVATE
    STATE_LATTICE_TEST_ACCESS=1
    TEST_MPC_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../multi_purpose_mpc_ros"
    TEST_STATE_LATTICE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

  ament_add_gtest(test_contract_and_state test/test_contract_and_state.cpp)
  target_link_libraries(test_contract_and_state ${PROJECT_NAME}_core)
  target_include_directories(test_contract_and_state PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../simple_pure_pursuit/include)

  ament_add_gtest(test_state_lattice_v2_publication_state
    test/test_state_lattice_v2_publication_state.cpp)
  target_link_libraries(test_state_lattice_v2_publication_state ${PROJECT_NAME}_core)

  ament_add_gtest(test_state_lattice_v2_final_fence
    test/test_state_lattice_v2_final_fence.cpp)
  target_link_libraries(test_state_lattice_v2_final_fence ${PROJECT_NAME}_core)

  ament_add_gtest(test_shadow_geometry_generator test/test_shadow_geometry_generator.cpp)
  target_link_libraries(test_shadow_geometry_generator ${PROJECT_NAME}_core)

  ament_add_gtest(test_instant_controller test/test_instant_controller.cpp)
  target_link_libraries(test_instant_controller ${PROJECT_NAME}_core)

  ament_add_gtest(test_speed_transition_evidence
    test/test_speed_transition_evidence.cpp)
  target_link_libraries(test_speed_transition_evidence ${PROJECT_NAME}_core)

  ament_add_gtest(test_c002ay0_shadow_proposal
    test/test_c002ay0_shadow_proposal.cpp)
  target_link_libraries(test_c002ay0_shadow_proposal ${PROJECT_NAME}_core)
  add_dependencies(test_c002ay0_shadow_proposal
    c002ay0_state_lattice_shadow_worker)
  target_compile_definitions(test_c002ay0_shadow_proposal PRIVATE
    TEST_MPC_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../multi_purpose_mpc_ros"
    TEST_C002AY0_SHADOW_WORKER_PATH="$<TARGET_FILE:c002ay0_state_lattice_shadow_worker>")

  ament_add_gtest(test_c002ay0_shadow_proposal_record
    test/test_c002ay0_shadow_proposal_record.cpp)
  target_link_libraries(test_c002ay0_shadow_proposal_record ${PROJECT_NAME}_core)
  add_dependencies(test_c002ay0_shadow_proposal_record
    c002ay0_state_lattice_shadow_worker)
  target_compile_definitions(test_c002ay0_shadow_proposal_record PRIVATE
    TEST_C002AY0_SHADOW_WORKER_PATH="$<TARGET_FILE:c002ay0_state_lattice_shadow_worker>")
endif()

ament_auto_package(INSTALL_TO_SHARE config data launch docs scripts)
