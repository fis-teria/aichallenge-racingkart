cmake_minimum_required(VERSION 3.8)
project(overtake_transport_contract)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/c002ay0_canonical.cpp
  src/c002ay0_fixed_record.cpp
  src/c002ay0_shadow_binding.cpp
  src/c002ay0_worker_ipc.cpp
  src/c002ay1_runtime_observer.cpp
  src/state_lattice_v2_binding.cpp
)

ament_auto_add_executable(c002ay0_shadow_worker
  src/c002ay0_shadow_worker.cpp
)
target_link_libraries(c002ay0_shadow_worker ${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(rclcpp REQUIRED)
  find_package(rmw REQUIRED)
  find_package(Threads REQUIRED)
  ament_add_gtest(test_c002ay0_canonical
    test/test_c002ay0_canonical.cpp
  )
  target_link_libraries(test_c002ay0_canonical ${PROJECT_NAME})
  ament_add_gtest(test_state_lattice_v2_binding
    test/test_state_lattice_v2_binding.cpp
  )
  target_link_libraries(test_state_lattice_v2_binding ${PROJECT_NAME})
  ament_add_gtest(test_c002ay0_phase1_measurement
    test/test_c002ay0_phase1_measurement.cpp
  )
  target_link_libraries(test_c002ay0_phase1_measurement ${PROJECT_NAME})
  ament_target_dependencies(test_c002ay0_phase1_measurement rclcpp)
  ament_add_gtest(test_c002ay0_fixed_ingress
    test/test_c002ay0_fixed_ingress.cpp
  )
  target_link_libraries(test_c002ay0_fixed_ingress ${PROJECT_NAME})
  ament_add_gtest(test_c002ay0_fixed_record
    test/test_c002ay0_fixed_record.cpp
  )
  target_link_libraries(test_c002ay0_fixed_record ${PROJECT_NAME})
  ament_target_dependencies(test_c002ay0_fixed_record rclcpp)
  ament_add_gtest(test_c002ay0_worker_ipc
    test/test_c002ay0_worker_ipc.cpp
  )
  target_link_libraries(test_c002ay0_worker_ipc ${PROJECT_NAME})
  add_dependencies(test_c002ay0_worker_ipc c002ay0_shadow_worker)
  ament_add_gtest(test_c002ay0_shadow_binding_runtime
    test/test_c002ay0_shadow_binding_runtime.cpp
  )
  target_link_libraries(test_c002ay0_shadow_binding_runtime ${PROJECT_NAME})
  ament_target_dependencies(
    test_c002ay0_shadow_binding_runtime
    rclcpp
    std_msgs
  )
  add_dependencies(
    test_c002ay0_shadow_binding_runtime
    c002ay0_shadow_worker
  )
  set_tests_properties(
    test_c002ay0_shadow_binding_runtime
    PROPERTIES
      ENVIRONMENT "ROS_DOMAIN_ID=224;ROS_LOCALHOST_ONLY=1;CYCLONEDDS_URI="
      RUN_SERIAL TRUE
      TIMEOUT 45
  )
  ament_add_gtest(test_c002ay1_cadence_gate
    test/test_c002ay1_cadence_gate.cpp
  )
  target_link_libraries(test_c002ay1_cadence_gate ${PROJECT_NAME})
  ament_add_gtest(test_c002ay1_runtime_observer
    test/test_c002ay1_runtime_observer.cpp
  )
  target_link_libraries(test_c002ay1_runtime_observer ${PROJECT_NAME})
  add_executable(c002ay0_phase1_rmw_measurement
    test/c002ay0_phase1_rmw_measurement.cpp
  )
  target_link_libraries(c002ay0_phase1_rmw_measurement ${PROJECT_NAME})
  ament_target_dependencies(c002ay0_phase1_rmw_measurement rclcpp rmw)
  add_executable(c002ay0_worker_rmw_measurement
    test/c002ay0_worker_rmw_measurement.cpp
  )
  target_link_libraries(
    c002ay0_worker_rmw_measurement
    ${PROJECT_NAME}
    Threads::Threads
  )
  ament_target_dependencies(c002ay0_worker_rmw_measurement rclcpp rmw)
  add_executable(c002ay0_exact_proposal_relay
    test/c002ay0_exact_proposal_relay.cpp
  )
  target_link_libraries(c002ay0_exact_proposal_relay ${PROJECT_NAME})
  ament_target_dependencies(c002ay0_exact_proposal_relay rclcpp)
  add_executable(c002ay1_cadence_rmw_measurement
    test/c002ay0_worker_rmw_measurement.cpp
  )
  target_compile_definitions(
    c002ay1_cadence_rmw_measurement
    PRIVATE C002AY1_CADENCE_SEPARATED=1
  )
  target_link_libraries(
    c002ay1_cadence_rmw_measurement
    ${PROJECT_NAME}
    Threads::Threads
  )
  ament_target_dependencies(c002ay1_cadence_rmw_measurement rclcpp rmw)
  add_executable(c002ay1_runtime_collector
    test/c002ay1_runtime_collector.cpp
  )
  target_link_libraries(c002ay1_runtime_collector ${PROJECT_NAME})
  add_executable(c002ay1_runtime_producer_fixture
    test/c002ay1_runtime_producer_fixture.cpp
  )
  target_link_libraries(c002ay1_runtime_producer_fixture ${PROJECT_NAME})
  add_executable(c002ay1_runtime_attach_probe
    test/c002ay1_runtime_attach_probe.cpp
  )
  target_link_libraries(c002ay1_runtime_attach_probe ${PROJECT_NAME})
  add_test(
    NAME test_c002ay0_no_live_wiring
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_no_live_wiring.py
      ${CMAKE_CURRENT_SOURCE_DIR}/..
  )
  add_test(
    NAME test_c002ay1_no_live_graph_parity
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_c002ay1_no_live_graph_parity.py
      ${CMAKE_CURRENT_SOURCE_DIR}/..
  )
  add_test(
    NAME test_c002ay1_runtime_collector
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_c002ay1_runtime_collector.py
      $<TARGET_FILE:c002ay1_runtime_collector>
      $<TARGET_FILE:c002ay1_runtime_producer_fixture>
  )
  add_test(
    NAME test_c002ay1_runtime_attach_negative
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_c002ay1_runtime_attach_negative.py
      $<TARGET_FILE:c002ay1_runtime_attach_probe>
  )
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
