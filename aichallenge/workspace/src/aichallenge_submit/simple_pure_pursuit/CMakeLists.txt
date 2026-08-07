cmake_minimum_required(VERSION 3.8)
project(simple_pure_pursuit)

find_package(autoware_cmake REQUIRED)
autoware_package()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(
  simple_pure_pursuit
  src/ay0_base_capture.cpp
  src/aw2_shadow_ipc.cpp
  src/aw2_shadow_transport.cpp
  src/controller_applied_envelope.cpp
  src/controller_execution_envelope.cpp
  src/simple_pure_pursuit.cpp
  src/state_lattice_shadow_binding.cpp
)

ament_auto_add_executable(
  controller_applied_shadow_supervisor
  src/aw2_shadow_ipc.cpp
  src/aw2_shadow_transport.cpp
  src/controller_applied_envelope.cpp
  src/controller_applied_shadow_supervisor.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(overtake_planner REQUIRED)

  add_executable(
    test_aw2_shadow_exec_consumer
    test/aw2_shadow_exec_consumer.cpp
    src/aw2_shadow_ipc.cpp
    src/aw2_shadow_transport.cpp
  )
  target_include_directories(test_aw2_shadow_exec_consumer PUBLIC include)

  add_executable(
    test_aw2_shadow_hanging_supervisor
    test/aw2_shadow_hanging_supervisor.cpp
  )

  ament_add_gtest(
    test_aw2_shadow_transport
    test/test_aw2_shadow_transport.cpp
    src/aw2_shadow_transport.cpp
  )
  target_include_directories(test_aw2_shadow_transport PUBLIC include)

  ament_add_gtest(
    test_aw2_shadow_process
    test/test_aw2_shadow_process.cpp
    src/aw2_shadow_ipc.cpp
    src/aw2_shadow_transport.cpp
  )
  target_include_directories(test_aw2_shadow_process PUBLIC include)
  add_dependencies(
    test_aw2_shadow_process
    controller_applied_shadow_supervisor
    test_aw2_shadow_exec_consumer
    test_aw2_shadow_hanging_supervisor
  )

  ament_add_gtest(
    test_aw2_shadow_supervisor_conversion
    test/test_aw2_shadow_supervisor_conversion.cpp
    src/aw2_shadow_ipc.cpp
    src/aw2_shadow_transport.cpp
    src/controller_applied_envelope.cpp
  )
  target_include_directories(
    test_aw2_shadow_supervisor_conversion
    PUBLIC include
  )
  ament_target_dependencies(
    test_aw2_shadow_supervisor_conversion
    autoware_auto_control_msgs
    autoware_auto_planning_msgs
    autoware_auto_vehicle_msgs
    geometry_msgs
    multi_purpose_mpc_ros_msgs
    motion_utils
    nav_msgs
    overtake_transport_contract
    rclcpp
    std_msgs
    tf2
  )

  ament_add_gtest(
    test_lookahead
    test/test_lookahead.cpp
    src/ay0_base_capture.cpp
    src/aw2_shadow_ipc.cpp
    src/aw2_shadow_transport.cpp
    src/controller_applied_envelope.cpp
    src/controller_execution_envelope.cpp
    src/simple_pure_pursuit.cpp
    src/state_lattice_shadow_binding.cpp
  )
  target_compile_definitions(test_lookahead PRIVATE
    SIMPLE_PURE_PURSUIT_NO_MAIN)
  target_include_directories(test_lookahead PUBLIC include)
  ament_target_dependencies(
    test_lookahead
    autoware_auto_control_msgs
    autoware_auto_planning_msgs
    autoware_auto_vehicle_msgs
    geometry_msgs
    multi_purpose_mpc_ros_msgs
    motion_utils
    nav_msgs
    overtake_planner
    overtake_transport_contract
    rclcpp
    std_msgs
    tf2
  )

  add_executable(
    simple_pure_pursuit_timing_fixture
    src/ay0_base_capture.cpp
    src/aw2_shadow_ipc.cpp
    src/aw2_shadow_transport.cpp
    src/controller_applied_envelope.cpp
    src/controller_execution_envelope.cpp
    src/simple_pure_pursuit.cpp
    src/state_lattice_shadow_binding.cpp
    test/pp_callback_span_recorder.cpp
  )
  target_compile_definitions(
    simple_pure_pursuit_timing_fixture
    PRIVATE SIMPLE_PURE_PURSUIT_TIMING_DIAGNOSTIC
  )
  target_include_directories(
    simple_pure_pursuit_timing_fixture
    PRIVATE include test
  )
  ament_target_dependencies(
    simple_pure_pursuit_timing_fixture
    autoware_auto_control_msgs
    autoware_auto_planning_msgs
    autoware_auto_vehicle_msgs
    geometry_msgs
    multi_purpose_mpc_ros_msgs
    motion_utils
    nav_msgs
    overtake_planner
    overtake_transport_contract
    rclcpp
    std_msgs
    tf2
  )

  ament_add_gtest(
    test_pp_callback_span_recorder
    test/test_pp_callback_span_recorder.cpp
    test/pp_callback_span_recorder.cpp
  )
  target_include_directories(
    test_pp_callback_span_recorder
    PRIVATE test
  )

  ament_add_gtest(
    test_state_lattice_shadow_binding
    test/test_state_lattice_shadow_binding.cpp
    src/state_lattice_shadow_binding.cpp
  )
  target_include_directories(test_state_lattice_shadow_binding PUBLIC include)
  ament_target_dependencies(
    test_state_lattice_shadow_binding
    multi_purpose_mpc_ros_msgs
    overtake_transport_contract
  )
endif()

ament_auto_package()
