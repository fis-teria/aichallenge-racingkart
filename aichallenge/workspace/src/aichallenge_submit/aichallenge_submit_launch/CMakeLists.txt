cmake_minimum_required(VERSION 3.5)
project(aichallenge_submit_launch)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME test_c002ay1_runtime_graph_output_parity
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_c002ay1_runtime_graph_output_parity.py
  )
  set_tests_properties(
    test_c002ay1_runtime_graph_output_parity
    PROPERTIES
      ENVIRONMENT
        "ROS_DOMAIN_ID=221;ROS_LOCALHOST_ONLY=1;CYCLONEDDS_URI="
      TIMEOUT 60
  )
  add_test(
    NAME test_c002ay0_pp_runtime_measurement
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/c002ay0_pp_runtime_measurement.py
  )
  set_tests_properties(
    test_c002ay0_pp_runtime_measurement
    PROPERTIES
      ENVIRONMENT
        "ROS_DOMAIN_ID=222;ROS_LOCALHOST_ONLY=1;CYCLONEDDS_URI="
      TIMEOUT 120
  )
  add_test(
    NAME test_c002ay0_real_state_lattice_e2e_wcet
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/c002ay0_pp_runtime_measurement.py
  )
  set_tests_properties(
    test_c002ay0_real_state_lattice_e2e_wcet
    PROPERTIES
      ENVIRONMENT
        "C002AY0_REAL_STATE_LATTICE_E2E_WCET_ONLY=1;ROS_DOMAIN_ID=223;ROS_LOCALHOST_ONLY=1;CYCLONEDDS_URI="
      RUN_SERIAL TRUE
      TIMEOUT 60
  )
  add_test(
    NAME test_gnss_covariance_launch_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_gnss_covariance_launch_contract.py
  )
  add_test(
    NAME test_state_lattice_nonlive_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_state_lattice_nonlive_contract.py
  )
  add_test(
    NAME test_state_lattice_test_only_observer_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_state_lattice_test_only_observer_contract.py
  )
  add_test(
    NAME test_c002ay0_reanchor_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_c002ay0_reanchor_contract.py
  )
  set_tests_properties(
    test_c002ay0_reanchor_contract
    PROPERTIES TIMEOUT 10
  )
  add_test(
    NAME test_c002ay0_raw_handoff_unit
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_c002ay0_raw_handoff_microgate.py
  )
  set_tests_properties(
    test_c002ay0_raw_handoff_unit
    PROPERTIES TIMEOUT 10
  )
  add_test(
    NAME test_c002ay0_raw_handoff_integration
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_c002ay0_raw_handoff_microgate.py
  )
  set_tests_properties(
    test_c002ay0_raw_handoff_integration
    PROPERTIES
      ENVIRONMENT
        "C002AY0_RAW_HANDOFF_INTEGRATION=1;ROS_DOMAIN_ID=230;ROS_LOCALHOST_ONLY=1;CYCLONEDDS_URI="
      RUN_SERIAL TRUE
      TIMEOUT 60
  )
  add_test(
    NAME test_c002ay0_isolated_input_publisher_unit
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_c002ay0_isolated_input_publisher.py
  )
  set_tests_properties(
    test_c002ay0_isolated_input_publisher_unit
    PROPERTIES TIMEOUT 10
  )
  add_test(
    NAME test_c002ay0_isolated_input_publisher_integration
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_c002ay0_isolated_input_publisher.py
  )
  set_tests_properties(
    test_c002ay0_isolated_input_publisher_integration
    PROPERTIES
      ENVIRONMENT
        "C002AY0_ISOLATED_PUBLISHER_INTEGRATION=1;ROS_DOMAIN_ID=231;ROS_LOCALHOST_ONLY=1;CYCLONEDDS_URI="
      RUN_SERIAL TRUE
      TIMEOUT 30
  )
endif()

ament_auto_package(INSTALL_TO_SHARE launch map data config)
