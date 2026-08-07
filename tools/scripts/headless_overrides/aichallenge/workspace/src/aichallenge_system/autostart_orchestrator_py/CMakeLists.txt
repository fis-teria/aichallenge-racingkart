cmake_minimum_required(VERSION 3.8)
project(autostart_orchestrator_py)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(PROGRAMS
  autostart_orchestrator_py/autostart_orchestrator_node.py
  autostart_orchestrator_py/awsim_state_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_race_arm_latch
    test/test_race_arm_latch.py
  )
  ament_add_pytest_test(
    test_initial_pose_retry
    test/test_initial_pose_retry.py
  )
  ament_add_pytest_test(
    test_rosbag_required_topics
    test/test_rosbag_required_topics.py
  )
endif()

ament_package()
