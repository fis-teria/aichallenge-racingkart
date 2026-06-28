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
  src/behavior_state_machine.cpp
  src/frenet_frame.cpp
  src/overtake_planner_core.cpp
  src/safety_evaluator.cpp
)

ament_auto_add_executable(overtake_planner_node
  src/overtake_planner_node.cpp
)
target_link_libraries(overtake_planner_node ${PROJECT_NAME}_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frenet_frame test/test_frenet_frame.cpp)
  target_link_libraries(test_frenet_frame ${PROJECT_NAME}_core)

  ament_add_gtest(test_safety_evaluator test/test_safety_evaluator.cpp)
  target_link_libraries(test_safety_evaluator ${PROJECT_NAME}_core)

  ament_add_gtest(test_state_machine test/test_state_machine.cpp)
  target_link_libraries(test_state_machine ${PROJECT_NAME}_core)

  ament_add_gtest(test_overtake_planner_core test/test_overtake_planner_core.cpp)
  target_link_libraries(test_overtake_planner_core ${PROJECT_NAME}_core)
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
