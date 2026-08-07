cmake_minimum_required(VERSION 3.8)
project(simple_state_lattice_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(multi_purpose_mpc_ros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(v2x_msgs REQUIRED)

add_library(${PROJECT_NAME}_core SHARED
  src/candidate_evaluator.cpp
  src/costmap_builder.cpp
  src/instant_controller.cpp
  src/lattice_generator.cpp
  src/planner_core.cpp
  src/static_input_loader.cpp
)

add_executable(simple_state_lattice_planner_node
  src/simple_state_lattice_planner_node.cpp
)
target_link_libraries(simple_state_lattice_planner_node ${PROJECT_NAME}_core)
ament_target_dependencies(simple_state_lattice_planner_node
  ament_index_cpp multi_purpose_mpc_ros_msgs nav_msgs rclcpp std_msgs v2x_msgs
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS simple_state_lattice_planner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_types_and_lattice_generator
    test/test_types_and_lattice_generator.cpp
  )
  target_link_libraries(test_types_and_lattice_generator ${PROJECT_NAME}_core)

  ament_add_gtest(test_costmap_builder test/test_costmap_builder.cpp)
  target_link_libraries(test_costmap_builder ${PROJECT_NAME}_core)

  ament_add_gtest(test_candidate_evaluator test/test_candidate_evaluator.cpp)
  target_link_libraries(test_candidate_evaluator ${PROJECT_NAME}_core)

  ament_add_gtest(test_planner_core test/test_planner_core.cpp)
  target_link_libraries(test_planner_core ${PROJECT_NAME}_core)

  ament_add_gtest(test_instant_controller test/test_instant_controller.cpp)
  target_link_libraries(test_instant_controller ${PROJECT_NAME}_core)
endif()

ament_package()
