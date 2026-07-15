cmake_minimum_required(VERSION 3.8)
project(wall_recovery_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_core
  src/wall_recovery_core.cpp
)

ament_auto_add_executable(wall_recovery_planner_node
  src/wall_recovery_planner_node.cpp
)
target_link_libraries(wall_recovery_planner_node ${PROJECT_NAME}_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_wall_recovery_core test/test_wall_recovery_core.cpp)
  target_include_directories(test_wall_recovery_core PUBLIC include)
  target_link_libraries(test_wall_recovery_core ${PROJECT_NAME}_core)
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
