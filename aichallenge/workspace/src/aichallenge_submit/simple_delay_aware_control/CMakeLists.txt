cmake_minimum_required(VERSION 3.8)
project(simple_delay_aware_control)

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
  src/control_core.cpp
)

ament_auto_add_library(${PROJECT_NAME}_exercise SHARED
  src/control_core_exercise.cpp
)

ament_auto_add_executable(simple_delay_aware_control_node
  src/simple_delay_aware_control_node.cpp
)
target_link_libraries(simple_delay_aware_control_node ${PROJECT_NAME}_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control_core test/test_control_core.cpp)
  target_link_libraries(test_control_core ${PROJECT_NAME}_core)
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
  docs
  launch
)
