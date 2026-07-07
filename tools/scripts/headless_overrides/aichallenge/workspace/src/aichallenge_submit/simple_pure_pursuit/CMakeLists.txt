cmake_minimum_required(VERSION 3.8)
project(simple_pure_pursuit)

find_package(autoware_cmake REQUIRED)
autoware_package()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(simple_pure_pursuit
  src/simple_pure_pursuit.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lookahead test/test_lookahead.cpp)
  target_include_directories(test_lookahead PUBLIC include)
  ament_target_dependencies(test_lookahead autoware_auto_planning_msgs geometry_msgs tf2)
endif()

ament_auto_package()
