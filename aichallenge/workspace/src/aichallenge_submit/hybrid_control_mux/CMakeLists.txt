cmake_minimum_required(VERSION 3.8)
project(hybrid_control_mux)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  hybrid_control_mux/hybrid_control_mux_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  docs
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_hybrid_control_mux_core test/test_core.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_hybrid_control_mux_launch_contracts test/test_launch_contracts.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_hybrid_control_mux_steady_timer test/test_steady_timer.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
