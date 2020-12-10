cmake_minimum_required(VERSION 3.5)
project(scenario_api_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(yaml-cpp REQUIRED)

ament_auto_add_library(scenario_api_utils SHARED
  src/scenario_api_utils.cpp
)
target_link_libraries(scenario_api_utils
  ${YAML_CPP_LIBRARIES}
)

# Install python script
install(PROGRAMS scripts/ros_time_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)