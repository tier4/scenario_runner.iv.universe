cmake_minimum_required(VERSION 3.5)
project(action_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include)

ament_auto_add_library(${PROJECT_NAME} SHARED
   src/acceleration_action.cpp
   src/change_signal_action.cpp
   src/follow_route_action.cpp
   src/lane_change_action.cpp
   src/speed_action.cpp
  )

pluginlib_export_plugin_description_file(scenario_actions plugins.xml)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
