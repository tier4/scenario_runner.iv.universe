cmake_minimum_required(VERSION 3.5)
project(scenario_runner)

## Compile as C++11, supported in ROS Kinetic and newer
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(yaml-cpp REQUIRED)


ament_auto_add_library(scenario_runner SHARED
  src/scenario_runner.cpp)

target_link_libraries(scenario_runner
  Boost::filesystem
  glog
)

ament_auto_add_executable(scenario_runner_node
  src/scenario_runner_node.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
        INSTALL_TO_SHARE launch
)
