cmake_minimum_required(VERSION 3.5)
project(scenario_conditions)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(SCENARIO_CONDITIONS_SRC
  src/condition_visualizer.cpp
)

ament_auto_add_library(scenario_conditions SHARED
  src/condition_manager.cpp 
  ${SCENARIO_CONDITIONS_SRC}
)

ament_auto_package()

