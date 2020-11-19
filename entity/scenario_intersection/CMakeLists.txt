cmake_minimum_required(VERSION 3.5)
project(scenario_intersection)

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

### Target executable
set(SCENARIO_INTERSECTION_SRC
  src/arrow.cpp
  src/color.cpp
  src/intersection.cpp
  src/intersection_manager.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${SCENARIO_INTERSECTION_SRC}
)

ament_auto_package()
