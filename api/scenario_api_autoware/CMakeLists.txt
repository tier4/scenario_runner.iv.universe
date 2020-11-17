cmake_minimum_required(VERSION 3.5)
project(scenario_api_autoware)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Go for shared to avoid linker issues because in a downstream package, the IDL dependencies are passed to the linker
# before this lib and the order is ignored only for shared objects
ament_auto_add_library(scenario_api_autoware SHARED
  src/${PROJECT_NAME}.cpp
)

ament_auto_package()
