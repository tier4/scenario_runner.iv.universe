cmake_minimum_required(VERSION 3.5)
project(scenario_api)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
  # TODO remove before merging
  add_compile_options(-Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
# filters kdtree registration io
find_package(PCL REQUIRED COMPONENTS common filters kdtree registration io)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} STATIC
  src/scenario_api_calc_dist_utils.cpp
  src/scenario_api_coordinate_manager.cpp
  src/scenario_api_core.cpp
)
target_compile_definitions(${PROJECT_NAME} PUBLIC ${PCL_DEFINITIONS})
target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}  ${PCL_LIBRARIES})
target_link_directories(${PROJECT_NAME} PUBLIC ${PCL_LIBRARY_DIRS})

ament_auto_add_executable(${PROJECT_NAME}_node
  src/scenario_api_node.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
    INSTALL_TO_SHARE launch rviz
)