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

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   scenario_api
#   scenario_logger
#   scenario_logger_msgs
#   scenario_utility
#   )

# find_package(yaml-cpp REQUIRED)

# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS roscpp
#                  scenario_api
#                  scenario_logger
#                  scenario_logger_msgs
#   )

### Target executable
set(SCENARIO_INTERSECTION_SRC
  src/arrow.cpp
  # src/color.cpp
  # src/intersection.cpp
  # src/intersection_manager.cpp
)

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${YAML_CPP_INCLUDE_DIR}
#   )

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${SCENARIO_INTERSECTION_SRC}
)

# add_dependencies(${PROJECT_NAME}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
#   )

# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${YAML_CPP_LIBRARIES}
#   )

# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# install(
#   TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

ament_auto_package()
