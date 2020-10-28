cmake_minimum_required(VERSION 2.8.3)
project(scenario_utility)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  scenario_logger
  scenario_logger_msgs
)

find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scenario_utility
  CATKIN_DEPENDS roscpp geometry_msgs tf2_ros tf2_geometry_msgs scenario_logger scenario_logger_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(scenario_utility SHARED
  src/converter.cpp
  src/indentation.cpp
  src/misc.cpp
  src/parse.cpp
)
add_dependencies(scenario_utility ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scenario_utility
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

install(TARGETS scenario_utility
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
