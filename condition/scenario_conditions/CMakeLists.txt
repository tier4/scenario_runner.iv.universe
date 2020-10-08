cmake_minimum_required(VERSION 2.8.3)
project(scenario_conditions)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  scenario_api
  scenario_intersection
  scenario_utility
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scenario_conditions
  CATKIN_DEPENDS
    pluginlib
    roscpp
    scenario_api
    scenario_intersection
    scenario_utility
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_library(scenario_conditions SHARED
  src/condition_manager.cpp src/condition_visualizer.cpp
)
add_dependencies(scenario_conditions ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scenario_conditions
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

install(TARGETS scenario_conditions
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
