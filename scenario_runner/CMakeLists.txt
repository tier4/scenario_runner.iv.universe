cmake_minimum_required(VERSION 2.8.3)
project(scenario_runner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  scenario_api
  scenario_entities
  scenario_expression
  scenario_intersection
  scenario_logger
  scenario_logger_msgs
  scenario_runner_msgs
  scenario_sequence
  scenario_utility
)

find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
                 scenario_api
                 scenario_entities
                 scenario_expression
                 scenario_intersection
                 scenario_logger
                 scenario_logger_msgs
                 scenario_sequence
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_library(scenario_runner SHARED
  src/scenario_terminator.cpp
  src/scenario_runner.cpp)
add_dependencies(scenario_runner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(scenario_runner
  ${catkin_LIBRARIES}
  Boost::filesystem
  glog
)

add_executable(scenario_runner_node
  src/scenario_runner_node.cpp)
add_dependencies(scenario_runner_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(scenario_runner_node
  ${catkin_LIBRARIES}
  scenario_runner
  ${YAML_CPP_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

foreach(dir launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(TARGETS
  scenario_runner scenario_runner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
