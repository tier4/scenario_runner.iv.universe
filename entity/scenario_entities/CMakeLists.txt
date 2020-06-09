cmake_minimum_required(VERSION 2.8.3)
project(scenario_entities)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  scenario_actions
  scenario_api
  scenario_intersection
  scenario_logger
  scenario_utility
)

find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scenario_entities
  CATKIN_DEPENDS roscpp
                 scenario_actions
                 scenario_api
                 scenario_logger
                 scenario_utility
                 pluginlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(scenario_entities SHARED
  src/entitiy_manager.cpp
  src/entity_base.cpp
)
add_dependencies(scenario_entities ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scenario_entities
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

install(TARGETS scenario_entities
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
