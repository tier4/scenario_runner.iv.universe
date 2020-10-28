cmake_minimum_required(VERSION 2.8.3)

project(scenario_expression)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED
  COMPONENTS
    pluginlib
    roscpp
    scenario_api
    scenario_conditions
    scenario_entities
    scenario_intersection
    scenario_utility
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    pluginlib
    roscpp
    scenario_api
    scenario_conditions
    scenario_entities
    scenario_intersection
    scenario_utility
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  )

add_library(${PROJECT_NAME} SHARED
  src/expression.cpp
  )

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
