cmake_minimum_required(VERSION 2.8.3)
project(condition_plugins)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  scenario_conditions
  scenario_utility
  scenario_logger
  scenario_logger_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    pluginlib
    scenario_conditions
    scenario_logger
    scenario_logger_msgs
    scenario_utility
    )

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME} SHARED
  # src/accelaration_condition.cpp
  # src/always_false_condition.cpp
  # src/always_true_condition.cpp
  # src/collision_by_entity_condition.cpp
  # src/reach_position_condition.cpp
  # src/relative_distance_condition.cpp
  # src/signal_condition.cpp
  # src/simulation_time_condition.cpp
  # src/speed_condition.cpp
  src/targets.cpp
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

install(
  FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

