cmake_minimum_required(VERSION 2.8.3)
project(scenario_sequence)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  scenario_actions
  scenario_api
  scenario_entities
  scenario_expression
  scenario_intersection
  scenario_logger
  scenario_logger_msgs
  scenario_utility
  )

find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
                 scenario_actions
                 scenario_api
                 scenario_entities
                 scenario_expression
                 scenario_intersection
                 scenario_logger
                 scenario_logger_msgs
                 scenario_utility
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  )

add_library(${PROJECT_NAME} SHARED
  src/event.cpp
  src/event_manager.cpp
  src/sequence.cpp
  src/sequence_manager.cpp
  )

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

