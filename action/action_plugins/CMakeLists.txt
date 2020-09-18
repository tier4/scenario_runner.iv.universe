cmake_minimum_required(VERSION 2.8.3)
project(action_plugins)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  scenario_actions
  scenario_intersection
  scenario_logger
  scenario_utility
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS pluginlib
                 scenario_actions
                 scenario_intersection
                 scenario_logger
                 scenario_utility
)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME} SHARED
  # src/acceleration_action.cpp
  # src/change_signal_action.cpp
  # src/follow_route_action.cpp
  # src/lane_change_action.cpp
  # src/speed_action.cpp
  src/targets.cpp
  )

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

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

