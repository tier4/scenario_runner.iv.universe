cmake_minimum_required(VERSION 2.8.3)
project(entity_plugins)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  scenario_actions
  scenario_entities
  scenario_logger
  scenario_utility
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS pluginlib
                 scenario_actions
                 scenario_entities
                 scenario_logger
                 scenario_utility
)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME} SHARED
  # src/bicycle_entity.cpp
  # src/ego_entity.cpp
  # src/motobike_entity.cpp
  # src/pedestrian_entity.cpp
  # src/vehicle_entity.cpp
  src/targets.cpp
  )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
