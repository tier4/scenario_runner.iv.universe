cmake_minimum_required(VERSION 2.8.3)
project(scenario_api)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  autoware_planning_msgs
  autoware_system_msgs
  autoware_vehicle_msgs
  dummy_perception_publisher
  geometry_msgs
  lanelet2_extension
  pcl_conversions
  pcl_ros
  roscpp
  scenario_api_autoware
  scenario_api_simulator
  scenario_api_utils
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    autoware_perception_msgs
    autoware_planning_msgs
    autoware_system_msgs
    autoware_vehicle_msgs
    dummy_perception_publisher
    geometry_msgs
    lanelet2_extension
    pcl_conversions
    pcl_ros
    roscpp
    scenario_api_autoware
    scenario_api_simulator
    scenario_api_utils
    sensor_msgs
    std_msgs
    tf2_geometry_msgs
    tf2_ros
 )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME} SHARED
  src/scenario_api_calc_dist_utils.cpp
  src/scenario_api_coordinate_manager.cpp
  src/scenario_api_core.cpp
  )
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )

add_executable(${PROJECT_NAME}_node
  src/scenario_api_node.cpp
  )
add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  )

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(
  DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
