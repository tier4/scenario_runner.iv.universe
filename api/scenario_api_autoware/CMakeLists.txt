cmake_minimum_required(VERSION 2.8.3)
project(scenario_api_autoware)

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
  scenario_api_utils
  scenario_logger
  sensor_msgs
  std_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scenario_api_autoware
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
    scenario_api_utils
    scenario_logger
    sensor_msgs
    std_msgs
    tf2_ros
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)

add_library(scenario_api_autoware SHARED
src/scenario_api_autoware.cpp
)
add_dependencies(scenario_api_autoware ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scenario_api_autoware
${catkin_LIBRARIES}
${YAML_CPP_LIBRARIES}
)

install(TARGETS scenario_api_autoware
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
