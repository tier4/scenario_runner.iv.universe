cmake_minimum_required(VERSION 2.8.3)
project(scenario_api_autoware)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  geometry_msgs
  std_msgs
  autoware_planning_msgs
  autoware_system_msgs
  autoware_perception_msgs
  autoware_vehicle_msgs
  dummy_perception_publisher
  roscpp
  tf2_ros
  pcl_ros
  pcl_conversions
  lanelet2_extension
  scenario_api_utils
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES scenario_api_autoware
 CATKIN_DEPENDS scenario_api_utils sensor_msgs geometry_msgs std_msgs autoware_planning_msgs autoware_system_msgs autoware_perception_msgs autoware_vehicle_msgs dummy_perception_publisher roscpp tf2_ros pcl_ros pcl_conversions lanelet2_extension
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
