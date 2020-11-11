cmake_minimum_required(VERSION 3.5)
project(scenario_api_autoware)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(autoware_system_msgs REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(autoware_debug_msgs REQUIRED)
find_package(dummy_perception_publisher REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(lanelet2_extension REQUIRED)
find_package(scenario_api_utils REQUIRED)
find_package(vehicle_info_util REQUIRED)

### Includes
include_directories(include)

add_library(scenario_api_autoware
  src/scenario_api_autoware.cpp
)

set(SCENARIO_API_AUTOWARE_DEPENDENCIES
  scenario_api_utils
  sensor_msgs
  geometry_msgs
  std_msgs
  autoware_planning_msgs
  autoware_system_msgs
  autoware_perception_msgs
  autoware_vehicle_msgs
  autoware_debug_msgs
  dummy_perception_publisher
  rclcpp
  tf2_ros
  PCL
  pcl_conversions
  lanelet2_extension
  vehicle_info_util
)

target_include_directories(scenario_api_autoware
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scenario_api_autoware PUBLIC ${SCENARIO_API_AUTOWARE_DEPENDENCIES})
target_compile_definitions(scenario_api_autoware PUBLIC ${PCL_DEFINITIONS})
target_include_directories(scenario_api_autoware PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(scenario_api_autoware PUBLIC ${PCL_LIBRARIES})
target_link_directories(scenario_api_autoware PUBLIC ${PCL_LIBRARY_DIRS})

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS scenario_api_autoware
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
