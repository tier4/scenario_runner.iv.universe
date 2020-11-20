cmake_minimum_required(VERSION 3.5)
project(condition_plugins)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
   src/accelaration_condition.cpp
   src/always_false_condition.cpp
   src/always_true_condition.cpp
   src/collision_by_entity_condition.cpp
   src/reach_position_condition.cpp
   src/relative_distance_condition.cpp
   src/signal_condition.cpp
   src/simulation_time_condition.cpp
   src/speed_condition.cpp
)

pluginlib_export_plugin_description_file(scenario_conditions plugins.xml)

ament_auto_package()

