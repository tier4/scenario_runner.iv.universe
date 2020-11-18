cmake_minimum_required(VERSION 3.5)
project(scenario_entities)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include
)

ament_auto_add_library(scenario_entities SHARED
  src/entitiy_manager.cpp
  src/entity_base.cpp
)

ament_auto_package()