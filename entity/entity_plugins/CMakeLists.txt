cmake_minimum_required(VERSION 3.5)
project(entity_plugins)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
# We require all dependencies but by default ament_auto_find_build_dependencies does not require packages
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/bicycle_entity.cpp
  src/ego_entity.cpp
  src/motobike_entity.cpp
  src/pedestrian_entity.cpp
  src/vehicle_entity.cpp
)

install(FILES plugins.xml
  DESTINATION share
)

ament_auto_package()
