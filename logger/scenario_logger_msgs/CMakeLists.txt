cmake_minimum_required(VERSION 2.8.3)
project(scenario_logger_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  Log.msg
  Level.msg
  MetaData.msg
  LoggedData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS message_runtime std_msgs
)

foreach(dir msg)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)