
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_robcog)

find_package(catkin REQUIRED
  world_control_msgs
  unreal_vis_msgs
  rosprolog
  roscpp
  roslib
  rospack
  geometry_msgs)


generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  world_control_msgs
  unreal_vis_msgs
)

catkin_package(
  DEPENDS
   roscpp
   roslib
   rospack
   world_control_msgs
   unreal_vis_msgs
)


find_package(PkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)
message(STATUS "SWIPL found.")

include_directories(include ${SWIPL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(prolog_service_calls SHARED src/prolog_service_calls.cpp)
target_link_libraries(prolog_service_calls ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})

#add_library(display_types_service_calls SHARED src/display_types_service_calls.cpp)
#target_link_libraries(display_types_service_calls ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})

install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)

FILE(GLOB sh_files "scripts/*")
install(PROGRAMS ${sh_files}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/)
