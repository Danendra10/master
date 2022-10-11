cmake_minimum_required(VERSION 3.0.2)
project(master)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  roslib
  dynamic_reconfigure
  comm
  motion
  multirole_func
  utils
  goalkeeper
  assist
  defender
  attacker
)

find_package(yaml-cpp REQUIRED)

add_message_files(
  FILES
  Vision.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES master
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_library(master_lib src/init.cpp)

target_link_libraries(master_lib ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} multirole_func_lib motion_lib utils_lib goalkeeper_lib defender_lib assist_lib attacker_lib)