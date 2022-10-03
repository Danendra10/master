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
  goalkeeper
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

add_executable(decision_maker src/decision_maker.cpp )
add_dependencies(decision_maker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(decision_maker ${catkin_LIBRARIES} goalkeeper_lib attacker_lib)