cmake_minimum_required(VERSION 3.0.2)
project(master)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  comm
  goalkeeper
  attacker
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

add_library(master src/init.cpp)

target_link_libraries(master ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES})

add_executable(decision_maker src/decision_maker.cpp )
add_dependencies(decision_maker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(decision_maker ${catkin_LIBRARIES} goalkeeper_lib attacker_lib)