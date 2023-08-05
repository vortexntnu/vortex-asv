cmake_minimum_required(VERSION 3.0.2)
project(thruster_interface)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  vortex_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}_node
  src/thruster_interface.cpp
  src/thruster_interface_ros.cpp
  src/thruster_interface_node.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)