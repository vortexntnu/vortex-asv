cmake_minimum_required(VERSION 3.0.2)
project(bms)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  rospy
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Declare catkin package
catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES freya_bms
  CATKIN_DEPENDS roscpp sensor_msgs
  DEPENDS Boost
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(freya_${PROJECT_NAME}_node src/freya_bms_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(freya_${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)