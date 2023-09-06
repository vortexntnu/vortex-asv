cmake_minimum_required(VERSION 3.0.2)
project(lqr_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)