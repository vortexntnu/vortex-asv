cmake_minimum_required(VERSION 2.8.3)
project(thruster_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  vortex_msgs
  std_msgs
  message_generation
  roslint
)

roslint_python()

catkin_package(
  CATKIN_DEPENDS
  rospy
  vortex_msgs
  std_msgs
  message_runtime
)
