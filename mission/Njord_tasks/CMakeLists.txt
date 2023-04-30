cmake_minimum_required(VERSION 3.0.2)
project(Njord_tasks)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rospy
  std_msgs
  std_srvs
  vortex_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)