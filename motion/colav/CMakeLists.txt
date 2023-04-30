cmake_minimum_required(VERSION 3.0.2)
project(colav)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  nav_msgs
  reference_model
  rospy
  tf
  vortex_msgs
  std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

