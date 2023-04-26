cmake_minimum_required(VERSION 3.0.2)
project(Njord_fsm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  vortex_msgs
)


catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
