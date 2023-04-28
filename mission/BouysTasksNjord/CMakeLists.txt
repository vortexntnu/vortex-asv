cmake_minimum_required(VERSION 3.0.2)
project(njord_fsm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  vortex_msgs
  # smach
  # smach_ros
)


catkin_python_setup()

catkin_package(
  #CATKIN_DEPENDS rospy smach smach_ros  # Add smach and smach_ros as catkin dependencies
)


include_directories(
  #${catkin_INCLUDE_DIRS}
)
