cmake_minimum_required(VERSION 3.0.2)
project(pid_controller)

find_package(catkin REQUIRED COMPONENTS
 dynamic_reconfigure
 geometry_msgs
 rospy
 vortex_msgs
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

 catkin_install_python(PROGRAMS
   scripts/pid_controller.py
   DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}
)

