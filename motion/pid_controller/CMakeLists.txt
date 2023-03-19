cmake_minimum_required(VERSION 3.0.2)
project(pid_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
 dynamic_reconfigure
 geometry_msgs
 rospy
 vortex_msgs
)


catkin_python_setup()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 catkin_install_python(PROGRAMS
   scripts/pid_controller.py
   DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}
)

