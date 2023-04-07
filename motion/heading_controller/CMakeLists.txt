cmake_minimum_required(VERSION 3.0.2)
project(heading_controller)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  rospy
  vortex_msgs
)

generate_dynamic_reconfigure_options(
  cfg/headingController.cfg
)


catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
)