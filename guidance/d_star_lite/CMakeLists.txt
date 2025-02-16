cmake_minimum_required(VERSION 3.8)
project(d_star_lite)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  d_star_lite/ros_d_star_lite_node.py
  d_star_lite/d_star_lite.py
  d_star_lite/d_star_lite_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
