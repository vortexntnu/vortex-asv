cmake_minimum_required(VERSION 3.8)
project(hybridpath_controller)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  hybridpath_controller/hybridpath_controller_node.py
  hybridpath_controller/adaptive_backstep.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()