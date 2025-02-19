cmake_minimum_required(VERSION 3.8)
project(joystick_interface_asv)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/${PROJECT_NAME}_node.py
  ${PROJECT_NAME}/joystick_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
