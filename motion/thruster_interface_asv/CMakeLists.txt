cmake_minimum_required(VERSION 3.8)
project(thruster_interface_asv)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(fmt REQUIRED)

include_directories(include)

add_executable(${PROJECT_NAME}_node
  src/thruster_interface_asv_node.cpp
  src/thruster_interface_asv_ros.cpp
  src/thruster_interface_asv_driver.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  vortex_msgs
)

target_link_libraries(${PROJECT_NAME}_node
  fmt::fmt
  spdlog::spdlog
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()