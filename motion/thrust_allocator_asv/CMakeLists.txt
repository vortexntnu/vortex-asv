cmake_minimum_required(VERSION 3.8)
project(thrust_allocator_asv)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(include)

add_executable(${PROJECT_NAME}_node
  src/thrust_allocator_asv_node.cpp
  src/allocator_ros.cpp
  src/pseudoinverse_allocator.cpp
  )

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  geometry_msgs
  std_msgs
  Eigen3
  )

target_link_libraries(${PROJECT_NAME}_node rclcpp_lifecycle::rclcpp_lifecycle)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
