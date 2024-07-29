cmake_minimum_required(VERSION 3.8)
project(docking_task)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(njord_task_base REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(docking_task_node
  src/docking_task_node.cpp
  src/docking_task_ros.cpp)

target_link_libraries(docking_task_node
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)

ament_target_dependencies(docking_task_node 
  rclcpp
  njord_task_base
  geometry_msgs
  nav_msgs
  sensor_msgs
  vortex_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(
  DIRECTORY include/
  DESTINATION include

)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}/
  )

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
