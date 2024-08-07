cmake_minimum_required(VERSION 3.8)
project(navigation_task)

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

# Find dependencies
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
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)


include_directories(include)

add_executable(navigation_task_node
  src/navigation_task_node.cpp
  src/navigation_task_ros.cpp
)

target_link_libraries(navigation_task_node
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  pcl_common
)

ament_target_dependencies(navigation_task_node 
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  vortex_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  njord_task_base
  pcl_conversions
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  navigation_task_node
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
