cmake_minimum_required(VERSION 3.8)
project(map_manager)

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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)

include_directories(include)

add_executable(map_manager_node src/map_manager_ros.cpp
  src/map_manager_node.cpp
)

target_link_libraries( ${PROJECT_NAME}_node
    pcl_common)

ament_target_dependencies(
  map_manager_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
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
