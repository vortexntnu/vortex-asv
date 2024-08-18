cmake_minimum_required(VERSION 3.8)
project(landmark_server)

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
find_package(std_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

# Create an executable
add_executable(landmark_server_node 
    src/landmark_server.cpp
    src/landmark_server_main.cpp
    src/grid_manager.cpp)

target_link_libraries( ${PROJECT_NAME}_node
    pcl_common
    ${PCL_LIBRARIES}
    Eigen3::Eigen)

ament_target_dependencies(landmark_server_node
    rclcpp
    std_msgs
    vortex_msgs
    sensor_msgs
    pcl_conversions
    nav_msgs
    rclcpp_action
    rclcpp_components
    tf2
    tf2_geometry_msgs
  )

    install(TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
  )
  
  install(DIRECTORY
    launch
    params
    DESTINATION share/${PROJECT_NAME}/
  )




ament_package()
