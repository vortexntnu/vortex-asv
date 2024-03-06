cmake_minimum_required(VERSION 3.8)
project(waypoint_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#list(APPEND CMAKE_PREFIX_PATH "/ros2_ws/install")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


#find vortex_msgs package 
find_package(vortex_msgs REQUIRED)
#if (NOT vortex_msgs_FOUND)
 # message(FATAL_ERROR "vortex_msgs ppacket not found")
#endif()

ament_python_install_package(scripts)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/waypoint_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
find_package(ament_cmake_pytest REQUIRED)
set(_pytest_tests
  test/test_waypoint_manager.py
)
foreach(_test_path ${_pytest_tests})
  get_filename_component(_test_name ${_test_path} NAME_WE)
  ament_add_pytest_test(${_test_name} ${_test_path}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
    TIMEOUT 60
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
endforeach()
endif()

ament_package()
