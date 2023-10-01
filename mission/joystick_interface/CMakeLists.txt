cmake_minimum_required(VERSION 3.8)
project(joystick_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
#find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

#ament_python_install_package("joystick_interface")
ament_python_install_package(${PROJECT_NAME})

#add_executable(joystick_interface_node scripts/joystick_interface.py)
#ament_target_dependencies(joystick_interface_node rclpy sensor_msgs geometry_msgs)

install(PROGRAMS
  joystick_interface/joystick_interface.py
  #DESTINATION lib/joystick_interface
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests
    joystick_interface/test/test_joystick_interface.py
    # Add other test files here
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
