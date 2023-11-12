cmake_minimum_required(VERSION 3.10)

find_package(GTest REQUIRED)

set(TEST_EXECUTABLE thruster_allocator_tester)

set(TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/test_allocator_utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_pseudoinverse_allocator.cpp
    ${CMAKE_SOURCE_DIR}/src/pseudoinverse_allocator.cpp
)

# Create test executable
add_executable(${TEST_EXECUTABLE} ${TEST_SOURCES})

# Link libraries
target_link_libraries(${TEST_EXECUTABLE} ${googletest_LIBRARIES}
                      GTest::GTest GTest::Main)

ament_target_dependencies(${TEST_EXECUTABLE}
    vortex_msgs
    Eigen3
)

target_include_directories(
    ${TEST_EXECUTABLE} PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

gtest_discover_tests(${TEST_EXECUTABLE})
