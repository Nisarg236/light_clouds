cmake_minimum_required(VERSION 3.10)
project(light_clouds)

# C++ Standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(include)

# Enable testing
enable_testing()

# Add Google Test
add_subdirectory(external/googletest)

# Sources for the main application
set(SOURCES
    src/main.cpp
)

# Main executable
add_executable(light_clouds ${SOURCES})

# Tests
add_executable(light_clouds_tests
    tests/point_cloud_tests.cpp
)

target_link_libraries(light_clouds_tests
    gtest
    gtest_main
    pthread
)

# Register the tests
add_test(NAME PointCloudTests COMMAND light_clouds_tests)
