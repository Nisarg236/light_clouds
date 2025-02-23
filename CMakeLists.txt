cmake_minimum_required(VERSION 3.10)
project(light_clouds VERSION 1.0)


add_library(light_clouds_lib INTERFACE)  # No source files needed

# Public headers for users
target_include_directories(light_clouds_lib
  INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include  # Expose include/light_clouds
)

enable_testing()
add_subdirectory(external/googletest)

add_executable(light_clouds_tests
  tests/point_cloud_tests.cpp
)

target_link_libraries(light_clouds_tests
  PRIVATE
    light_clouds_lib  # Get headers from here
    gtest
    gtest_main
    pthread
)

add_test(NAME PointCloudTests COMMAND light_clouds_tests)

# Example Executable
add_executable(light_clouds_example
  examples/example.cpp
)

target_link_libraries(light_clouds_example
  PRIVATE light_clouds_lib
)