cmake_minimum_required(VERSION 3.10)
project(light_clouds)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(include)

enable_testing()

add_subdirectory(external/googletest)

set(SOURCES
    src/example.cpp
)

add_executable(light_clouds ${SOURCES})

add_executable(light_clouds_tests
    tests/point_cloud_tests.cpp
)

target_link_libraries(light_clouds_tests
    gtest
    gtest_main
    pthread
)

add_test(NAME PointCloudTests COMMAND light_clouds_tests)
