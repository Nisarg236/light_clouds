cmake_minimum_required(VERSION 3.10)
project(light_clouds)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(include)

enable_testing()

set(SOURCES
    src/main.cpp
)

add_executable(light_clouds ${SOURCES})
