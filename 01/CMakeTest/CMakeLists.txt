cmake_minimum_required(VERSION 2.8)
project(my_hello_project)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

add_library(cuboid cuboid.cpp)

add_executable(cuboidExe main.cpp)

target_link_libraries(cuboidExe cuboid)
