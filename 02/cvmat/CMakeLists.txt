project(cvmat)
cmake_minimum_required(VERSION 2.8)

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

find_package(OpenCV REQUIRED)

add_executable(cvmat cvmat.cpp)

target_link_libraries(cvmat ${OpenCV_LIBS})

