cmake_minimum_required(VERSION 3.8)
project(pathplanning)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-pthread")

set(SOURCE_FILES main.cpp coordinate.cpp coordinate.h node.cpp node.h pathplanner.cpp pathplanner.h obstacle.cpp obstacle.h)
add_executable(pathplanning ${SOURCE_FILES})