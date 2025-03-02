cmake_minimum_required(VERSION 3.10)
project(serialib)

set(CMAKE_CXX_STANDARD 17)

add_library(serialib serialib.cpp)

target_include_directories(serialib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})