cmake_minimum_required(VERSION 2.6)
project(graph)

add_executable(graph main.cpp)

install(TARGETS graph RUNTIME DESTINATION bin)
