cmake_minimum_required(VERSION 3.12)
project(SFC)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(TARGET ${CMAKE_PROJECT_NAME})

add_executable(SFC main.cpp robot.cpp robot.cpp)

target_link_libraries(${TARGET})
