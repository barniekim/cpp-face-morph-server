cmake_minimum_required(VERSION 2.8)

project(face-morph)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(face-morph FaceMorph.cpp)
target_link_libraries(face-morph ${OpenCV_LIBS})
