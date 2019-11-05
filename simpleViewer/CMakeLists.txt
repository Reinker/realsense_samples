cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(Viewer)

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

add_executable (viewer Viewer.cpp)

target_link_libraries (viewer ${realsense2_LIBRARY})
target_link_libraries (viewer ${OpenCV_LIBS})
