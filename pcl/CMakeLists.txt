cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Viz)

find_package(realsense2 REQUIRED)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (viz Viz.cpp)

target_link_libraries (viz ${realsense2_LIBRARY})
target_link_libraries (viz ${PCL_LIBRARIES})

