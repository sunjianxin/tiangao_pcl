cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(fitting_3d)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (fitting_3d fitting_3d.cpp)
target_link_libraries (fitting_3d ${PCL_LIBRARIES})
add_executable (createPC createPC.cpp)
target_link_libraries (createPC ${PCL_LIBRARIES})
