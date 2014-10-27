cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(remedi2)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)

add_executable (remedi2 run_system.cpp)
target_link_libraries (remedi2 ${OpenCV_LIBS} ${PCL_LIBRARIES})
