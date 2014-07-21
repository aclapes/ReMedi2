cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(remedi2)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)

add_executable (remedi2 MACOSX_BUNDLE main.cpp ColorFrame.h ColorFrame.cpp conversion.h conversion.cpp DepthFrame.h DepthFrame.cpp Frame.h Frame.cpp InteractiveRegisterer.h InteractiveRegisterer.cpp KinectReader.h KinectReader.cpp Remedi.h Remedi.cpp Sequence.h Sequence.cpp cvxtended.h cvxtended.cpp statistics.h statistics.cpp ColorDepthFrame.h ColorDepthFrame.cpp)
target_link_libraries (remedi2 ${OpenCV_LIBS} ${PCL_LIBRARIES})
