cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
add_definitions(-std=c++17)

   
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
   
project(playback)
   
include(C:/Users/andre/Documents/vcpkg/vcpkg/scripts/buildsystems/vcpkg.cmake)
find_package(PCL 1.2 REQUIRED)
   
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
   
   
add_executable (project_1 src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)

target_link_libraries (project_1 PRIVATE ${PCL_LIBRARIES})



