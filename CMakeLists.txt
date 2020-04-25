cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(lmr_project)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(say-hello
	    hello.cpp
	    hello.hpp)

add_executable(hello hello.cpp)
target_link_libraries(hello PRIVATE say-hello)

add_executable(pcd_write_test pcd_write.cpp)
target_link_libraries(pcd_write_test ${PCL_LIBRARIES})

add_executable (concatenate_clouds concatenate_clouds.cpp)
target_link_libraries (concatenate_clouds ${PCL_LIBRARIES})

add_executable (matrix_transform matrix_transform.cpp)
target_link_libraries (matrix_transform ${PCL_LIBRARIES})

