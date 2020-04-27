cmake_minimum_required(VERSION 2.6 FATAL_ERROR)


project(lmr_project)
find_package(PCL 1.5 REQUIRED)
find_package(VTK REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(my-bin
	    InteractiveICP.cpp
	    InteractiveICP.hpp
       	    ICP.cpp
	    ICP.hpp
	    main.cpp
	    Test.cpp)

add_executable (ICP ICP.cpp)
target_link_libraries (ICP PRIVATE my-bin ${PCL_LIBRARIES})

add_executable (main main.cpp)
target_link_libraries (main PRIVATE my-bin ${PCL_LIBRARIES})

add_executable (InteractiveICP InteractiveICP.cpp)
target_link_libraries (InteractiveICP PRIVATE my-bin ${PCL_LIBRARIES})

add_executable (Test Test.cpp)
target_link_libraries (Test PRIVATE my-bin ${PCL_LIBRARIES})
