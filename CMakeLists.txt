cmake_minimum_required(VERSION 2.6 FATAL_ERROR)


project(lmr_project)
find_package(PCL 1.5 REQUIRED)
find_package(VTK REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(my-bin
	    #hello.cpp
	    #hello.hpp
       	    matrix_transform.cpp
            matrix_transform.hpp
       	    ICP.cpp
	    ICP.hpp
	    main.cpp)

#add_executable(hello hello.cpp)
#target_link_libraries(hello PRIVATE my-bin)

add_executable(transform transform.cpp)
target_link_libraries(transform ${PCL_LIBRARIES})

add_executable(pcd_write_test pcd_write.cpp)
target_link_libraries(pcd_write_test ${PCL_LIBRARIES})

add_executable (concatenate_clouds concatenate_clouds.cpp)
target_link_libraries (concatenate_clouds ${PCL_LIBRARIES})

add_executable (matrix_transform matrix_transform.cpp)
target_link_libraries (matrix_transform PRIVATE my-bin ${PCL_LIBRARIES})

add_executable (ICP ICP.cpp)
target_link_libraries (ICP PRIVATE my-bin ${PCL_LIBRARIES})

add_executable (main main.cpp)
target_link_libraries (main PRIVATE my-bin ${PCL_LIBRARIES})
