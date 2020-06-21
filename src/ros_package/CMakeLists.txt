cmake_minimum_required(VERSION 2.8.3)
project(ros_package)

find_package(catkin REQUIRED COMPONENTS rospy pcl_conversions pcl_ros roscpp sensor_msgs std_msgs message_generation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


## Declare ROS messages and services
add_message_files(FILES Num.msg)
add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()


include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(my-bin
	    src/InteractiveICP.cpp
	    src/InteractiveICP.hpp)

##add_executable (PBVS PBVS.cpp)
##target_link_libraries (PBVS ${PCL_LIBRARIES})

add_executable (InteractiveICP src/InteractiveICP.cpp)
target_link_libraries (InteractiveICP PRIVATE my-bin ${catkin_LIBRARIES})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker ros_package_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener ros_package_generate_messages_cpp)

