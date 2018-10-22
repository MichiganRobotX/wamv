cmake_minimum_required(VERSION 2.8.3)
project(wamv_bringup)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	advanced_navigation_driver
	velodyne_pointcloud
	geometry_msgs
	sensor_msgs
	std_msgs
	nav_msgs
	)

catkin_package()

#include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(wamv_tf_publisher src/wamv_tf_publisher.cpp)
#target_link_libraries(wamv_tf_publisher ${catkin_LIBRARIES})
