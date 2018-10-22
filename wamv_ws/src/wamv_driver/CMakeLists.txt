cmake_minimum_required(VERSION 2.8.3)
project(wamv_driver)

find_package(catkin REQUIRED COMPONENTS
	velodyne_driver
	velodyne_pointcloud
	roscpp
	rospy
	gps_common
	sensor_msgs
	nav_msgs
	std_msgs
	geometry_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(wamv_odometry src/wamv_odometry.cpp)

target_link_libraries(wamv_odometry ${catkin_LIBRARIES})

#include_directories(include ${catkin_INCLUDE_DIRS})
