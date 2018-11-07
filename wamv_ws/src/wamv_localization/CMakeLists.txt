cmake_minimum_required(VERSION 2.8.3)
project(wamv_localization)

find_package(catkin REQUIRED
	gmapping
	gps_common
	move_base
	nav_msgs
	robot_localization
	robot_state_publisher
	roscpp
	sensor_msgs
	tf
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
