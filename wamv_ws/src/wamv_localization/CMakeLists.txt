cmake_minimum_required(VERSION 2.8.3)
project(wamv_localization)

find_package(catkin REQUIRED
	roscpp
	rospy
	gps_common
	sensor_msgs
	nav_msgs
	std_msgs
	geometry_msgs
	tf2
	tf
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(wamv_tf_publisher src/wamv_tf_publisher.cpp)
target_link_libraries(wamv_tf_publisher ${catkin_LIBRARIES})

# Task 1 : Straight Line task
#add_executable(straightLine nodes/straightLine.cpp)
#target_link_libraries(straightLine ${catkin_LIBRARIES})
