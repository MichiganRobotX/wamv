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

add_executable(tf_publisher src/tf_publisher.cpp)
target_link_libraries(tf_publisher ${catkin_LIBRARIES})

add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry ${catkin_LIBRARIES})

# Task 1 : Straight Line task
#add_executable(straightLine nodes/straightLine.cpp)
#target_link_libraries(straightLine ${catkin_LIBRARIES})
