cmake_minimum_required(VERSION 2.8.3)
project(wolvmarine_navigation)

find_package(catkin REQUIRED COMPONENTS
	rospy
	std_msgs
	actionlib
	move_base_msgs
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
	rospy
	std_msgs
	actionlib
	move_base_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})
