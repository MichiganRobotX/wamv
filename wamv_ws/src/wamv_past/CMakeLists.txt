cmake_minimum_required(VERSION 2.8.3)
project(wamv_past)

find_package(catkin REQUIRED COMPONENTS
	rospy
	geometry_msgs
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
	rospy
	geometry_msgs
)
