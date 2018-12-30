cmake_minimum_required(VERSION 2.8.3)
project(wolvmarine_controls)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	joy
	nav_msgs
	pid
	rospy
	sensor_msgs
	std_msgs
	wolvmarine_msgs
)

catkin_python_setup()

catkin_package()
