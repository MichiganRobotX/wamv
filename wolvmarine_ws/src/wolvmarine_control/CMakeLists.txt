cmake_minimum_required(VERSION 2.8.3)
project(wamv_control)

find_package(catkin REQUIRED COMPONENTS
	pid
	rospy
	sensor_msgs
	std_msgs
	wamv_msgs
)

catkin_python_setup()

catkin_package()
