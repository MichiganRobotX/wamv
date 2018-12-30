cmake_minimum_required(VERSION 2.8.3)
project(wolvmarine_fixer)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	message_filters
	orocos_kdl
	rospy
	sensor_msgs
	tf2_ros
)

catkin_python_setup()

catkin_package()
