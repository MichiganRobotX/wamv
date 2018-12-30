cmake_minimum_required(VERSION 2.8.3)
project(wolvmarine_tasks)

find_package(catkin REQUIRED COMPONENTS
	rospy
	sensor_msgs
	std_msgs
	wolvmarine_msgs
)

catkin_python_setup()

# add catkin packages
catkin_package()

# specify location of header files
include_directories(include ${catkin_INCLUDE_DIRS})
