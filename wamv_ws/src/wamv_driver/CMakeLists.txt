cmake_minimum_required(VERSION 2.8.3)
project(wamv_driver)

find_package(catkin REQUIRED COMPONENTS
	advanced_navigation_driver
#	ladybug3_ros_pkg
	geometry_msgs
	nav_msgs
	rospy
	rosserial_python
	sensor_msgs
	velodyne_driver
	velodyne_pointcloud
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
	geometry_msgs
	nav_msgs
	rospy
	sensor_msgs
)
