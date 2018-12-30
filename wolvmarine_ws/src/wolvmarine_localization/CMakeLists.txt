cmake_minimum_required(VERSION 2.8.3)
project(wolvmarine_localization)

find_package(catkin REQUIRED
	geometry_msgs
	hector_compressed_map_transport
	hector_geotiff
	hector_mapping
	hector_trajectory_server
	laser_scan_matcher
	message_filters
	nav_msgs
	orokos_kdl
	robot_localization
	rospy
	sensor_msgs
	std_msgs
	tf2_geometry_msgs
	tf2_ros
)

catkin_python_setup()

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
