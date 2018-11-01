cmake_minimum_required(VERSION 2.8.3)
project(wamv_driver)

find_package(catkin REQUIRED COMPONENTS
	advanced_navigation_driver
#	ladybug3_ros_pkg
	rosserial_python
	velodyne_driver
	velodyne_pointcloud
	wamv_localization
	wamv_tasks
	gps_common
	tf
)

catkin_package()
