cmake_minimum_required(VERSION 2.8.3)
project(wolvmarine_drivers)

find_package(catkin REQUIRED COMPONENTS
	advanced_navigation_driver
	laser_filters
	rospy
	rosserial_python
	rostest
	sensor_msgs
	std_msgs
	velodyne_driver
	velodyne_pointcloud
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
#	geometry_msgs
#	nav_msgs
#	sensor_msgs
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest(tests/tests.py)
endif()
