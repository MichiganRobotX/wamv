cmake_minimum_required(VERSION 2.8.3)
project(wamv_driver)

find_package(catkin REQUIRED COMPONENTS
	advanced_navigation_driver
#	ladybug3_ros_pkg
	geometry_msgs
	nav_msgs
	rosserial_python
	sensor_msgs
	velodyne_driver
	velodyne_pointcloud
)

catkin_package(CATKIN_DEPENDS
	geometry_msgs
	nav_msgs
	sensor_msgs
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest(tests/tests.py)
endif()
