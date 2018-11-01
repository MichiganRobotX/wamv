cmake_minimum_required(VERSION 2.8.3)
project(wamv_transform)

find_package(catkin REQUIRED COMPONENTS
	tf2_ros
)

catkin_package(CATKIN_DEPENDS
	tf2_ros
)

include_directories(include ${catkin_INCLUDE_DIRS})
