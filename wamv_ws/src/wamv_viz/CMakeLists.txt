cmake_minimum_required(VERSION 2.8.3)
project(wamv_viz)

find_package(catkin REQUIRED COMPONENTS
	rviz
	wamv_localization
)

# add catkin packages
catkin_package()

# specify location of header files
include_directories(include ${catkin_INCLUDE_DIRS})
