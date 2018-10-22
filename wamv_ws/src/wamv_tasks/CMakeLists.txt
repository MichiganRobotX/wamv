cmake_minimum_required(VERSION 2.8.3)
project(wamv_tasks)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	wamv_msgs
	sensor_msgs
	std_msgs
)

# add catkin packages
catkin_package()

# specify location of header files
include_directories(include ${catkin_INCLUDE_DIRS})

# add node to publish signal for heartbeat
add_executable(heartbeat nodes/heartbeat.cpp)

# build node to publish signal for heartbeat
target_link_libraries(heartbeat ${catkin_LIBRARIES})
