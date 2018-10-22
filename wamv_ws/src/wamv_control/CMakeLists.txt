cmake_minimum_required(VERSION 2.8.3)
project(wamv_control)

find_package(catkin REQUIRED COMPONENTS
	rospy
	std_msgs
	sensor_msgs
)

catkin_python_setup()

catkin_package(
	CATKIN_DEPENDS rospy std_msgs sensor_msgs
)

include_directories(include
	${catkin_INCLUDE_DIRS}
)

#catkin_install_python(PROGRAMS bin/hello
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# build node to publish signal for heartbeat
#target_link_libraries(signal ${catkin_LIBRARIES})

#add_executable(signal src/signal.cpp)
#add_dependencies(controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
