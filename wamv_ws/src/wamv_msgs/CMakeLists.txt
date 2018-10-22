cmake_minimum_required(VERSION 2.8.3)
project(wamv_msgs)

find_package(catkin REQUIRED COMPONENTS
	std_msgs
	message_generation
)

add_message_files(
	FILES
	Heartbeat.msg
	Gate.msg
	ScanCode.msg
	SymbolDock.msg
	MotorSpeeds.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
	CATKIN_DEPENDS
	std_msgs
	message_runtime
)
