cmake_minimum_required(VERSION 2.8.3)
project(wolvmarine_tests)

find_package(catkin REQUIRED COMPONENTS)

# add catkin packages
catkin_package()

# specify location of header files
include_directories(include ${catkin_INCLUDE_DIRS})
