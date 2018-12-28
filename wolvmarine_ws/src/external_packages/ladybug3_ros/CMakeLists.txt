cmake_minimum_required(VERSION 2.8)
project(ladybug3_ros_pkg)

add_definitions(-std=c++11)

# Find catkin macros and libraries, also other catkin packages.
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  diagnostic_updater
  driver_base
  dynamic_reconfigure
  image_transport
  nodelet
  roscpp
  rostest
  sensor_msgs
  tf
  cv_bridge
)

# This driver uses Boost threads
find_package(Boost REQUIRED COMPONENTS thread)

# Resolve system dependency on libdc1394, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
pkg_check_modules(DC1394_PKGCONF REQUIRED libdc1394-2)
find_path(DC1394_INCLUDE_DIR
  NAMES dc1394.h
  PATHS ${DC1394_PKGCONF_INCLUDE_DIRS}
)
find_library(DC1394_LIBRARY
  NAMES dc1394
  PATHS ${DC1394_PKGCONF_LIBRARY_DIRS}
)

# auto-generate dynamic reconfiguration GUI before catkin_package()
generate_dynamic_reconfigure_options(cfg/Ladybug.cfg)

catkin_package(CATKIN_DEPENDS
                 camera_info_manager
                 diagnostic_updater
                 dynamic_reconfigure
                 image_transport
                 nodelet
                 cv_bridge
                 roscpp
                 tf
               DEPENDS Boost DC1394_LIBRARY
               LIBRARIES src/nodes/ladybug_nodelet)

include_directories(
    ${catkin_INCLUDE_DIRS}
    cfg/cpp
)

# build package sources
# add_subdirectory(src/nodes)

# common source files for both node and nodelet driver interfaces
set(ladybug3_ros_pkg_SOURCES
  src/nodes/driverladybug.cpp
  src/nodes/dev_ladybug.cpp
  src/nodes/featuresladybug.cpp
  src/nodes/format7ladybug.cpp
  src/nodes/modes.cpp)

## The following used to be necessary rosbuild on Mac OSX.
## No telling what, if anything, is needed with catkin.

# Mac OSX requires extra linker parameters
#if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
#  set(DC1394 dc1394 "-framework CoreServices")
#else(CMAKE_SYSTEM_NAME MATCHES "Darwin")
#  set(DC1394 dc1394)
#endif(CMAKE_SYSTEM_NAME MATCHES "Darwin")

# build and install the node
add_executable(ladybug_node src/nodes/ladybug_node.cpp ${ladybug3_ros_pkg_SOURCES})
add_dependencies(ladybug_node ${ladybug3_ros_pkg_EXPORTED_TARGETS})
target_link_libraries(ladybug_node ${catkin_LIBRARIES} ${DC1394_LIBRARY} ${OpenCV_LIBS})

# build and install the node
add_executable(driver_jpeg_disk src/nodes/driver_jpeg_disk.cpp)
add_dependencies(driver_jpeg_disk ${ladybug3_ros_pkg_EXPORTED_TARGETS})
target_link_libraries(driver_jpeg_disk ${catkin_LIBRARIES} ${DC1394_LIBRARY} ${OpenCV_LIBS})
