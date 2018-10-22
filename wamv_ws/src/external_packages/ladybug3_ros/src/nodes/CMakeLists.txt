# common source files for both node and nodelet driver interfaces
set(ladybug3_ros_pkg_SOURCES 
  driverladybug.cpp
  dev_ladybug.cpp
  featuresladybug.cpp
  format7ladybug.cpp
  modes.cpp)

## The following used to be necessary rosbuild on Mac OSX.
## No telling what, if anything, is needed with catkin.

# Mac OSX requires extra linker parameters
#if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
#  set(DC1394 dc1394 "-framework CoreServices")
#else(CMAKE_SYSTEM_NAME MATCHES "Darwin")
#  set(DC1394 dc1394)
#endif(CMAKE_SYSTEM_NAME MATCHES "Darwin")

# build and install the node
add_executable(ladybug_node ladybug_node.cpp ${ladybug3_ros_pkg_SOURCES})
add_dependencies(ladybug_node ${ladybug3_ros_pkg_EXPORTED_TARGETS})
target_link_libraries(ladybug_node ${catkin_LIBRARIES} ${DC1394_LIBRARY} ${OpenCV_LIBS})

# build and install the node
add_executable(driver_jpeg_disk driver_jpeg_disk.cpp)
add_dependencies(driver_jpeg_disk ${ladybug3_ros_pkg_EXPORTED_TARGETS})
target_link_libraries(driver_jpeg_disk ${catkin_LIBRARIES} ${DC1394_LIBRARY} ${OpenCV_LIBS})
