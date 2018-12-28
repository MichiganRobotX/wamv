# External Packages

These packages are not indexed in ROS and therefore it is easiest to include
them here.

## Notes
- `advanced_navigation_driver` compiles with the following warning:

        Warnings << advanced_navigation_driver:make /home/mjsyp/Documents/wamv/wamv_ws/logs/advanced_navigation_driver/build.make.000.log
        cc1: warning: command line option ‘-std=c++11’ is valid for C++/ObjC++ but not for C
        cc1: warning: command line option ‘-std=c++11’ is valid for C++/ObjC++ but not for C
        cc1: warning: command line option ‘-std=c++11’ is valid for C++/ObjC++ but not for C

- `ladybug3_ros` builds with the following warning:

        Warnings << ladybug3_ros_pkg:cmake /home/mjsyp/Documents/wamv/wamv_ws/logs/ladybug3_ros_pkg/build.cmake.000.log
        CMake Warning at /opt/ros/kinetic/share/catkin/cmake/catkin_package.cmake:166 (message):
            catkin_package() DEPENDS on 'DC1394_LIBRARY' but neither
            'DC1394_LIBRARY_INCLUDE_DIRS' nor 'DC1394_LIBRARY_LIBRARIES' is defined.
        Call Stack (most recent call first):
            /opt/ros/kinetic/share/catkin/cmake/catkin_package.cmake:102 (_catkin_package)
            CMakeLists.txt:41 (catkin_package)
