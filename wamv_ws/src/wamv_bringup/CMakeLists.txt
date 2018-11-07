cmake_minimum_required(VERSION 2.8.3)
project(wamv_bringup)

find_package(catkin REQUIRED COMPONENTS
    rviz
    wamv_driver
    wamv_transform
)

catkin_package()
