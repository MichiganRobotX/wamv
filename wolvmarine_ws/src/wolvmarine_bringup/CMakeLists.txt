cmake_minimum_required(VERSION 2.8.3)
project(wolvmarine_bringup)

find_package(catkin REQUIRED COMPONENTS
    wolvmarine_controls
    wolvmarine_drivers
    wolvmarine_fixer
    wolvmarine_localization
    wolvmarine_transforms
    wolvmarine_viz
)

catkin_package()
