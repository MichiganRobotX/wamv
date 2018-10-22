#!/bin/bash

sudo jstest /dev/input/js2

# Set the joystick permission
sudo chmod a+rw /dev/input/js2

# Map the joystick in ros
# for wireless joystick, use js2
# for wired joystick, use js0
rosparam set joy_node/dev "/dev/input/js2"

# Start the Joystick Node
rosrun joy joy_node

# Open a new terminal and show the joystick
# this doesn't work
# terminal -e rostopic echo joy
