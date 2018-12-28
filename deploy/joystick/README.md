## Deploying the Xbox controller

These instructions apply to the Logitech Xbox-style controller. First, ensure
that the **mode light is off**. The mode light affects the mapping of the
controller's inputs (buttons and joysticks), and the code has been configured
to work with the mode light disabled.



### Automatic Deployment

##### Wired controller

Open a terminal in this directory and run `./wired_joystick_setup.sh`

##### Wireless controller


Open a terminal in this directory and run `./wireless_joystick_setup.sh`


### Manual Deployment

##### Wired controller

Open a terminal and run the following:

    sudo chmod a+rw /dev/input/js1
    rosparam set joy_node/dev "/dev/input/js1"
    rosrun joy joy_node


##### Wireless controller

Open a terminal and run the following:

    sudo chmod a+rw /dev/input/js2
    rosparam set joy_node/dev "/dev/input/js2"
    rosrun joy joy_node


### Verifying the Joystick Output

To check the controller input to the base computer, open a terminal and run:

    sudo jstest /dev/input/jsX

To check the controller output in ROS, open a terminal and run:

    rostopic echo /joy


### Joystick Configuration

The joystick and `/joy` node mapping is as follows.

In the array `/joy.buttons[]`

| Index | Button
|   -   |         -         
| 0 | Green/A
| 1 | Red/B
| 2 | Blue/X
| 3 | Yellow/Y
| 4 | Left Button
| 5 | Right Button
| 6 | Back
| 7 | Start
| 8 | Logitech Button (Center)
| 9 | Left Stick (Press in)
| 10 | Right Stick (Press in)

In the array `/joy.axis[]`

| Index | Axis
|-|-|
| 0 | Left Stick, Left-Right
| 1 | Left Stick, Up-Down
| 2 | Left Trigger
| 3 | Right Stick, Left-Right
| 4 | Right Stick, Up-Down
| 5 | Right Trigger
| 6 | D-Pad, Left-Right
| 7 | D-Pad, Up-Down


## Resources

http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick
