# The Michigan WolvMarine, Michigan's Maritime RobotX Team

This is the central code repository for the Michigan Maritime RobotX team and
the WolvMarine. The current setup utilizes ROS Kinetic and Ubuntu 16.04 LTS.

## Contents

The contents of this repository are as follows:
- `deploy/` contains instructions and scripts for field deployment of the
  WolvMarine.
- `setup/` contains instructions and scripts for setting up the WolvMarine and
  its constituent parts. This includes setting up the remote as base computers,
  Arduino boards, network devices, etc.
- `wolvmarine_ws/` is the catkin workspace for the WolvMarine and contains its
  ROS packages.

## Field deployment

TBD

## Action items

Each of the contained folders details more specific action items. The following
are larger considerations given lessons learned from the past competition.

- Develop field deployment scripts and documentation.

- Consider moving the INS sensor. For navigation, it is important that the
  center of the robot is placed at the rotational center of the robot. To
  reduce excessive movement noise, the INS should be placed  the base_link be paced at the center of floatation?

- Robust remote control solution.

  The current setup utilizes an Xbox-style controller whose inputs are funneled
  through the base computer to the remote computer and the to the Arduino acting
  as a motor controller. The communication occurs via a TCP socket, which
  sacrifices speed to prioritize completeness. In addition, joystick commands
  are posted to a ROS topic, but only as the commands change. This causes issues
  when packets are lost.

  During the competition, wireless networks, bandwidths, and center fequencies
  were not controlled by the competition's leadership. As such, wireless
  communication became a battle of who has the highest powered base-station
  antenna. Our performance suffered due to these network issues given that our
  remote control setup was using TCP.

  Two immediate solutions would be (1) to implement a UDP socket for sending
  remote control commands, or (2) have

- Adopt the taxonomy of *remote* and *base* computers, rather than referring to
  them by *puget* and *dell*.

- Adopt the platform name of WolvMarine in the documentation, rather than
  referring to it as the WAM-V.

- Setup auto-generation documentation software, such as sphinx. This would allow
  us to comment only once, in the code. For example, each of the developed ROS
  nodes may accept a set of parameters. The documentation for these parameters
  should be given in file that initializes the node, rather than including
  definitions in separate parameter files.

- Parameter files should be avoided from now on. In some cases, they make sense,
  but in most others, it is clearer to use the `rosparam` tag when spinning up
  a node in a launch file.

- It may be better to consolidate the descriptions and action items from the
  multiple `README`'s.
