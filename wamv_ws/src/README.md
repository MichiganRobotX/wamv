## WAM-V

- `external_packages` : Packages that are not indexed by ROS.
- `wamv_bringup` : Launch files and scripts for bringing up the Puget and the laptop.
- `wamv_control` : Control configuration including teleoperation controls, PID controllers, and PID interpreter.
- `wamv_description` : Robot description (URDF). Currently not utilized.
- `wamv_detection` : Object detection and image recognition.
- `wamv_drivers` : Launch files for the Velodyne LiDARs and Ladybug cameras.
- `wamv_gazebo` : Gazebo plugin definitions and extensions to the robot URDF. Currently not utilized.
- `wamv_localization` : Mapping and GPS.
- `wamv_msgs` : Messages.
- `wamv_navigation` : Path planning and waypoint following.
- `wamv_ros` : Metapackage.
- `wamv_tasks` : RobotX task definitions and implementations, including the heartbeat signal.
- `wamv_transform` : Broadcasts the wamv transforms.
- `wamv_viz` : Visualization configuration for rviz.

## Installing Dependencies

To install all the dependencies for the wamv package, run the following in the base directory of the wamv workspace:

    rosdep install --from-paths src --ignore-src -r -y

## Running Launch Files

- `wamv_bringup`

    - `bringup_puget.launch` :
    - `bringup_laptop.launch` :

- `wamv_control`

    - `controls.launch` : Runs the PIDs and the PID interpreter.
    - `teleoperations.launch` : The teleoperations controller.   
    - `test_heading_pid.launch` : **FOR PID TUNING**
    - `test_pid.launch` : **FOR PID TUNING**
    - `test_speed_pid.launch` : **FOR PID TUNING**

- `wamv_drivers`

    - `drivers.launch` : Runs the LiDARs, advanced navigation device, Ladybug camera, odometry publisher, and motor controller driver (rosserial to the arduino).

- `wamv_navigation`

    - `pid_tuning.launch` : Publishes the boat's current linear and angular speed to `/current_linear_speed` and `/current_angular_speed`

- `wamv_transform`

    - `transforms.launch` : Publishes the static transforms for the wamv.
