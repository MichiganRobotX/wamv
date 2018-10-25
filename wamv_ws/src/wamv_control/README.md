## WAMV Controls

### Launch File Description
- `controls.launch`: Launches heading PID controller, speed PID controller, and PID interpreter.
- `test_heading_pid.launch`: **TESTING ONLY**. To be used for tuning the heading PID controller on its own. Launches the 'testing_heading_node', heading PID controller, and PID interpreter. Testing parameters are set in `heading_params.yaml`.
- `test_speed_pid.launch`: **TESTING ONLY**.  To be used for tuning the speed PID controller on its own. Launches the 'testing_speed_node', speed PID controller, and PID interpreter. Testing parameters are set in `speed_params.yaml`.
- `test_both_pid.launch`: **TESTING ONLY**.  To be used for tuning both the heading and speed PID controllers together. Launches both 'testing_heading_node' and 'testing_speed_node', heading and speed PID controllers, and PID interpreter.

### Param File Description
- `heading_params.yaml`: Contains the parameters to test the heading PID controller.
- `speed_params.yaml`: Contains the parameters to test the speed PID controller.

### Source File Description
- `calc_heading.py`: Calculates heading setpoint to single waypoint using global coordinates of WAMV location and global coordinates of waypoint.
- `pid_interpreter.py`: Contains logic to interpret heading/speed control efforts from PID controllers to commands sent to motors.
- `teleop_controller.py`: Sends commands from joy node to motors and lateral thrusters.
- `set_test_headings.py`: **TESTING ONLY**. Sets the setpoint for the heading PID controller, switching from a max value to a min value in the specified period, outlined in `heading_params.yaml`.
- `set_test_speeds.py`: **TESTING ONLY**. Sets the setpoint for the speed PID controller, switching from a max value to a min value in the specified period outlined in `speed_params.yaml`.
