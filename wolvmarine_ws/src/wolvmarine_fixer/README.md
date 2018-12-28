## WAMV Controls

### Launch File Description
- `controls.launch`: Launches heading PID controller, speed PID controller, and PID interpreter. Puts everything in the `/pid` namespace. Arguments:

    `heading_pid`: Boolean value that determines if the heading PID controller should be launched. Default is true.
    `speed_pid`: Boolean value that determines if the speed PID controller should be launched. Default is true.

- `interpreter.launch`: Launches the PID interpreter. Arguments:

    `param_file`: The name of the parameter file in the `params/` folder: Default is `interpreter_params.yaml`
    `topic_to_port_motor`
    `topic_to_strbrd_motor`
    `topic_from_heading_controller`
    `topic_from_speed_controller`

- `pid.launch`: Launches a PID controller. Arguments:

    `param_file`: The name of the parameter file in the `params/` folder: Default is `controller_params.yaml`
    `setpoint`: The topic name of the controller's setpoint.
    `state`: The topic name of the plant's current state.

- `test_heading_pid.launch`: **TESTING ONLY**. To be used for tuning the heading PID controller on its own. Launches the 'testing_heading_node', heading PID controller, and PID interpreter. Testing parameters are set in `heading_params.yaml`.

- `test_speed_pid.launch`: **TESTING ONLY**.  To be used for tuning the speed PID controller on its own. Launches the 'testing_speed_node', speed PID controller, and PID interpreter. Testing parameters are set in `speed_params.yaml`.

- `test_pid.launch`: **TESTING ONLY**.  To be used for tuning both the heading and speed PID controllers together. Launches both 'testing_heading_node' and 'testing_speed_node', heading and speed PID controllers, and PID interpreter.

### Param File Description
- `heading_test_params.yaml`: Contains the parameters to test the heading PID controller.
- `speed_test_params.yaml`: Contains the parameters to test the speed PID controller.
- `heading_controller_params.yaml`: Contains the parameters that define the heading PID controller.
- `speed_controller_params.yaml`: Contains the parameters that define the speed PID controller.
- `interpreter_params.yaml`: Contains the parameters to modify the PID interpreter. Currently empty.

### Source File Description
- `calc_heading.py`: Calculates heading setpoint to single waypoint using global coordinates of WAMV location and global coordinates of waypoint. (moved to wamv_navigation)

- `pid_interpreter.py`: Contains logic to interpret heading/speed control efforts from PID controllers to commands sent to motors.

- `teleop_controller.py`: Sends commands from joy node to motors and lateral thrusters.

- `pid_tester.py`: **TESTING ONLY**. Sets the setpoint for the PID controller, switching from a value `point_a` to a value `point_b` in the specified period.
