## WAMV Controls

### Launch File Description
- `controls.launch`: Launches heading PID controller, speed PID controller, and PID interpreter.

### Source File Description
- `calc_heading.py`: Calculates heading setpoint to single waypoint using global coordinates of WAMV location and global coordinates of waypoint.
- `pid_interpreter.py`: Contains logic to interpret heading/speed control efforts from PID controllers to commands sent to motors.
- `teleop_controller.py`: Sends commands from joy node to motors and lateral thrusters.
