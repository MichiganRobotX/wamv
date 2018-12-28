# Arduino code

These folders contain the `.ino` files that run on the Arduinos and not natively
on the remote computer. The status light and motor controller code is up-to-date
and utilized on the boat. The emergency stop code may be from the original
e-stop, in which case, it is not used.

## Action items

- Import custom ROS message types. The current code uses ROS standard message
  types and as such, the master computer implements an interpreter and
  controller for each of the Arduino devices. Instead, it would be better (in
  terms of conciseness and proper ROS usage) to import and use custom message
  types in the Arduinos.

- Determine if the `emergency_stop` code is needed, and remove it if not.
