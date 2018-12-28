# Michigan Robot X

This repository is for all Michigan RobotX Code Development and Builds

## Installation

### Base Software Installation
1. Clone this Repository and got into the ```wamv_setup``` folder
2. To install individual packages you can select the respective installation files. ```install_base``` is recommended to be installed on a fresh system
3. For the Laptop, run ```./install_Laptop``` and it will automatically install the recommended packages
4. For the Puget, run ```./install_Puget``` to install the recommended packages

### ROS Workspace Installation
1. On the Puget, copy the wamv_drivers workspace into the Documents folder and run ```./install``` from within the workspace. You might have to add permissions to run the file using ```chmod +x install```. Do the same for the wamv_base, wamv_autonomy, wamv_calibration and wamv_misc workspaces.

2. On the Laptop, install wamv_drivers, wamv_base and wamv_autonomy workspaces same as above.


### Network Setup:
1. Make sure both the ops laptop and the WAM-V computer are on the same network.<br/>
	<strong>Puget IP</strong>: 192.168.1.103<br/>
	<strong>Laptop IP</strong>: 192.168.1.104<br/>

2. On <strong>Puget (in every terminal you open)</strong>:<br/>
``` bash
export ROS_IP=192.168.1.103
export ROS_MASTER_URI=http://192.168.1.103:11311
```

3. On <strong>Laptop (in every terminal you open)</strong>: <br/>
``` bash
export ROS_IP=192.168.1.104
export ROS_MASTER_URI=http://192.168.1.103:11311
```
<em>Steps 2 and 3 can also be put into the <strong>~/.bashrc</strong> file on both the systems.</em>

4. ssh into the Puget from the laptop with the following details:  <br/>
``` bash
ssh wamv@192.168.1.103
password: RobotXuser!
```  
