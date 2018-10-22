# Contents

This directory contains scripts to install the requisite software.

## Operating System

If the operating system needs to be installed, see below for the operating system and preferred boot device.

#### Laptop
Ubuntu Desktop 16.04.5 LTS 64-bit via bootable USB drive.

#### Puget
Ubuntu Desktop 16.04.5 LTS 64-bit via bootable USB drive.

#### Raspberry Pi
Raspbian Stretch Desktop 4.14 via bootable microSD card.



## Software

The scripts to install the software are segregated into the following:
* `install_arduino` installs arduino packages.
* `install_base` installs packages for java, python, camera, sublime text, terminator, and some other essentials. It also allows ssh and performs a simple network setup.
* `install_CV` installs OpenCV for computer vision.
* `install_LCM` installs Lightweight Communications and Marshalling.
* `install_ROS` installs Robot Operating System, Kinetic.

After the operating system has been installed, see below for installing the software.

#### Laptop
Open a terminal in this directory and run ```./install_Laptop```. This will install everything in `install_base`, `install_ROS`, `install_CV`, and `install_arduino`.

#### Puget
Open a terminal in this directory and run ```./install_Puget```. This will install everything in `install_base`, `install_ROS`, `install_CV`, `install_LCM`, and `install_arduino`.

#### Raspberry Pi

TBD
