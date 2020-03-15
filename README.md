# Fenrir Arm
A low-cost, ROS-compatible Robotic Arm using Dynamixels Servo Motors.

We make available all the necessary 3D parts and libraries for communicating with the arm using a ROS interface. 

## Dependecies
- ROS Kinect (http://wiki.ros.org/kinetic/Installation)

## Installation
- Using git (or download the zip file) clone this repository into the "source code" folder of your ROS workspace (e.g. ~/catkin_ws/src ).

```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/verlab/Fenrir-Arm.git
```

- Compile your ROS workspace directory (e.g. ~/catkin-ws ):

```sh
$ cd ~/catkin_ws
$ catkin_make # or catkin build
```

- Fixing package dependencies:

```sh
$ rosdep install fenrir
```

## Start ROS Driver
Run the kaku_driver.launch file with:

```sh
$ roslaunch fenrir kaku_driver.launch
```

## 3D Parts Files

The full 3D assembly of the arm can be found at: https://a360.co/3aX4rV5
