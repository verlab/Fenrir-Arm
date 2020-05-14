# Fenrir Arm

A low-cost, ROS-compatible Robotic Arm using Dynamixels Servo Motors.

We make available all the necessary 3D parts and libraries for communicating with the arm using a ROS interface.

## Dependecies

-   ROS Kinect (http://wiki.ros.org/kinetic/Installation)
-   Dynamixel SDK (http://wiki.ros.org/dynamixel_sdk)

## Installation

-   Using git (or download the zip file) clone this repository into the "source code" folder of your ROS workspace (e.g. ~/catkin_ws/src ).

```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/verlab/Fenrir-Arm.git
```

-   Compile your ROS workspace directory (e.g. ~/catkin-ws ):

```sh
$ cd ~/catkin_ws
$ catkin_make # or catkin build
```

-   Fixing package dependencies:

```sh
$ rosdep install fenrir
```

## Start ROS Driver

Run the fenrir_driver.launch file with:

```sh
$ roslaunch fenrir fenrir_driver.launch
```

## Use simulator RVIZ

Run the simulator launch:

```sh
$ roslaunch fenrir fenrir_simulator.launch
```

You can see the robot simulation on RVIZ.

## 3D Part Files

The full 3D assembly of the arm can be found at: https://a360.co/3aX4rV5
