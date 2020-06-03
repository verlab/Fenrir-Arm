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

Run the fenrir_driver.launch file using:

```sh
$ roslaunch fenrir fenrir_driver.launch
```

## Use simulator RVIZ

Run the simulator launch:

```sh
$ roslaunch fenrir fenrir_simulation.launch
```

You can see the robot simulation on RVIZ.

## 3D Part Files and Construction

The full 3D assembly of the arm can be found at
<iframe src="https://myhub.autodesk360.com/ue2ae9b78/shares/public/SH919a0QTf3c32634dcfc2cfa3694b63d784?mode=embed" width="640" height="480" allowfullscreen="true" webkitallowfullscreen="true" mozallowfullscreen="true"  frameborder="0"></iframe>

Construction of the arm requires purchase of the components listed at [mechanical components](construction/mechanical_components.md).
