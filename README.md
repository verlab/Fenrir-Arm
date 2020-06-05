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

## Usage

### Simulator only

```sh
$ roslaunch fenrir simulator.launch
```

### Driver only (no RVIZ)

```sh
$ roslaunch fenrir driver.launch
```

### Driver with RVIZ

_This will initialize both the driver for the servos as well as the state publisher and RVIZ for visualization_

```sh
$ roslaunch fenrir initialize.launch
```

## Testing

Run the example python code

```sh
rosrun fenrir publish_joints.py
```

Or with an inline message publish

```sh
rostopic pub -r 1 /fenrir/joint_commands/ sensor_msgs/JointState  '{name: [Base, Shoulder, Elbow, Wrist, Gripper], position: [0.5, 0.5, 0.5, 0.5, 0.0], velocity: [], effort: []}'
```

## 3D Part Files and Construction

The full 3D assembly of the arm can be [downloaded here](https://a360.co/2XFVj2e).

Construction of the arm requires purchase of the components listed at [mechanical components](construction/mechanical_components.md).
