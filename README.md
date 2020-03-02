# KAKU-Arm
A low-cost, ROS-compatible Robotic Arm using Dynamixels Servo Motors.


## Dependecies
- ROS Kinect (http://wiki.ros.org/kinetic/Installation)

## How to install
- Using git (or download the zip file) clone this repository into the "source code" folder of your ROS workspace (e.g. ~/catkin_ws/src ).

```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/verlab/KAKU-Arm.git
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

## How to start communication
Run the kaku_driver.launch file with:

```sh
$ roslaunch fenrir kaku_driver.launch
```
