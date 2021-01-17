#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import yaml
import sys
import os
import numpy as np

from colors import *
from threading import Lock

from dynamixel_sdk.packet_handler import *
from dynamixel_sdk.port_handler import *
from Dynamixel import Dynamixel

import rospkg
import rospy

from fenrir_driver.msg import MotorInfo

from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool



# Define the getch function according to os
if os.name == 'nt':
    import msvcrt

    def getch():
        return msvcrt.getch().decode()
else:
    import sys
    import tty
    import termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Define driver class


class dynamixel_driver():
    """ Class to create the dynamixel driver"""

    def __init__(self):

        rospy.init_node('dynamixel_driver', anonymous=False)
        rospack = rospkg.RosPack()
        path = rospack.get_path('fenrir_driver')

        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~baudrate', 1000000)
        self.yaml = rospy.get_param('~config', path + '/config/arm.yaml')

        self.portHandler = PortHandler(self.port)
        self.rate = rospy.Rate(30)
        self.state = JointState()
        self.info = MotorInfo()
        self.write = Lock()

        # Open port
        try:
            self.portHandler.openPort()
            rospy.loginfo("Open the port {} with success!".format(self.port ))
            # printC(INFO, "Open port successful")
        except serial.serialutil.SerialException:
            # printC(INFO, "Failed to open port %s" % self.port)
            rospy.logerr("Failed to open the port {}! Check if the device drive is the fact the one conected to the robot.".format(self.port ))
            getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(self.baudrate):
            # printC(INFO, "Succeeded to set the baudrate")
            rospy.loginfo("Setting the UART baudrate to {} bps.".format(self.baudrate ))
        else:
            # printC(INFO, "Failed to change the baudrate")
            rospy.logerr("Unable to set UART baudrate to {} bps! Check if the baudrate is supported by dynamixel motors.".format(self.baudrate ))
            # printC(KEY)
            getch()
            quit()

        # Instanciate the subscriber and publisher
        rospy.Subscriber("/joint_commands", JointState, self.jointCallback)
        rospy.Service('/set_torque', SetBool, self.setTorqueCallback)
        self.pubJointPosition = rospy.Publisher('/joint_states', JointState, queue_size=1)
        self.pubJointInfo = rospy.Publisher('/joint_info', MotorInfo, queue_size=1)

    def __del__(self):
        try:
            self.setTorque (state=False)
        except AttributeError:
            pass

    def jointCallback(self, data):
        self.write.acquire()
        for i, name in enumerate(data.name):
            for joint in self.dynamixels.keys():
                if name in joint:
                    if self.dynamixels[joint].mode == 3:
                        self.dynamixels[joint].writeAngle(max(min(data.position[i]+np.pi, 2.0*np.pi), 0.0))
                    elif self.dynamixels[joint].mode == 1:
                        self.dynamixels[joint].writeVelocity(data.velocity[i])
        self.write.release()

    def setTorqueCallback(self, data):
        self.setTorque(state=data.data)
        return True, "Setting joint torque to {}.".format(data.data)

    def setDynamixels (self):
        self.dynamixels = {}
        with open(self.yaml) as file:
            # printC(INFO, "list of motors: ")
            rospy.loginfo("Searching for dynamixel motors: ")
            documents = yaml.safe_load(file)
            for item, doc in documents.items():
                self.dynamixels[item] = Dynamixel(
                    self.portHandler, doc["ID"], float(doc["Protocol"]))
                for flag, value in doc.items():
                    if (flag != "ID" and flag != "Protocol"):
                        self.dynamixels[item].write(flag, value)
            

    def updatePosition (self):
        # Updating position limit
        # printC(INFO, "Updating positions")
        rospy.loginfo("Updating joints to their limits. Wait...")
        for dyn in self.dynamixels:
            self.dynamixels[dyn].updateLimits()
        rospy.loginfo("All joints was updated!")

    def setTorque (self, state=True):
        # Updating torque of motors
        self.write.acquire()
        for key in self.dynamixels:
            self.dynamixels[key].write("Torque_Enabled", int(state))
        self.write.release()
        if state:
            # printC(INFO, "Enabling motors torque")
            rospy.logwarn("Enabling the motors torque. BE CAREFUL!")
        else:
            rospy.logwarn("Desabling the motors torque. BE CAREFUL!")
            # printC(INFO, "Desabling motors torque")

    #TODO verify the difference here from workbench
    def verifyMoving (self, event=None):
        for key in self.dynamixels:
            test = self.dynamixels[key].reachedGoal()
            if not test:
                self.is_moving = True
                return
        self.is_moving = False

    def setControlMode(self):
        for dyn in self.dynamixels:
            self.dynamixels[dyn].setControlMode()


    def run(self):
        self.setDynamixels()
        self.updatePosition()
        self.setControlMode()
        self.setTorque()
        #rospy.Timer(rospy.Duration(0.5), self.verifyMoving)
        while not rospy.is_shutdown():
            names = []
            angles, efforts, velocities = [], [], []
            currents, voltages, temperatures = [], [], []

            self.write.acquire()
            for dyn in self.dynamixels:
                names.append(dyn)
                # information about position
                angles.append(self.dynamixels[dyn].readAngle()[0] - np.pi)
                efforts.append(self.dynamixels[dyn].readLoad()[0])
                velocities.append(self.dynamixels[dyn].readVelocity()[0])

                # information about status
                currents.append(self.dynamixels[dyn].readCurrent()[0])
                voltages.append(self.dynamixels[dyn].readVoltage()[0])
                temperatures.append(self.dynamixels[dyn].readTemp()[0])
            self.write.release()

            # Remove shadow joints
            for i, joint in enumerate(names):
                if 'shadow' in joint:
                    del names[i]
                    del angles[i]
                    del efforts[i]
                    del velocities[i]
                    
            self.state.name = names
            self.state.position = angles
            self.state.effort = efforts
            self.state.velocity = velocities

            self.info.name = names
            self.info.current = currents
            self.info.voltage = voltages
            self.info.temperature = temperatures

            self.state.header.stamp = rospy.Time.now()
            self.pubJointPosition.publish(self.state)
            self.pubJointInfo.publish(self.info)
            self.rate.sleep()


if __name__ == "__main__":
    print("╔═══════════════════════════════════════════════════════╗")
    print("║\t\33[92m███████╗\33[91m███████╗\33[92m███╗   ██╗\33[91m██████╗ \33[92m██╗\33[91m██████╗ \33[0m  \t║")
    print("║\t\33[92m██╔════╝\33[91m██╔════╝\33[92m████╗  ██║\33[91m██╔══██╗\33[92m██║\33[91m██╔══██╗\33[0m  \t║")
    print("║\t\33[92m█████╗  \33[91m█████╗  \33[92m██╔██╗ ██║\33[91m██████╔╝\33[92m██║\33[91m██████╔╝\33[0m  \t║")
    print("║\t\33[92m██╔══╝  \33[91m██╔══╝  \33[92m██║╚██╗██║\33[91m██╔══██╗\33[92m██║\33[91m██╔══██╗\33[0m  \t║")
    print("║\t\33[92m██║     \33[91m███████╗\33[92m██║ ╚████║\33[91m██║  ██║\33[92m██║\33[91m██║  ██║\33[0m  \t║")
    print("║\t\33[92m╚═╝     \33[91m╚══════╝\33[92m╚═╝  ╚═══╝\33[91m╚═╝  ╚═╝\33[92m╚═╝\33[91m╚═╝  ╚═╝\33[0m  \t║")
    print("╚═══════════════════════════════════════════════════════╝")
                                             

    drv = dynamixel_driver()
    drv.run()
