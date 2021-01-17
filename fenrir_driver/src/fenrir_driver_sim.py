#!/usr/bin/env python2

from sensor_msgs.msg import JointState
from colors import *
import numpy as np
import threading
import rospkg
import rospy
import yaml
import time

class dynamixel_driver_sim():

    ''' Class to simulate dynamixel motors '''

    def __init__(self):
        rospy.init_node('dynamixel_driver_sim', anonymous=False)
        printC(TITLE, "Initializing simulated driver")
        rospack = rospkg.RosPack()
        self.path = rospack.get_path('fenrir')
        self.yaml = rospy.get_param('~config', self.path + '/config/arm.yaml')
        self.yamlHome = rospy.get_param('~home', self.path + '/config/home.yaml')
        self.shadow = []
        self.actualPosition = {}
        self.positions = {}
        self.limits = {}
        self.times = {}
        self.t3 = {}
        self.initPos = {}
        self.iniTime = rospy.Time.now()
        with open(self.yaml) as file:
            documents = yaml.safe_load(file)
            for item, doc in documents.items():

                # Check if not 'shadow joint'
                # The 'shadow joint' just exists on the YAML file to mirror another servo via hardware,
                # but should be ignored on sofware
                if 'Shadow_ID' in doc and doc['Shadow_ID'] != 254:
                    self.shadow.append(item)
                    continue

                # Set initial position to 0
                pos = self.__homePosition(item)
                self.actualPosition[item] = pos
                self.positions[item] = pos
                self.initPos[item] = pos

                # Get max angle/Time perfil depending on protocol. For protocol 1 max angle is 1023 and
                # protocol 2 it is 4095
                if (int(doc['Protocol']) == 2):
                    self.times[item] = (doc['Profile_Acceleration']* 214.577, doc['Profile_Velocity']*0.229)
                    max_angle = 4095.0
                else:
                    self.times[item] = (30*214.577, 120*0.229)
                    max_angle = 1023.0

                # Get min and max angles and convert to radians
                self.limits[item] = [0, 3.15]
                if 'Min_Position' in doc:
                    self.limits[item][0] = doc['Min_Position']/max_angle*6.28
                if 'Max_Position' in doc:
                    self.limits[item][1] = doc['Max_Position']/max_angle*6.28

                # Debug info
                console_info = ("Joint %s \t started with min angle=%f, " % (item, self.limits[item][0]) +
                    "max angle=%f and initial pos=%f" % (self.limits[item][1], pos))
                printC(INFO, console_info)

        rospy.Subscriber('/fenrir/joint_commands', JointState, self.jointCallback)
        self.pubJointPosition = rospy.Publisher('/fenrir/joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(50)


    def jointCallback(self, data):
        printC(INFO, 'Position command received')
        for i, name in enumerate(data.name):
            if name in self.shadow:
                continue
            self.positions[name] = self.__clamp(name, data.position[i], self.limits[name][0], self.limits[name][1])
            t1 = 64*self.times[name][1]/self.times[name][0]
            t2 = 60*abs(self.positions[name] - self.actualPosition[name])*0.5/(self.times[name][1]*2*np.pi) + t1
            self.t3[name] = (t1 + t2)*1000
            self.initPos[name] = self.actualPosition[name]
        self.iniTime = int(round(time.time() * 1000))

    def run(self):
        printC(TITLE, "Initializing control")
        while not rospy.is_shutdown():
            self.state = JointState()
            for name in self.actualPosition:
                if ((self.positions[name] - self.actualPosition[name]) > 0.01):
                    timeV = abs(int(round(time.time() * 1000)) - self.iniTime)
                    self.actualPosition[name] = self.initPos[name] + (1/(1 + np.exp(-4*(timeV/self.t3[name] - 1))))*abs(self.positions[name] - self.initPos[name])
                elif ((self.positions[name] - self.actualPosition[name]) < -0.01):
                    timeV = abs(int(round(time.time() * 1000)) - self.iniTime)
                    self.actualPosition[name] = self.initPos[name] - (1/(1 + np.exp(-4*(timeV/self.t3[name] - 1))))*abs(self.positions[name] - self.initPos[name])
                self.state.name.append(name)
                self.state.position.append(self.actualPosition[name])
            self.state.header.stamp = rospy.Time.now()
            self.pubJointPosition.publish(self.state)
            self.rate.sleep()

    # "Private" functions

    # Clap function
    def __clamp(self, name, n, minn, maxn):
        value = max(min(maxn, n), minn)
        if (n < minn) or (n > maxn):
            printC (WARN_SIGN, "joint %s received a position value out of bounds." % (name) +
                    " Clipping the value to %s." % round(value, 3))
        return value

    # Home position function
    def __homePosition(self, jointName):
        try:
            with open(self.yamlHome) as file:
                documents = yaml.safe_load(file)
                for item, doc in documents.items():
                    if item == jointName:
                        return doc
                printC(ERROR, "Home position for joint %s not defined." % (jointName) +
                       " Setting position to 0 rad.")
                return 0
        except IOError:
            printC(ERROR, "file %s don't exist, setting position to 0 rad." % (self.yamlHome))
            return 0

if __name__ == '__main__':
    drvS = dynamixel_driver_sim()
    drvS.run()
