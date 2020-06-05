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
        rospack = rospkg.RosPack()
        path = rospack.get_path('fenrir')
        self.yaml = rospy.get_param('~config', path + '/config/arm.yaml')
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
                # A 'shadow joint' just exists on the YAML file to mirror another servo via hardware, but should be ignored on sofware
                if 'Shadow_ID' in doc and doc['Shadow_ID'] != 254:
                    continue

                self.actualPosition[item] = 0
                self.positions[item] = 0
                self.initPos[item] = 0
                
                # Get max angle depending on protocol. For protocol 1 max angle is 1023 and protocol 2 it is 4095
                max_angle = 4095.0
                
                if (int(doc['Protocol']) == 2):
                    self.times[item] = (doc['Profile_Acceleration']* 214.577, doc['Profile_Velocity']*0.229)
                else:
                    self.times[item] = (30* 214.577, 120*0.229)
                    max_angle = 1023.0

                # Get min and max angles and convert to radians
                self.limits[item] = [0, 3.15]
                if 'Min_Position' in doc:
                    self.limits[item][0] = doc['Min_Position']/max_angle*6.28
                if 'Max_Position' in doc:
                    self.limits[item][1] = doc['Max_Position']/max_angle*6.28

                # Debug info
                console_info = 'Joint %s \t started with min angle=%f, max angle=%f' % ( item, self.limits[item][0], self.limits[item][1] )
                printC(INFO, console_info)
                

        rospy.Subscriber('/fenrir/joint_commands', JointState, self.jointCallback)
        self.pubJointPosition = rospy.Publisher('/fenrir/joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(50)

    def jointCallback(self, data):
        printC(INFO, 'Position command received')
        for i, name in enumerate(data.name):
            self.positions[name] = clamp(data.position[i], self.limits[name][0], self.limits[name][1])
            t1 = 64*self.times[name][1]/self.times[name][0]
            t2 = 60*abs(self.positions[name] - self.actualPosition[name])*0.5/(self.times[name][1]*2*np.pi) + t1
            self.t3[name] = (t1 + t2)*1000
            self.initPos[name] = self.actualPosition[name]
        self.iniTime = int(round(time.time() * 1000))

    def run(self):
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

# Clap function
# Source: https://stackoverflow.com/questions/5996881/how-to-limit-a-number-to-be-within-a-specified-range-python
def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

if __name__ == '__main__':
    drvS = dynamixel_driver_sim()
    drvS.run()
