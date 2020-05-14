#!/usr/bin/env python2

from sensor_msgs.msg import JointState
from colors import *
import numpy as np
import threading
import rospkg
import rospy
import yaml

class dynamixel_driver_sim():

    """ Class to simulate dynamixel motors """

    def __init__(self):
        rospy.init_node('dynamixel_driver_sim', anonymous=False)
        rospack = rospkg.RosPack()
        path = rospack.get_path('fenrir')
        self.yaml = rospy.get_param('~config', path + '/config/arm.yaml')
        self.actualPosition = {}
        self.positions = {}
        self.times = {}
        self.iniTime = rospy.Time.now()
        with open(self.yaml) as file:
            documents = yaml.safe_load(file)
            for item, doc in documents.items():
                if (float(doc["Protocol"]) == 2):
                    self.times[item] = (doc["Profile_Acceleration"]* 214.577, doc["Profile_Velocity"]*0.229)
                    self.actualPosition[item] = 0
                    self.positions[item] = 0
        rospy.Subscriber("/fenrir/joint_commands", JointState, self.jointCallback)
        self.pubJointPosition = rospy.Publisher('/fenrir/joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(30)

    def jointCallback(self, data):
        printC(INFO, "Reciving position command!")
        for i, name in enumerate(data.name):
            self.positions[name] = data.position[i]
        self.iniTime = rospy.Time.now()

    def updatePosition(self, name):
        t1 = 64*self.times[name][1]/self.times[name][0]
        while not rospy.is_shutdown():
            t2 = 64*abs(self.positions[name] - self.actualPosition[name])/self.times[name][1]
            t3 = (t1 + t2)*1000000000
            time = abs(rospy.Time.now().nsecs - self.iniTime.nsecs)
            self.actualPosition[name] = (1/(1 + np.exp(-4*time/t3)))*self.positions[name]

    def run(self):
        printC(INFO, "Initing simulated Fenrir")
        for name in self.times:
            thread = threading.Thread(target=self.updatePosition, args=(name,))
            thread.start()
        while not rospy.is_shutdown():
            self.state = JointState()
            for name in self.actualPosition:
                self.state.name.append(name)
                self.state.position.append(self.actualPosition[name])
            self.state.header.stamp = rospy.Time.now()
            self.pubJointPosition.publish(self.state)
            self.rate.sleep()


if __name__ == "__main__":
    drvS = dynamixel_driver_sim()
    drvS.run()
