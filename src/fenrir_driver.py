#!/usr/bin/env python2

from sensor_msgs.msg import JointState
from colors import *
from threading import Lock
from dynamixel_sdk.packet_handler import *
from dynamixel_sdk.port_handler import *
from Dynamixel import Dynamixel
from fenrir.msg import MotorInfo
import serial
import rospkg
import rospy
import yaml
import sys
import os


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
        path = rospack.get_path('fenrir')

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
            printC(INFO, "Open port successful")
        except serial.serialutil.SerialException:
            printC(INFO, "Failed to open port %s" % self.port)
            getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(self.baudrate):
            printC(INFO, "Succeeded to set the baudrate")
        else:
            printC(INFO, "Failed to change the baudrate")
            printC(KEY)
            getch()
            quit()

        # Instanciate the subscriber and publisher
        rospy.Subscriber("/fenrir/joint_commands", JointState, self.jointCallback)
        self.pubJointPosition = rospy.Publisher('/fenrir/joint_states', JointState, queue_size=1)
        self.pubJointInfo = rospy.Publisher('/fenrir/joint_info', MotorInfo, queue_size=1)

    def __del__(self):
        try:
            self.setTorque (state=False)
        except AttributeError:
            pass

    def jointCallback(self, data):
        self.write.acquire()
        for i, name in enumerate(data.name):
            mode = self.dynamixels[name].read("Operating_Mode")
            self.dynamixels[name].writeAngle(data.position[i])
        self.write.release()

    def setDynamixels (self):
        self.dynamixels = {}
        with open(self.yaml) as file:
            printC(INFO, "list of motors: ")
            documents = yaml.safe_load(file)
            for item, doc in documents.items():
                self.dynamixels[item] = Dynamixel(
                    self.portHandler, doc["ID"], float(doc["Protocol"]))
                for flag, value in doc.items():
                    if (flag != "ID" and flag != "Protocol"):
                        self.dynamixels[item].write(flag, value)

    def updatePosition (self):
        # Updating position limit
        printC(INFO, "Updating positions")
        for dyn in self.dynamixels:
            self.dynamixels[dyn].updateMaxPositions()

    def setTorque (self, state=True):
        # Updating torque of motors
        for key in self.dynamixels:
            self.dynamixels[key].write("Torque_Enabled", state*1)
        if state:
            printC(INFO, "Enabling motors torque")
        else:
            printC(INFO, "Desabling motors torque")

    #TODO verify the difference here from workbench
    def verifyMoving (self, event=None):
        for key in self.dynamixels:
            test = self.dynamixels[key].reachedGoal()
            if not test:
                self.is_moving = True
                return
        self.is_moving = False

    def run(self):
        self.setDynamixels()
        self.updatePosition()
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
                angles.append(self.dynamixels[dyn].readAngle()[0])
                efforts.append(self.dynamixels[dyn].readLoad()[0])
                velocities.append(self.dynamixels[dyn].readVelocity()[0])

                # information about status
                currents.append(self.dynamixels[dyn].readCurrent()[0])
                voltages.append(self.dynamixels[dyn].readVoltage()[0])
                temperatures.append(self.dynamixels[dyn].readTemp()[0])

            self.write.release()
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
    drv = dynamixel_driver()
    drv.run()
