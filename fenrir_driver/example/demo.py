#!/usr/bin/env python2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np
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

def joint_publisher():
    pub = rospy.Publisher('/joint_commands/', JointState, queue_size=0)
    rospy.init_node('joint_publisher_example', anonymous=False)
    rate = rospy.Rate(0.3) # 10hz
    offset = [-0.2646766535897931, -0.060606653589793336, 0.1388593464102068, 0.08898034641020706, -0.14740565358979296]
    start =   [0.0 + offset[0],   0.0 + offset[1], 0.0 + offset[2], 0.0 + offset[3],  0.0 + offset[4]]
    home = [0.0 + offset[0],   0.6 + offset[1], -2.1 + offset[2], 0.0 + offset[3],  0. + offset[4]]
    grasp0 = [0.0 + offset[0],   0.6 + offset[1], -2.1 + offset[2], 0.0 + offset[3],  -0.7 + offset[4]]
    grasp1 = [0.0 + offset[0],   0.6 + offset[1], -2.1 + offset[2], 0.0 + offset[3],  -0.86 + offset[4]]
    # grasp0 = [0.0 + offset[0], 0.0 + offset[1], 1.0 + offset[2], 0.5 + offset[3], -0.44+ offset[4]]
    # grasp1 = [0.0 + offset[0],   0.0 + offset[1], 0.0+ offset[2], 0.0 + offset[3], -0.94+ offset[4]]
    # grasp2 = [1.5 + offset[0], 0.0 + offset[1], 0.0+ offset[2], 0.0 + offset[3], -0.94+ offset[4]]    
    actions = {'0':['start', start], '1':['home', home], '2':['grasp', grasp0], '3':['grasp1', grasp1]}#, '4':['grasp2', grasp2]}
    action_index = 0

    while not rospy.is_shutdown():
        rospy.loginfo("Select an action by typing from 0-{}: ".format(len(actions)-1))
        for action in actions:
            rospy.loginfo("- Type {} to go to {}.".format(action, actions[action][0]))
        action_index = getch()
        rospy.loginfo("Setting {} position.".format(actions[action_index][0]))

        state = JointState()
        # state.name = [ "Base", "Shoulder", "Shoulder_shadow", "Elbow", "Wrist", "Gripper" ]
        state.name = [ "base_joint", "shoulder_joint", "elbow_joint", "wrist_joint", "gripper_joint" ]

        # state.header.stamp = rospy.Time.now()
        # state.position = [0.52, 0.52, 0.52, 0]
        # pub.publish(state)
        # print 'published command'
        # rate.sleep()

        state.header.stamp = rospy.Time.now()
        state.position = actions[action_index][1]
        pub.publish(state)
        rate.sleep()

if __name__ == '__main__':
    try:
        joint_publisher()
    except rospy.ROSInterruptException:
        pass