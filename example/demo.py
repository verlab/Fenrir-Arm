#!/usr/bin/env python2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState


def joint_publisher():
    pub = rospy.Publisher('/fenrir/joint_commands/', JointState, queue_size=0)
    rospy.init_node('joint_publisher', anonymous=False)
    rate = rospy.Rate(0.3) # 10hz
    home = [3.14, 3.6778498611256336, 3.676315505739265, 1.155369605935587, 3.2296268563296833, 3.1214]
    grasp = [3.14, 2.3613729396213388+0.5, 2.3629072950077075+0.5, 1.2211333676406078, 3.2296268563296833, 2.2]

    while not rospy.is_shutdown():
        state = JointState()
        state.name = [ "Base", "Shoulder", "Shoulder_shadow", "Elbow", "Wrist", "Gripper" ]

        # state.header.stamp = rospy.Time.now()
        # state.position = [0.52, 0.52, 0.52, 0]
        # pub.publish(state)
        # print 'published command'
        # rate.sleep()

        state.header.stamp = rospy.Time.now()
        state.position = home
        pub.publish(state)
        print 'published command'
        rate.sleep()

        state.header.stamp = rospy.Time.now()
        # state.position = [ 3.14, 4.0]
        state.position = grasp
        pub.publish(state)
        print 'published command'
        rate.sleep()

if __name__ == '__main__':
    try:
        joint_publisher()
    except rospy.ROSInterruptException:
        pass