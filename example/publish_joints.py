#!/usr/bin/env python2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState


def joint_publisher():
    pub = rospy.Publisher('/fenrir/joint_commands/', JointState, queue_size=0)
    rospy.init_node('joint_publisher', anonymous=False)
    rate = rospy.Rate(0.2) # 10hz
    while not rospy.is_shutdown():
        state = JointState()
        state.name = ["Base", "Shoulder", "Elbow", "Wrist", "Gripper"]

        # state.header.stamp = rospy.Time.now()
        # state.position = [0.52, 0.52, 0.52, 0]
        # pub.publish(state)
        # print 'published command'
        # rate.sleep()

        state.header.stamp = rospy.Time.now()
        state.position = [0.5, 0.5, 0.5, 0.5, -0.8]
        pub.publish(state)
        print 'published command'
        rate.sleep()

        state.header.stamp = rospy.Time.now()
        state.position = [0, 0, 0, 0, 0]
        pub.publish(state)
        print 'published command'
        rate.sleep()

if __name__ == '__main__':
    try:
        joint_publisher()
    except rospy.ROSInterruptException:
        pass