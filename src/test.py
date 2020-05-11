#!/usr/bin/env python2
import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import time


def joint_publisher():
    pub = rospy.Publisher('/dynamixel/joint_trajectory/', JointTrajectory, queue_size=0)
    rospy.init_node('joint_publisher', anonymous=False)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        p1, p2 = JointTrajectoryPoint(), JointTrajectoryPoint()
        p1.positions = [120]
        #p2.positions = [220]
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ["Wrist"]
        trajectory_msg.points = [p1]

        pub.publish(trajectory_msg)
        print 'published command'
        rate.sleep()

        # p1, p2 = JointTrajectoryPoint(), JointTrajectoryPoint()
        # p1.positions = [230]
        # p2.positions = [120]
        # trajectory_msg = JointTrajectory()
        # trajectory_msg.joint_names = ["Elbow", 'Shoulder']
        # trajectory_msg.points = [p1, p2]

        # pub.publish(trajectory_msg)
        # print 'published command'
        # rate.sleep()

if __name__ == '__main__':
    try:
        joint_publisher()
    except rospy.ROSInterruptException:
        pass
