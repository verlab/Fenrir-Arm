#!/usr/bin/python

import rospy
import numpy as np
import matplotlib.pyplot as plt
import sys
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import ChannelFloat32

excav_x = 1.58868741989
excav_y = 0.263290971518
_pub = None

def cb_shovel(msg):
	global excav_x, excav_y, _pub
	data = ChannelFloat32()
	data.name = "Dist, Angle"
	data.values = [np.sqrt((msg.x - excav_x)**2 + (msg.y - excav_y)**2), msg.theta]
	# print("Data: {}".format(data))
	if _pub:
		_pub.publish(data)


def run():
	global _pub
	rospy.init_node('shovel_att_publisher', anonymous=False)
	_pub = rospy.Publisher('/dist_angle_norm', ChannelFloat32, queue_size=0)
	sub = rospy.Subscriber('/shovel/ground_pose', Pose2D, cb_shovel)
	print("Starting")
	rospy.spin()


if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass
