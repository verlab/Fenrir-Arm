#!/usr/bin/env python2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

_ref = []
once = True


def state_cb(data):
    global _ref, once
    if once:
        print("Getting reference position!")
        _ref = data
        once = False

def joint_publisher():
    global _ref, once
    pub = rospy.Publisher('/fenrir/joint_commands', JointState, queue_size=0)
    sub = rospy.Subscriber('/fenrir/joint_states', JointState, state_cb)
    rospy.init_node('joint_publisher', anonymous=False)
    rate = rospy.Rate(0.05) # 10hz
    step = 0.01
    iterations = 2
    i = 0

    while not rospy.is_shutdown():
        if i >= iterations:
            break
        if (_ref == []):
            rate.sleep()
            continue
        
        # Compute next position
        i += 1.0
        offset = 0.0
        if (i%2):
            offset = 0.01
        else:
            offset = 0.0
        # _ref.position = [ref_pos[0] + offset, ref_pos[1] + offset, _ref.position[2], _ref.position[3], _ref.position[4], _ref.position[5]]
        print('Setting elbow velocity to: {} rad/s'.format(offset))
        # _ref.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, offset] # elbow
        _ref.velocity = [0.0, 0.0, 0.0, 0.0, offset, 0.0] # elbow
        _ref.header.stamp = rospy.Time.now()
        pub.publish(_ref)

        # Loop control
        rate.sleep()

if __name__ == '__main__':
    try:
        joint_publisher()
    except rospy.ROSInterruptException:
        pass