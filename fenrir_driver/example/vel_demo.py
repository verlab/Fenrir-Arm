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
    rate = rospy.Rate(1.0) # 10hz
    step = 0.08
    iterations = 10
    i = 0

    while not rospy.is_shutdown():
        if i >= iterations:
            _ref.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # elbow
            _ref.header.stamp = rospy.Time.now()
            pub.publish(_ref)
            rate.sleep()
            break
        if (_ref == []):
            rate.sleep()
            continue
        
        # Compute next position
        if (i%2):
            _ref.velocity =  [0.3, 0.3, 0.3, 0.3, 0.3, 0.3] # elbow
        else:
            _ref.velocity = [-0.3, -0.3, -0.3, -0.3, -0.3, -0.3] # elbow
        i += 1.0
        # print('Setting elbow velocity to: {} rad/s'.format(offset))
        # _ref.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, offset] # elbow
        # _ref.velocity = [0.0, 0.0, 0.0, 0.0, offset, offset] # elbow
        _ref.header.stamp = rospy.Time.now()
        pub.publish(_ref)

        # Loop control
        rate.sleep()

if __name__ == '__main__':
    try:
        joint_publisher()
    except rospy.ROSInterruptException:
        pass