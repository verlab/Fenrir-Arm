#!/usr/bin/env python2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

ref_pos = []
ref_vel = []
ref_effort = []
_pos = []
_vel = []
_effort = []
_ref = []
once = True


def state_cb(data):
    global ref_pos, ref_vel, ref_effort, once, _pos, _vel, _effort, _ref
    if once:
        print("Get init!")
        _ref = data
        ref_pos = [data.position[0], data.position[1]]
        ref_vel = [data.velocity[0], data.velocity[1]]
        ref_effort = [data.effort[0], data.effort[1]]
        # print('[{}] pos: {}, vel: {}, effort: {}'.format(0, ref_pos, ref_vel, ref_effort))
        once = False
    else:
        _pos = [data.position[0], data.position[1]]
        _vel = [data.velocity[0], data.velocity[1]]
        _effort = [data.effort[0], data.effort[1]]



def joint_publisher():
    global ref_pos, ref_vel, ref_effort, once, _pos, _vel, _effort, _ref
    pub = rospy.Publisher('/fenrir/joint_commands', JointState, queue_size=0)
    sub = rospy.Subscriber('/fenrir/joint_states', JointState, state_cb)
    rospy.init_node('joint_publisher', anonymous=False)
    rate = rospy.Rate(0.05) # 10hz
    step = -0.01
    iterations = 20
    i = 0

    while not rospy.is_shutdown():
        if i >= 2 * iterations:
            break
        if (ref_pos == []):
            rate.sleep()
            continue
        
        print('[{}] pos: {}, vel: {}, effort: {}'.format(i, _pos, _vel, _effort))

        # Compute next position
        offset = 0.0
        if i < iterations:
            offset = step*(i % iterations)
        else:
            offset = step*(iterations - 1.0 - (i % iterations))
        _ref.position = [ref_pos[0] + offset, ref_pos[1] + offset, _ref.position[2], _ref.position[3], _ref.position[4], _ref.position[5]]
        _ref.header.stamp = rospy.Time.now()
        pub.publish(_ref)

        i += 1.0
        # Loop control
        rate.sleep()

if __name__ == '__main__':
    try:
        joint_publisher()
    except rospy.ROSInterruptException:
        pass