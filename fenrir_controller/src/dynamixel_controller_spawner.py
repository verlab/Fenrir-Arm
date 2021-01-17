#!/usr/bin/env python
import sys
import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)

from trajectory_msgs.msg import  JointTrajectory, JointTrajectoryPoint

class JointTrajectoryActionServer(object):

    def __init__(self, controller_name):
        self.dx_trajectory_pub = rospy.Publisher ("/joint_trajectory", JointTrajectory, queue_size=10)  
        self._action_ns = controller_name + '/follow_joint_trajectory'
        rospy.loginfo('Starting controller: {}'.format(self._action_ns))
        self._as = actionlib.SimpleActionServer(
                self._action_ns,
                FollowJointTrajectoryAction,
                execute_cb=self.execute_cb,
                auto_start = False)
        self._action_name = rospy.get_name()
        self._as.start()
        rospy.loginfo('Controller {} has been successfuly started.'.format(controller_name))
        self._feedback = FollowJointTrajectoryFeedback
        self._result = FollowJointTrajectoryResult

        
    def execute_cb(self, goal):
        rospy.loginfo('Receving controller trajectories...')
        jt = JointTrajectory()
        jt.header.stamp = rospy.Time.now()
        jt.joint_names = goal.trajectory.joint_names
        jt.points = []
        for point in goal.trajectory.points:
            jtp = JointTrajectoryPoint()
            jtp.positions = point.positions
            jtp.velocities = point.velocities
            jtp.accelerations = point.accelerations
            jtp.effort = point.effort
            jtp.time_from_start = point.time_from_start
            jt.points.append(jtp)
        self.dx_trajectory_pub.publish(jt)
        result_ = FollowJointTrajectoryResult()
        self._as.set_succeeded(result_)
        rospy.loginfo('Joint trajectories has been successfuly transmitted to dynamixel branch.')


if __name__ == '__main__':
    rospy.init_node('fenrir_controller_interface')
    rospy.loginfo("Starting fenrir trajectory controller interface.")
    if len(sys.argv) < 2:
        rospy.loginfo("Please! Set a controller_name using args.")
        exit()
    controller_name = sys.argv[1]
    server = JointTrajectoryActionServer(controller_name)
    # servers = []
    # for controller_name in sys.argv[2:]:
        
    #     servers.append(server)
    rospy.spin()
