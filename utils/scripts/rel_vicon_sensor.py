#!/usr/bin/python

import rospy
import numpy as np
from numpy.linalg import inv
from math import pi
import tf.transformations as TR
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

def homogeneous_matrix(trans, quat):
    return TR.concatenate_matrices(TR.translation_matrix(trans), TR.quaternion_matrix(quat))

def homogeneous_from_pose(pose):
    x, y, z = pose.position.x, pose.position.y, pose.position.z
    qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    return homogeneous_matrix((x, y, z), (qx, qy, qz, qw))

def pose_from_homogeneous(homogeneous):
    msg = Pose()
    t = TR.translation_from_matrix(homogeneous)
    q = TR.quaternion_from_matrix(homogeneous)
    msg.position.x = t[0]
    msg.position.y = t[1]
    msg.position.z = t[2]
    msg.orientation.x = q[0]
    msg.orientation.y = q[1]
    msg.orientation.z = q[2]
    msg.orientation.w = q[3]
    return msg

class ViconSensor:
    def __init__(self):
        self.b_vsub = rospy.Subscriber("vicon_base", PoseStamped, self.bCallback, queue_size=1)
        self.u_vsub = rospy.Subscriber("vicon_uav",  PoseStamped, self.uCallback, queue_size=1)
        self.reference_pub = rospy.Publisher("reference", Odometry, queue_size=1)
        self.T_UAV_XAV = TR.euler_matrix(pi, 0, 0, 'rxyz')
        self.T_XOAT_BOAT = TR.euler_matrix(pi, 0, 0, 'rxyz')
        self.T_XAV_X = None
        self.T_XOAT_X = None

    def bCallback(self, msg):
        self.T_XOAT_X = homogeneous_from_pose(msg.pose)

    def uCallback(self, msg):
        self.T_XAV_X = homogeneous_from_pose(msg.pose)
        if not self.T_XOAT_X is None:
            T_UAV_BOAT = TR.concatenate_matrices(self.T_XOAT_BOAT, np.linalg.inv(self.T_XOAT_X), self.T_XAV_X, self.T_UAV_XAV)
            ref_msg = Odometry()
            ref_msg.header.stamp = rospy.Time.now()
            ref_msg.pose.pose = pose_from_homogeneous(T_UAV_BOAT)
            self.reference_pub.publish(ref_msg)

def main():
    '''Initialize and cleanup ros node'''
    rospy.init_node('rel_vicon_sensor', anonymous=True)
    vs = ViconSensor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down relative ROS vicon sensor spoofer."

if __name__ == '__main__':
    main()
