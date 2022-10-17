#!/usr/bin/python

import rospy
import numpy as np
from numpy.linalg import inv
from math import pi
import tf.transformations as TR
from acl_msgs.msg import ViconState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

def homogeneous_matrix(trans, quat):
    return TR.concatenate_matrices(TR.translation_matrix(trans), TR.quaternion_matrix(quat))

class ViconSensor:
    def __init__(self):
        self.subscriber = rospy.Subscriber("vicon", ViconState, self.callback,
            queue_size=1)
        self.reference_pub = rospy.Publisher("reference", Odometry, queue_size=1)
        self.ext_att_pub = rospy.Publisher("external_attitude", Quaternion, queue_size=1)
        self.T_V_NED = TR.euler_matrix(pi, 0, 0, 'rxyz')
        # self.T_XAV_V = TR.identity_matrix() # from subscriber callback
        self.T_UAV_XAV = TR.euler_matrix(pi, 0, 0, 'rxyz')
        self.T_UAV_NED = TR.identity_matrix() # the output transform

    def callback(self, msg):
        # construct transform from vicon to pseudo-UAV frame, considering that
        # the transform is given using the robotics convention: T_XAV_V
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        qx, qy, qz, qw = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        T_XAV_V = homogeneous_matrix((x, y, z), (qx, qy, qz, qw))
        # perform required coordinate frame transformations
        self.T_UAV_NED = TR.concatenate_matrices(self.T_V_NED, T_XAV_V, self.T_UAV_XAV)
        # publish odometry
        ref_msg = Odometry()
        ref_msg.header.stamp = rospy.Time.now()
        q = TR.quaternion_from_matrix(self.T_UAV_NED)
        t = TR.translation_from_matrix(self.T_UAV_NED)
        ref_msg.pose.pose.position.x = t[0]
        ref_msg.pose.pose.position.y = t[1]
        ref_msg.pose.pose.position.z = t[2]
        ref_msg.pose.pose.orientation.x = q[0]
        ref_msg.pose.pose.orientation.y = q[1]
        ref_msg.pose.pose.orientation.z = q[2]
        ref_msg.pose.pose.orientation.w = q[3]
        att_msg = Quaternion()
        att_msg.x = q[0]
        att_msg.y = q[1]
        att_msg.z = q[2]
        att_msg.w = q[3]
        self.reference_pub.publish(ref_msg)
        self.ext_att_pub.publish(att_msg)

def main():
    '''Initialize and cleanup ros node'''
    rospy.init_node('vicon_sensor', anonymous=True)
    vs = ViconSensor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS vicon sensor spoofer."

if __name__ == '__main__':
    main()
