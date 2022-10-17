#!/usr/bin/python

import rospy
import numpy as np
from numpy.linalg import inv
from math import pi
import tf.transformations as TR
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

def homogeneous_matrix(trans, quat):
    return TR.concatenate_matrices(TR.translation_matrix(trans), TR.quaternion_matrix(quat))

class OdomConverter:
    def __init__(self):
        self.subscriber = rospy.Subscriber("input_odom", Odometry, self.callback,
            queue_size=1)
        self.publisher = rospy.Publisher("output_odom", Odometry, queue_size=1)
        self.T_NED_V = TR.euler_matrix(pi, 0, 0, 'rxyz')
        self.T_XAV_UAV = TR.euler_matrix(pi, 0, 0, 'rxyz')
        self.T_XAV_V = TR.identity_matrix() # the output transform

    def callback(self, msg):
        # construct transform from NED to UAV frame, considering that
        # the transform message gives the transform in robotics convention: T_UAV_NED
        x, y, z = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        T_UAV_NED = homogeneous_matrix((x, y, z), (qx, qy, qz, qw))
        # perform required coordinate frame transformation
        self.T_XAV_V = TR.concatenate_matrices(self.T_NED_V, T_UAV_NED, self.T_XAV_UAV)
        # publish odometry
        output = Odometry()
        output.header.stamp = rospy.Time.now()
        q = TR.quaternion_from_matrix(self.T_XAV_V)
        t = TR.translation_from_matrix(self.T_XAV_V)
        output.pose.pose.position.x = t[0]
        output.pose.pose.position.y = t[1]
        output.pose.pose.position.z = t[2]
        output.pose.pose.orientation.x = q[0]
        output.pose.pose.orientation.y = q[1]
        output.pose.pose.orientation.z = q[2]
        output.pose.pose.orientation.w = q[3]
        self.publisher.publish(output)

def main():
    '''Initialize and cleanup ros node'''
    rospy.init_node('odom_ned2nwu', anonymous=True)
    vs = OdomConverter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS odometry NED -> NWU converter node."

if __name__ == '__main__':
    main()
