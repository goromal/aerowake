#!/usr/bin/python

import rospy
import numpy as np
from numpy.linalg import inv
from math import pi
import tf.transformations as TR
from geometry_msgs.msg import PoseStamped, TwistStamped
from geometry_msgs.msg import Quaternion

def homogeneous_matrix(trans, quat):
    return TR.concatenate_matrices(TR.translation_matrix(trans), TR.quaternion_matrix(quat))

class ViconSensor:
    def __init__(self):
        self.pose_sub = rospy.Subscriber("vicon_pose_NWU", PoseStamped, self.poseCallback, queue_size=1)
        self.twist_sub = rospy.Subscriber("vicon_twist_NWU", TwistStamped, self.twistCallback, queue_size=1)
        self.pose_pub = rospy.Publisher("vicon_pose_NED", PoseStamped, queue_size=1)
        self.twist_pub = rospy.Publisher("vicon_twist_NED", TwistStamped, queue_size=1)

        self.pub_att = rospy.get_param("~publish_attitude_correction", False)
        if self.pub_att:
            self.ext_att_pub = rospy.Publisher("external_attitude", Quaternion, queue_size=1)

        self.T_V_NED = TR.euler_matrix(pi, 0, 0, 'rxyz')
        self.T_UAV_XAV = TR.euler_matrix(pi, 0, 0, 'rxyz')

    def poseCallback(self, msg):
        # construct transform from vicon to pseudo-UAV frame, considering that
        # the transform is given using the robotics convention: T_XAV_V
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        qx, qy, qz, qw = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        T_XAV_V = homogeneous_matrix((x, y, z), (qx, qy, qz, qw))
        # perform required coordinate frame transformations
        T_UAV_NED = TR.concatenate_matrices(self.T_V_NED, T_XAV_V, self.T_UAV_XAV)
        # publish odometry
        ref_msg = PoseStamped()
        ref_msg.header.stamp = rospy.Time.now()
        q = TR.quaternion_from_matrix(T_UAV_NED)
        t = TR.translation_from_matrix(T_UAV_NED)
        ref_msg.pose.position.x = t[0]
        ref_msg.pose.position.y = t[1]
        ref_msg.pose.position.z = t[2]
        ref_msg.pose.orientation.x = q[0]
        ref_msg.pose.orientation.y = q[1]
        ref_msg.pose.orientation.z = q[2]
        ref_msg.pose.orientation.w = q[3]
        self.pose_pub.publish(ref_msg)

        if self.pub_att:
            att_msg = Quaternion()
            att_msg.x = q[0]
            att_msg.y = q[1]
            att_msg.z = q[2]
            att_msg.w = q[3]
            self.ext_att_pub.publish(att_msg)

    def twistCallback(self, msg):
        vx, vy, vz = msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z
        wx, wy, wz = msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z
        v = np.array([vx, vy, vz])
        w = np.array([wx, wy, wz])
        vp = self.T_V_NED[0:3,0:3].dot(v)
        wp = self.T_V_NED[0:3,0:3].dot(w)
        ref_msg = TwistStamped()
        ref_msg.header.stamp = rospy.Time.now()
        ref_msg.twist.linear.x = vp[0]
        ref_msg.twist.linear.y = vp[1]
        ref_msg.twist.linear.z = vp[2]
        ref_msg.twist.angular.x = wp[0]
        ref_msg.twist.angular.y = wp[1]
        ref_msg.twist.angular.z = wp[2]
        self.twist_pub.publish(ref_msg)

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
