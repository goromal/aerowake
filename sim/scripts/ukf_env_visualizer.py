#!/usr/bin/python

"""
Spoof the following:
    - tf from NED to UAV frame (from subscribed odometry)
    - UAV marker (from odometry)
    - UKF estimated external force
"""

import rospy, tf
import numpy as np
from math import sqrt, pi
from geometry_msgs.msg import Pose, WrenchStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
import tf.transformations as TR

def homogeneous_matrix(trans, quat):
    return TR.concatenate_matrices(TR.translation_matrix(trans), TR.quaternion_matrix(quat))

def quat_from_two_vectors(u_, v_):
    qx = 0.0
    qy = 0.0
    qz = 0.0
    qw = 1.0
    # u = np.array(u_)
    u = u_ / np.linalg.norm(u_)
    # v = np.array(v_)
    v = v_ / np.linalg.norm(v_)

    d = np.dot(u, v)
    if d < 0.99999999 and d > -0.99999999:
        invs = 1.0/sqrt(2.0*(1.0+d))
        xyz = np.cross(u, v*invs)
        qw = 0.5/invs
        qx = xyz[0]
        qy = xyz[1]
        qz = xyz[2]
    elif d < -0.99999999: # infinite number of solutions; picking one
        qw = 0.0
        qx = 1.0
        qy = 0.0
        qz = 0.0
    # else keep identity rotation

    q_norm = np.linalg.norm(np.array([qx, qy, qz, qw]))
    return (qx/q_norm, qy/q_norm, qz/q_norm, qw/q_norm)

class EnvironmentSpoofer(object):
    def __init__(self):
        self.tf_BR             = tf.TransformBroadcaster()

        uavscale = 1.0
        self.uavMarkerPub = rospy.Publisher("uav_marker", Marker, queue_size=1)
        self.uavMarker = Marker()
        self.uavMarker.header.frame_id = "NED"
        self.uavMarker.header.stamp = rospy.Time.now()
        self.uavMarker.id = 1
        self.uavMarker.type = Marker.MESH_RESOURCE
        self.uavMarker.action = Marker.ADD
        self.uavMarker.mesh_resource = "package://uav_dynamics/mesh/quadrotor_bodyaxes.dae"
        self.uavMarker.scale.x = uavscale
        self.uavMarker.scale.y = uavscale
        self.uavMarker.scale.z = uavscale
        self.uavMarker.color.b = 1.0
        self.uavMarker.color.g = 1.0
        self.uavMarker.color.r = 1.0
        self.uavMarker.color.a = 1.0

        arrow_scale = 0.025
        self.fextMarkerPub = rospy.Publisher("fext_marker", Marker, queue_size=1)
        self.fextMarker = Marker()
        self.fextMarker.header.frame_id = "NED"
        self.fextMarker.id = 3
        self.fextMarker.type = Marker.ARROW
        self.fextMarker.action = Marker.ADD
        self.fextMarker.pose = Pose()
        self.fextMarker.pose.position.x = 0.0
        self.fextMarker.pose.position.y = 0.0
        self.fextMarker.pose.position.z = -0.1
        self.fextMarker.scale.x = arrow_scale
        self.fextMarker.scale.y = arrow_scale
        self.fextMarker.scale.z = arrow_scale
        self.fextMarker.color.r = 0.0
        self.fextMarker.color.g = 1.0
        self.fextMarker.color.b = 1.0
        self.fextMarker.color.a = 1.0

        self.textMarkerPub = rospy.Publisher("text_marker", Marker, queue_size=1)
        self.textMarker = Marker()
        self.textMarker.header.frame_id = "UAV"
        self.textMarker.id = 3
        self.textMarker.type = Marker.ARROW
        self.textMarker.action = Marker.ADD
        self.textMarker.pose = Pose()
        self.textMarker.pose.position.x = 0.0
        self.textMarker.pose.position.y = 0.0
        self.textMarker.pose.position.z = -0.1
        self.textMarker.scale.x = arrow_scale
        self.textMarker.scale.y = arrow_scale
        self.textMarker.scale.z = arrow_scale
        self.textMarker.color.r = 1.0
        self.textMarker.color.g = 1.0
        self.textMarker.color.b = 0.0
        self.textMarker.color.a = 1.0

        self.Fext_x = 0
        self.Fext_y = 0
        self.Fext_z = 0
        self.Text_z = 0

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odomCallback)
        self.Fext_sub = rospy.Subscriber("fext", WrenchStamped, self.wrenchCallback)

    def wrenchCallback(self, msg):
        # expressed in NED frame
        self.Fext_x = msg.wrench.force.x
        self.Fext_y = -msg.wrench.force.y
        self.Fext_z = -msg.wrench.force.z
        # expressed in the UAV frame
        self.Text_z = -msg.wrench.torque.z

    def odomCallback(self, msg):
        # tf from NED to UAV
        self.tf_BR.sendTransform((msg.pose.pose.position.x,
                                  msg.pose.pose.position.y,
                                  msg.pose.pose.position.z),
                                 (msg.pose.pose.orientation.x,
                                  msg.pose.pose.orientation.y,
                                  msg.pose.pose.orientation.z,
                                  msg.pose.pose.orientation.w),
                                 rospy.Time.now(), "UAV", "NED")

        # UAV marker
        self.uavMarker.header.stamp = rospy.Time.now()
        self.uavMarker.pose = msg.pose.pose
        self.uavMarkerPub.publish(self.uavMarker)

        # Fext marker
        F = np.array([self.Fext_x, self.Fext_y, self.Fext_z])
        self.fextMarker.scale.x = np.linalg.norm(F) / 10.0
        q_fext = quat_from_two_vectors(np.array([1.0,0.0,0.0]), F)
        self.fextMarker.pose.position.x = msg.pose.pose.position.x
        self.fextMarker.pose.position.y = msg.pose.pose.position.y
        self.fextMarker.pose.position.z = msg.pose.pose.position.z
        self.fextMarker.pose.orientation.x = q_fext[0]
        self.fextMarker.pose.orientation.y = q_fext[1]
        self.fextMarker.pose.orientation.z = q_fext[2]
        self.fextMarker.pose.orientation.w = q_fext[3]
        self.fextMarker.header.stamp = rospy.Time.now()
        self.fextMarkerPub.publish(self.fextMarker)

        # Text marker
        T = np.array([0.0, 0.0, self.Text_z])
        self.textMarker.scale.x = np.linalg.norm(T) / 10.0
        q_text = quat_from_two_vectors(np.array([1.0,0.0,0.0]), T)
        self.textMarker.pose.orientation.x = q_text[0]
        self.textMarker.pose.orientation.y = q_text[1]
        self.textMarker.pose.orientation.z = q_text[2]
        self.textMarker.pose.orientation.w = q_text[3]
        self.textMarker.header.stamp = rospy.Time.now()
        self.textMarkerPub.publish(self.textMarker)

def main():
    rospy.init_node("env_spoofer", anonymous=True)
    es = EnvironmentSpoofer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down environment spoofer node")

if __name__ == "__main__":
    main()
