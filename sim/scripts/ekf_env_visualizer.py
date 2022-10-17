#!/usr/bin/python

"""
Spoof the following:
    - tf from NED to UAV frame (from subscribed odometry)
    - UAV marker (from odometry)
    - UAV velocity vector (from odometry)
    - beacon markers (from yaml)
    - [other arrows?]
"""

import rospy, tf
import numpy as np
from math import sqrt, pi
from geometry_msgs.msg import TransformStamped, Pose, Point
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
        # self.tf_world_NED_BC   = tf2_ros.StaticTransformBroadcaster()
        # self.tf_NED_shipNED_BC = tf2_ros.StaticTransformBroadcaster()
        # self.sendStaticTransforms()

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

        self.reluavMarkerPub = rospy.Publisher("rel_uav_marker", Marker, queue_size=1)
        self.reluavMarker = Marker()
        self.reluavMarker.header.frame_id = "shipNED"
        self.reluavMarker.header.stamp = rospy.Time.now()
        self.reluavMarker.id = 2
        self.reluavMarker.type = Marker.MESH_RESOURCE
        self.reluavMarker.action = Marker.ADD
        self.reluavMarker.mesh_resource = "package://uav_dynamics/mesh/quadrotor_bodyaxes.dae"
        self.reluavMarker.scale.x = uavscale
        self.reluavMarker.scale.y = uavscale
        self.reluavMarker.scale.z = uavscale
        self.reluavMarker.color.b = 0.0
        self.reluavMarker.color.g = 1.0
        self.reluavMarker.color.r = 1.0
        self.reluavMarker.color.a = 1.0

        arrow_scale = 0.025
        self.velMarkerPub = rospy.Publisher("vel_marker", Marker, queue_size=1)
        self.velMarker = Marker()
        self.velMarker.header.frame_id = "UAV"
        self.velMarker.id = 3
        self.velMarker.type = Marker.ARROW
        self.velMarker.action = Marker.ADD
        self.velMarker.pose = Pose()
        self.velMarker.pose.position.x = 0.0
        self.velMarker.pose.position.y = 0.0
        self.velMarker.pose.position.z = -0.1
        self.velMarker.scale.x = arrow_scale
        self.velMarker.scale.y = arrow_scale
        self.velMarker.scale.z = arrow_scale
        self.velMarker.color.r = 0.0
        self.velMarker.color.g = 1.0
        self.velMarker.color.b = 1.0
        self.velMarker.color.a = 1.0

        self.relvelMarkerPub = rospy.Publisher("rel_vel_marker", Marker, queue_size=1)
        self.relvelMarker = Marker()
        self.relvelMarker.header.frame_id = "relUAV"
        self.relvelMarker.id = 4
        self.relvelMarker.type = Marker.ARROW
        self.relvelMarker.action = Marker.ADD
        self.relvelMarker.pose = Pose()
        self.relvelMarker.pose.position.x = 0.0
        self.relvelMarker.pose.position.y = 0.0
        self.relvelMarker.pose.position.z = -0.1
        self.relvelMarker.scale.x = arrow_scale
        self.relvelMarker.scale.y = arrow_scale
        self.relvelMarker.scale.z = arrow_scale
        self.relvelMarker.color.r = 0.0
        self.relvelMarker.color.g = 1.0
        self.relvelMarker.color.b = 1.0
        self.relvelMarker.color.a = 1.0

        vec_X_shipNED_beacon = rospy.get_param("~X_ship_beacon")
        H_beacon_shipNED = homogeneous_matrix([vec_X_shipNED_beacon[0],
                                               vec_X_shipNED_beacon[1],
                                               vec_X_shipNED_beacon[2]],
                      TR.quaternion_from_euler(vec_X_shipNED_beacon[3],
                                               vec_X_shipNED_beacon[4],
                                               vec_X_shipNED_beacon[5]))

        self.bcnMarkerPub = rospy.Publisher("bcn_marker", Marker, queue_size=1)
        self.bcnMarker = Marker()
        self.bcnMarker.header.frame_id = "shipNED"
        self.bcnMarker.id = 5
        self.bcnMarker.type = Marker.POINTS
        self.bcnMarker.action = Marker.ADD
        self.bcnMarker.pose.orientation.x = 0.0
        self.bcnMarker.pose.orientation.y = 0.0
        self.bcnMarker.pose.orientation.z = 0.0
        self.bcnMarker.pose.orientation.w = 1.0
        points_beacon = rospy.get_param("~beacon_positions")
        for i in range(0, 8):
            point_beacon_x = points_beacon[3 * i + 0]
            point_beacon_y = points_beacon[3 * i + 1]
            point_beacon_z = points_beacon[3 * i + 2]
            point_shipNED = np.dot(H_beacon_shipNED, np.array([point_beacon_x,
                                                               point_beacon_y,
                                                               point_beacon_z,
                                                               1.0]))
            p = Point()
            p.x = point_shipNED[0]
            p.y = point_shipNED[1]
            p.z = point_shipNED[2]
            self.bcnMarker.points.append(p)
        bcnscale = 0.05
        self.bcnMarker.color.r = 0.0
        self.bcnMarker.color.g = 1.0
        self.bcnMarker.color.b = 0.0
        self.bcnMarker.color.a = 1.0
        self.bcnMarker.scale.x = bcnscale
        self.bcnMarker.scale.y = bcnscale

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odomCallback)
        self.rel_odom_sub = rospy.Subscriber("rel_odom", Odometry, self.relodomCallback)

    # def sendStaticTransforms(self):
    #     H_NED_world = homogeneous_matrix([0.0, 0.0, 0.0],
    #               TR.quaternion_from_euler(pi, 0.0, 0.0))
    #     tf_world_NED = TransformStamped()
    #     t_world_NED = TR.translation_from_matrix(H_NED_world)
    #     q_world_NED = TR.quaternion_from_matrix(H_NED_world)
    #     tf_world_NED.header.stamp = rospy.Time.now()
    #     tf_world_NED.header.frame_id = "world"
    #     tf_world_NED.child_frame_id = "NED"
    #     tf_world_NED.transform.translation.x = t_world_NED[0]
    #     tf_world_NED.transform.translation.y = t_world_NED[1]
    #     tf_world_NED.transform.translation.z = t_world_NED[2]
    #     tf_world_NED.transform.rotation.x = q_world_NED[0]
    #     tf_world_NED.transform.rotation.y = q_world_NED[1]
    #     tf_world_NED.transform.rotation.z = q_world_NED[2]
    #     tf_world_NED.transform.rotation.w = q_world_NED[3]
    #     self.tf_world_NED_BC.sendTransform(tf_world_NED)
    #
    #     H_shipNED_NED = np.identity(4)
    #     tf_NED_shipNED = TransformStamped()
    #     t_NED_shipNED = TR.translation_from_matrix(H_shipNED_NED)
    #     q_NED_shipNED = TR.quaternion_from_matrix(H_shipNED_NED)
    #     tf_NED_shipNED.header.stamp = rospy.Time.now()
    #     tf_NED_shipNED.header.frame_id = "NED"
    #     tf_NED_shipNED.child_frame_id = "shipNED"
    #     tf_NED_shipNED.transform.translation.x = t_NED_shipNED[0]
    #     tf_NED_shipNED.transform.translation.y = t_NED_shipNED[1]
    #     tf_NED_shipNED.transform.translation.z = t_NED_shipNED[2]
    #     tf_NED_shipNED.transform.rotation.x = q_NED_shipNED[0]
    #     tf_NED_shipNED.transform.rotation.y = q_NED_shipNED[1]
    #     tf_NED_shipNED.transform.rotation.z = q_NED_shipNED[2]
    #     tf_NED_shipNED.transform.rotation.w = q_NED_shipNED[3]
    #     self.tf_NED_shipNED_BC.sendTransform(tf_NED_shipNED)

    def relodomCallback(self, msg):
        self.tf_BR.sendTransform((msg.pose.pose.position.x,
                                  msg.pose.pose.position.y,
                                  msg.pose.pose.position.z),
                                 (msg.pose.pose.orientation.x,
                                  msg.pose.pose.orientation.y,
                                  msg.pose.pose.orientation.z,
                                  msg.pose.pose.orientation.w),
                                 rospy.Time.now(), "relUAV", "shipNED")

        self.reluavMarker.header.stamp = rospy.Time.now()
        self.reluavMarker.pose = msg.pose.pose
        self.reluavMarkerPub.publish(self.reluavMarker)

        v = np.array([msg.twist.twist.linear.x,
                      msg.twist.twist.linear.y,
                      msg.twist.twist.linear.z])
        self.relvelMarker.scale.x = np.linalg.norm(v)
        q_arrow = quat_from_two_vectors(np.array([1.0,0.0,0.0]), v)
        self.relvelMarker.pose.orientation.x = q_arrow[0]
        self.relvelMarker.pose.orientation.y = q_arrow[1]
        self.relvelMarker.pose.orientation.z = q_arrow[2]
        self.relvelMarker.pose.orientation.w = q_arrow[3]
        self.relvelMarker.header.stamp = rospy.Time.now()
        self.relvelMarkerPub.publish(self.relvelMarker)

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

        # UAV velocity marker
        v = np.array([msg.twist.twist.linear.x,
                      msg.twist.twist.linear.y,
                      msg.twist.twist.linear.z])
        self.velMarker.scale.x = np.linalg.norm(v)
        q_arrow = quat_from_two_vectors(np.array([1.0,0.0,0.0]), v)
        self.velMarker.pose.orientation.x = q_arrow[0]
        self.velMarker.pose.orientation.y = q_arrow[1]
        self.velMarker.pose.orientation.z = q_arrow[2]
        self.velMarker.pose.orientation.w = q_arrow[3]
        self.velMarker.header.stamp = rospy.Time.now()
        self.velMarkerPub.publish(self.velMarker)

        # Beacon marker
        self.bcnMarker.header.stamp = rospy.Time.now()
        self.bcnMarkerPub.publish(self.bcnMarker)

def main():
    rospy.init_node("env_spoofer", anonymous=True)
    es = EnvironmentSpoofer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down environment spoofer node")

if __name__ == "__main__":
    main()
