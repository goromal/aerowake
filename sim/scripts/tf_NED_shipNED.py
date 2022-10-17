#!/usr/bin/python

"""
Use initial relative NED and vision data to spoof the following:
    - tf from NED to ship frame (static, defined according
      to determined initial relative NED and ship heading)
"""

import rospy, tf, tf2_ros
import numpy as np
from math import pi, sqrt, atan2
from geometry_msgs.msg import TransformStamped
from ublox_msgs.msg import NavRELPOSNED
from vision.msg import VisionPose
import tf.transformations as TR

def homogeneous_matrix(trans, quat):
    return TR.concatenate_matrices(TR.translation_matrix(trans), TR.quaternion_matrix(quat))

def quat_from_two_vectors(u_, v_):
    qx = 0.0
    qy = 0.0
    qz = 0.0
    qw = 1.0
    u = u_ / np.linalg.norm(u_)
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

def yaw_from_quat(q):
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]
    return atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))

relNED = None
relNEDsub = None
visPos = None
visPossub = None

# TODO: MAKE SURE THESE ARE TIME-SYNCED

def visPosCallback(msg):
    global visPos
    global visPossub
    visPos = msg
    visPossub.unregister()

def relNEDCallback(msg):
    global relNED
    global relNEDsub
    relNED = msg
    relNEDsub.unregister()

if __name__ == "__main__":
    global relNED
    global relNEDsub
    global visPos
    global visPossub

    rospy.init_node('tf_NED_shipNED')

    relNEDsub = rospy.Subscriber("relative_gps", NavRELPOSNED, relNEDCallback, queue_size=1)
    visPossub = rospy.Subscriber("vision_pose", VisionPose, visPosCallback, queue_size=1)

    p_ship2bgps_SHIP = np.array(rospy.get_param('~p_SHIP_BGPS'))
    p_b2g = rospy.get_param('~p_b2g')
    p_b2g_UAV = np.array([p_b2g[0], p_b2g[1], p_b2g[2], 1.0])

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "NED"
    static_transformStamped.child_frame_id = "shipNED"

    while True:
        if (not relNED is None) and (not visPos is None):
            q_UAV_SHIP = visPos.transform.rotation
            t_ship2uav_SHIP = visPos.transform.translation
            p_ship2uav_SHIP = np.array([t_ship2uav_SHIP.x,
                                        t_ship2uav_SHIP.y,
                                        t_ship2uav_SHIP.z])
            R_UAV_SHIP = TR.quaternion_matrix((q_UAV_SHIP.x,
                                               q_UAV_SHIP.y,
                                               q_UAV_SHIP.z,
                                               q_UAV_SHIP.w))
            p_b2g_SHIP = np.dot(R_UAV_SHIP, p_b2g_UAV)[0:3]
            p_bgps2gps_SHIP = p_ship2uav_SHIP - p_ship2bgps_SHIP + p_b2g_SHIP
            N = 1.0e-2*relNED.relPosN + 1.0e-4*relNED.relPosHPN
            E = 1.0e-2*relNED.relPosE + 1.0e-4*relNED.relPosHPE
            D = 1.0e-2*relNED.relPosD + 1.0e-4*relNED.relPosHPD
            p_bgps2gps_NED = np.array([N, E, D])
            rospy.loginfo("[TF SPOOFER] INITIAL VECTORS:\nVIS:\t%f\t%f\t%f\nGPS:\t%f\t%f\t%f" % \
                          (p_bgps2gps_SHIP[0], p_bgps2gps_SHIP[1], p_bgps2gps_SHIP[2],
                           p_bgps2gps_NED[0], p_bgps2gps_NED[1], p_bgps2gps_NED[2]))
            q_SHIP_NED = quat_from_two_vectors([-p_bgps2gps_SHIP[0], -p_bgps2gps_SHIP[1], 0.0],
                                               [-p_bgps2gps_NED[0], -p_bgps2gps_NED[1], 0.0])
            yaw = yaw_from_quat(q_SHIP_NED)
            rospy.loginfo("[TF SPOOFER] YAW FROM NED TO SHIP: %f" % yaw)

            R_SHIP_NED = TR.quaternion_matrix(q_SHIP_NED)
            p_ship2uav_NED = np.dot(R_SHIP_NED[0:3,0:3], p_ship2uav_SHIP)

            static_transformStamped.transform.translation.x = -p_ship2uav_NED[0]
            static_transformStamped.transform.translation.y = -p_ship2uav_NED[1]
            static_transformStamped.transform.translation.z = -p_ship2uav_NED[2]

            static_transformStamped.transform.rotation.x = q_SHIP_NED[0]
            static_transformStamped.transform.rotation.y = q_SHIP_NED[1]
            static_transformStamped.transform.rotation.z = q_SHIP_NED[2]
            static_transformStamped.transform.rotation.w = q_SHIP_NED[3]

            break

    broadcaster.sendTransform(static_transformStamped)
    rospy.spin()
