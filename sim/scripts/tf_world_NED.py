#!/usr/bin/python

"""
Spoof the following:
    - tf from world to NED frame (static)
"""

import rospy, tf, tf2_ros
import numpy as np
from math import pi
from geometry_msgs.msg import TransformStamped
import tf.transformations as TR

def homogeneous_matrix(trans, quat):
    return TR.concatenate_matrices(TR.translation_matrix(trans), TR.quaternion_matrix(quat))

if __name__ == "__main__":
    rospy.init_node('tf_world_NED')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "world"
    static_transformStamped.child_frame_id = "NED"

    H_NED_world = homogeneous_matrix([0.0, 0.0, 0.0],
              TR.quaternion_from_euler(pi, 0.0, 0.0))
    t_world_NED = TR.translation_from_matrix(H_NED_world)
    q_world_NED = TR.quaternion_from_matrix(H_NED_world)

    static_transformStamped.transform.translation.x = t_world_NED[0]
    static_transformStamped.transform.translation.y = t_world_NED[1]
    static_transformStamped.transform.translation.z = t_world_NED[2]

    static_transformStamped.transform.rotation.x = q_world_NED[0]
    static_transformStamped.transform.rotation.y = q_world_NED[1]
    static_transformStamped.transform.rotation.z = q_world_NED[2]
    static_transformStamped.transform.rotation.w = q_world_NED[3]

    broadcaster.sendTransform(static_transformStamped)
    rospy.spin()
