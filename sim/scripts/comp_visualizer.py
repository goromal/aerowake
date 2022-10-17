#!/usr/bin/python

"""
Spoof the following:
    - tf from NED to UAV frame (from subscribed odometry)
    - UAV marker (from odometry)
"""

import rospy, tf
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

class EnvironmentSpoofer(object):
    def __init__(self):
        self.tf_BR             = tf.TransformBroadcaster()

        uavscale = 1.0
        self.refuavMarkerPub = rospy.Publisher("ref_uav_marker", Marker, queue_size=1)
        self.refuavMarker = Marker()
        self.refuavMarker.header.frame_id = "NED"
        self.refuavMarker.header.stamp = rospy.Time.now()
        self.refuavMarker.id = 2
        self.refuavMarker.type = Marker.MESH_RESOURCE
        self.refuavMarker.action = Marker.ADD
        self.refuavMarker.mesh_resource = "package://uav_dynamics/mesh/quadrotor_bodyaxes.dae"
        self.refuavMarker.scale.x = uavscale
        self.refuavMarker.scale.y = uavscale
        self.refuavMarker.scale.z = uavscale
        self.refuavMarker.color.b = 0.0
        self.refuavMarker.color.g = 1.0
        self.refuavMarker.color.r = 1.0
        self.refuavMarker.color.a = 1.0

        self.ref_odom_sub = rospy.Subscriber("ref_odom", Odometry, self.refodomCallback)

    def refodomCallback(self, msg):
        self.tf_BR.sendTransform((msg.pose.pose.position.x,
                                  msg.pose.pose.position.y,
                                  msg.pose.pose.position.z),
                                 (msg.pose.pose.orientation.x,
                                  msg.pose.pose.orientation.y,
                                  msg.pose.pose.orientation.z,
                                  msg.pose.pose.orientation.w),
                                 rospy.Time.now(), "refUAV", "NED")

        self.refuavMarker.header.stamp = rospy.Time.now()
        self.refuavMarker.pose = msg.pose.pose
        self.refuavMarkerPub.publish(self.refuavMarker)

def main():
    rospy.init_node("comp_visualizer", anonymous=True)
    es = EnvironmentSpoofer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down environment spoofer node")

if __name__ == "__main__":
    main()
