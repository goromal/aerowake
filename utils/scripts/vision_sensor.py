#!/usr/bin/python

import rospy
import numpy as np
from numpy.linalg import inv
from math import pi
import tf.transformations as TR
from vision.msg import VisionPose
from nav_msgs.msg import Odometry

def homogeneous_matrix(trans, quat):
    return TR.concatenate_matrices(TR.translation_matrix(trans), TR.quaternion_matrix(quat))

class VisionSensor:
    def __init__(self):
        self.subscriber = rospy.Subscriber("vision_pose", VisionPose, self.callback,
            queue_size=1)
        self.reference_pub = rospy.Publisher("reference", Odometry, queue_size=1)
        self.probationary = False

    def callback(self, msg):
        # only publish if vision data valid, secondary guard against outliers
        if msg.sol_status == VisionPose.NO_SOLUTION or msg.sol_status == VisionPose.DERIVED_INIT:
            self.probationary = True
        else:
            if not self.probationary:
                if msg.dynamically_valid and not msg.outlier:
                    # publish odometry
                    ref_msg = Odometry()
                    ref_msg.header.stamp = rospy.Time.now()
                    ref_msg.pose.pose.position.x = msg.transform.translation.x
                    ref_msg.pose.pose.position.y = msg.transform.translation.y
                    ref_msg.pose.pose.position.z = msg.transform.translation.z
                    ref_msg.pose.pose.orientation = msg.transform.rotation
                    self.reference_pub.publish(ref_msg)
            else:
                self.probationary = False

def main():
    '''Initialize and cleanup ros node'''
    rospy.init_node('vision_sensor', anonymous=True)
    vs = VisionSensor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS vision sensor spoofer."

if __name__ == '__main__':
    main()
