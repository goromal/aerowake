#!/usr/bin/python

import rospy
from nav_msgs.msg import Odometry

if __name__ == '__main__':

    # initialize
    rospy.init_node('identity_pose_pub')
    pub = rospy.Publisher('reference', Odometry, queue_size=1)

    # populate fields
    msg = Odometry()
    msg.pose.pose.orientation.w = 1.0

    # main loop
    update_freq = 200 # Hz
    rate = rospy.Rate(update_freq)

    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()
