#!/usr/bin/python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class OdomConverter:
    def __init__(self):
        self.subscriber = rospy.Subscriber("input_odom", Odometry, self.callback,
            queue_size=1)
        self.publisher = rospy.Publisher("output_pose", PoseWithCovarianceStamped, queue_size=1)

    def callback(self, msg):
        output = PoseWithCovarianceStamped()
        output.header.stamp = rospy.Time.now()
        output.pose = msg.pose
        self.publisher.publish(output)

def main():
    '''Initialize and cleanup ros node'''
    rospy.init_node('odom2pose', anonymous=True)
    vs = OdomConverter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS odometry -> pose converter node."

if __name__ == '__main__':
    main()
