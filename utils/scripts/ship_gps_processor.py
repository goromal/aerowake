#!/usr/bin/python

import rospy
from std_msgs.msg import Float32

# TODO: use ship GPS data and publish upon callback

if __name__ == '__main__':
    rospy.init_node('base_gps_processor', anonymous=True)

    rate = rospy.Rate(2.0)

    psi0 = rospy.get_param('~initial_ship_yaw')

    pub = rospy.Publisher('ship_yaw', Float32, queue_size=1)

    output = Float32()
    output.data = psi0

    while not rospy.is_shutdown():
        pub.publish(output)
        rate.sleep()
