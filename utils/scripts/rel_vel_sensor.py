#!/usr/bin/python

import rospy
import numpy as np
from numpy.linalg import inv
from math import pi
import tf.transformations as TR
from ublox_msgs.msg import NavVELNED
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Float32

class RelVelSensor:
    def __init__(self):
        # self.psiSub = rospy.Subscriber("ship_yaw", Float32, self.yawCallback, queue_size=1)
        self.subscriber = rospy.Subscriber("rover/navvelned", NavVELNED, self.callback,
            queue_size=1)
        self.publisher = rospy.Publisher("relative_velocity", TwistWithCovarianceStamped, queue_size=1)
        # self.R_NED_SHIP = None

    # def yawCallback(self, msg):
    #     psi = msg.data # gives q_SHIP_NED
    #     q_NED_SHIP = TR.quaternion_from_euler(0, 0, -psi)
    #     self.R_NED_SHIP = TR.quaternion_matrix(q_NED_SHIP)[0:3,0:3]

    def callback(self, msg):
        # if not self.R_NED_SHIP is None:
            # TODO: take into account p_b2g and body omega...
        v_NED = 1.0e-2*np.array([msg.velN, msg.velE, msg.velD])
            # v_SHIP = np.dot(self.R_NED_SHIP, v_NED)
        sACC = 1.0e-2*msg.sAcc
        output = TwistWithCovarianceStamped()
        output.header.stamp = rospy.Time.now()
        output.twist.twist.linear.x = v_NED[0]
        output.twist.twist.linear.y = v_NED[1]
        output.twist.twist.linear.z = v_NED[2]
        output.twist.covariance[0] = sACC
        self.publisher.publish(output)

def main():
    '''Initialize and cleanup ros node'''
    rospy.init_node('rel_vel_sensor', anonymous=True)
    vs = RelVelSensor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS relative velocity sensor."

if __name__ == '__main__':
    main()
