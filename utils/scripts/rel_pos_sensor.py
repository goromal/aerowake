#!/usr/bin/python

import rospy
import numpy as np
from numpy.linalg import inv
from math import pi
import tf.transformations as TR
from ublox_msgs.msg import NavRELPOSNED
from vision.msg import VisionPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32

class RelPosSensor:
    def __init__(self):
        # self.psiSub = rospy.Subscriber("ship_yaw", Float32, self.yawCallback, queue_size=1)
        self.subscriber = rospy.Subscriber("rover/navrelposned", NavRELPOSNED, self.callback,
            queue_size=1)
        # self.R_UAV_SHIP = None
        # self.p_b2g_UAV = rospy.get_param('~p_b2g')
        # self.vpSub = rospy.Subscriber("vision_pose", VisionPose, self.vpCallback, queue_size=1)
        self.publisher = rospy.Publisher("relative_position", PoseWithCovarianceStamped, queue_size=1)
        # self.R_NED_SHIP = None

    # def vpCallback(self, msg):
    #     q_UAV_SHIP = msg.transform.rotation
    #     self.R_UAV_SHIP = TR.quaternion_matrix((q_UAV_SHIP.x,
    #                                             q_UAV_SHIP.y,
    #                                             q_UAV_SHIP.z,
    #                                             q_UAV_SHIP.w))[0:3,0:3]

    def yawCallback(self, msg):
        psi = msg.data # gives q_SHIP_NED
        q_NED_SHIP = TR.quaternion_from_euler(0, 0, -psi)
        self.R_NED_SHIP = TR.quaternion_matrix(q_NED_SHIP)[0:3,0:3]

    def callback(self, msg):
        # if not self.R_NED_SHIP is None:
        N = 1.0e-2*msg.relPosN + 1.0e-4*msg.relPosHPN
        E = 1.0e-2*msg.relPosE + 1.0e-4*msg.relPosHPE
        D = 1.0e-2*msg.relPosD + 1.0e-4*msg.relPosHPD
        p_NED = np.array([N, E, D])
        cov_NED = 1.0e-4*np.array([msg.accN, msg.accE, msg.accD])
            # p_SHIP = np.dot(self.R_NED_SHIP, p_NED)

            # if not self.R_UAV_SHIP is None:
            #     p_SHIP -= np.dot(self.R_UAV_SHIP, self.p_b2g_UAV)

            # cov_SHIP = np.dot(self.R_NED_SHIP, cov_NED)
        output = PoseWithCovarianceStamped()
        output.header.stamp = rospy.Time.now()
            # output.pose.pose.position.x = p_SHIP[0]
            # output.pose.pose.position.y = p_SHIP[1]
            # output.pose.pose.position.z = p_SHIP[2]
            # output.pose.covariance[0] = cov_SHIP[0]
            # output.pose.covariance[1] = cov_SHIP[1]
            # output.pose.covariance[2] = cov_SHIP[2]
        output.pose.pose.position.x = p_NED[0]
        output.pose.pose.position.y = p_NED[1]
        output.pose.pose.position.z = p_NED[2]
        output.pose.covariance[0] = cov_NED[0]
        output.pose.covariance[1] = cov_NED[1]
        output.pose.covariance[2] = cov_NED[2]
        self.publisher.publish(output)

def main():
    '''Initialize and cleanup ros node'''
    rospy.init_node('rel_pos_sensor', anonymous=True)
    vs = RelPosSensor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS relative position sensor."

if __name__ == '__main__':
    main()
