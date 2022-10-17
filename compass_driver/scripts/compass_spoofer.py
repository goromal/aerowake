#!/usr/bin/python

import rospy, math
import numpy as np
from geometry_msgs.msg import PoseStamped
import tf.transformations as TR
from sensor_msgs.msg import MagneticField

def sat(val, limit):
    rval = val
    if val > limit:
        rval = limit
    if val < -limit:
        rval = -limit
    return rval

class Spoofer(object):
    def __init__(self):
        self.inc = rospy.get_param('~inclination')
        self.dec = rospy.get_param('~declination')
        self.stdev = rospy.get_param('~mag_stdev')
        self.bias_rng = rospy.get_param('~mag_bias_range')
        self.bias_wlk_stdev = rospy.get_param('~mag_bias_walk_stdev')

        self.magf_I = np.array([np.cos(self.inc)*np.cos(self.dec),
                                np.cos(self.inc)*np.sin(self.dec),
                                np.sin(self.inc)])

        self.mag_walk_x = 0.0
        self.mag_walk_y = 0.0
        self.mag_walk_z = 0.0

        self.q_XOAT_X = (0.0, 0.0, 0.0, 1.0)
        self.R_X_NED = np.array([[1.0,  0.0,  0.0],
                                 [0.0, -1.0,  0.0],
                                 [0.0,  0.0, -1.0]])
        self.R_XOAT_BOAT = np.array([[1.0, 0.0, 0.0],
                                     [0.0, 1.0, 0.0],
                                     [0.0, 0.0, 1.0]])

        self.possub = rospy.Subscriber("mocap", PoseStamped, self.callback, queue_size=1)

        self.magpub = rospy.Publisher("mag", MagneticField, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1.0/2.0), self.update)

    def callback(self, msg):
        # Just store the quaternion
        o_XOAT_X = msg.pose.orientation
        self.q_XOAT_X = (o_XOAT_X.x, o_XOAT_X.y, o_XOAT_X.z, o_XOAT_X.w)

    def update(self, event):
        # Perform necessary coordinate transforms to obtain NED -> SHIP
        R_XOAT_X = TR.quaternion_matrix(self.q_XOAT_X)[0:3,0:3]
        R_XOAT_NED = np.dot(self.R_X_NED, R_XOAT_X)
        R_NED_BOAT = np.dot(self.R_XOAT_BOAT, np.linalg.inv(R_XOAT_NED))
        magf_BOAT = np.dot(R_NED_BOAT, self.magf_I)

        self.mag_walk_x  = sat(self.mag_walk_x + np.random.normal(0.0, self.bias_wlk_stdev), self.bias_rng)
        self.mag_walk_y  = sat(self.mag_walk_y + np.random.normal(0.0, self.bias_wlk_stdev), self.bias_rng)
        self.mag_walk_z  = sat(self.mag_walk_z + np.random.normal(0.0, self.bias_wlk_stdev), self.bias_rng)
        mag_x = magf_BOAT[0] + self.mag_walk_x + np.random.normal(0.0, self.stdev)
        mag_y = magf_BOAT[0] + self.mag_walk_y + np.random.normal(0.0, self.stdev)
        mag_z = magf_BOAT[0] + self.mag_walk_z + np.random.normal(0.0, self.stdev)

        magmsg = MagneticField()
        magmsg.header.stamp = rospy.Time.now()
        magmsg.magnetic_field.x = mag_x
        magmsg.magnetic_field.y = mag_y
        magmsg.magnetic_field.z = mag_z
        magmsg.magnetic_field_covariance[0] = self.stdev**2
        magmsg.magnetic_field_covariance[1] = 0.0
        magmsg.magnetic_field_covariance[2] = 0.0
        magmsg.magnetic_field_covariance[3] = 0.0
        magmsg.magnetic_field_covariance[4] = self.stdev**2
        magmsg.magnetic_field_covariance[5] = 0.0
        magmsg.magnetic_field_covariance[6] = 0.0
        magmsg.magnetic_field_covariance[7] = 0.0
        magmsg.magnetic_field_covariance[8] = self.stdev**2
        self.magpub.publish(magmsg)

def main():
    '''Initialize and cleanup ros node'''
    rospy.init_node('mag_spoofer', anonymous=True)
    s = Spoofer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS mag spoofer node."

if __name__ == '__main__':
    main()
