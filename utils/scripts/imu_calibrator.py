#!/usr/bin/python

import rospy, subprocess
import time

if __name__ == '__main__':

    # initialize
    rospy.init_node('imu_calibrator')
    rospy.loginfo("USING AUTOMATIC IMU CALIBRATOR FOR SIM TESTING")

    wait_secs = rospy.get_param('~wait_secs', 1)
    wait_rate = 1.0 / wait_secs
    rate = rospy.Rate(wait_rate)
    rate.sleep()

    rospy.loginfo('ATTEMPTING TO CALIBRATE IMU AFTER %f SECONDS' % wait_secs)
    subprocess.call(['rosservice','call','/calibrate_imu'])

    rate4secs = rospy.Rate(0.25)
    rate4secs.sleep()
