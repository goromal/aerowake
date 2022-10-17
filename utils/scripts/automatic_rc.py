#!/usr/bin/python

import rospy, subprocess, yaml, time
from rosflight_msgs.msg import RCRaw
import time

if __name__ == '__main__':

    # ENUMS
    HIGH = 2000
    MED  = 1500
    LOW  = 1000

    # initialize
    rospy.init_node('rc_joy')
    rc_pub = rospy.Publisher('RC', RCRaw, queue_size=10)
    rospy.loginfo("USING AUTOMATIC RC ARM FUNCTIONALITY")

    # GET ROSFLIGHT SWITCH CHANNELS
    RC_THR_CHN = None #2-
    RC_OVD_CHN = None #4-
    RC_ARM_CHN = None #5-
    rfpfile = rospy.get_param('~rosflight_paramfile')
    with open(rfpfile) as f:
        data = yaml.load(f, Loader=yaml.Loader)
        for datum in data:
            if datum['name'] == 'RC_F_CHN':
                RC_THR_CHN = datum['value']
            if datum['name'] == 'RC_THR_OVRD_CHN':
                RC_OVD_CHN = datum['value']
            if datum['name'] == 'ARM_CHANNEL':
                RC_ARM_CHN = datum['value']

    sleep_time = 0.05
    sleep_rate = 1.0 / sleep_time
    rate = rospy.Rate(sleep_rate)
    rateQsec = rospy.Rate(4)
    rate2sec = rospy.Rate(0.5)
    wait_secs = int(rospy.get_param('~wait_secs', 1))

    msg = RCRaw()
    # Values should range from 1000 - 2000
    rospy.loginfo('SETTING ARM AND OVERRIDE CHANNELS TO HIGH')
    msg.values[0] = MED
    msg.values[1] = MED
    msg.values[RC_THR_CHN] = LOW
    msg.values[3] = MED
    msg.values[RC_OVD_CHN] = LOW
    msg.values[RC_ARM_CHN] = HIGH
    msg.values[6] = MED
    msg.values[7] = MED

    idx = 0
    uninitiated = True
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        time_i = idx * sleep_time

        if time_i >= wait_secs and uninitiated:

            rc_pub.publish(msg)
            # rateQsec.sleep()
            # rateQsec.sleep()

            # rate2sec.sleep()
            # time.sleep(2.0)

            rospy.loginfo('SETTING THROTTLE HIGH')
            msg.values[RC_THR_CHN] = HIGH
            uninitiated = False

        rc_pub.publish(msg)
        rate.sleep()
        idx += 1
