#!/usr/bin/python

import rospy
from rosflight_msgs.msg import RCRaw

class RCFixer(object):
    def __init__(self):
        self.rc_sub = rospy.Subscriber("input_RC", RCRaw, self.rcCallback)
        self.rc_pub = rospy.Publisher("output_RC", RCRaw, queue_size=1)

    def rcCallback(self, msg):
        out = RCRaw()
        out.header = msg.header
        for i in range(0,8):
            out.values[i] = msg.values[i] + 1000
        self.rc_pub.publish(out)

def main():
    rospy.init_node("rc_signal_fixer", anonymous=True)
    rf = RCFixer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down RC signal fixer node")

if __name__ == "__main__":
    main()
