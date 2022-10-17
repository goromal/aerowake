#!/usr/bin/python
import rospy
from math import pi
from rosflight_msgs.msg import OutputRaw
from geom_inertia_estimator.msg import MotorRPM

class MotorSpeedCalculator(object):
    def __init__(self):
        self.signalSub = rospy.Subscriber('signals', OutputRaw, self.signalCallback)
        self.speedPub = rospy.Publisher('motor_speeds', MotorRPM, queue_size=1)
        self.kop = rospy.get_param('~motor/k_omega_poly')

    def signalCallback(self, msg):
        actuators_msg = MotorRPM()
        actuators_msg.header = msg.header
        actuators_msg.rpm = [0, 0, 0, 0]
        for i in range(4):
            s = msg.values[i]
            w = self.kop[0] * s + self.kop[1] # calculate motor speed
            _w = int(w*60.0/(2*pi)) # convert from rad/s to RPM
            actuators_msg.rpm[i] = _w
        self.speedPub.publish(actuators_msg)

if __name__ == '__main__':
    rospy.init_node('motor_speed_calculator', anonymous=True)
    MSC = MotorSpeedCalculator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print 'Shutting down motor speed calculator node.'
