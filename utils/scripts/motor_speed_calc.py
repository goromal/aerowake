#!/usr/bin/python
import rospy
from rosflight_msgs.msg import OutputRaw
from mav_msgs.msg import Actuators

class MotorSpeedCalculator(object):
    def __init__(self):
        self.signalSub = rospy.Subscriber('signals', OutputRaw, self.signalCallback)
        self.speedPub = rospy.Publisher('motor_speeds', Actuators, queue_size=1)
        self.num_rotors = rospy.get_param('~num_rotors')
        self.omega_poly_coeff = rospy.get_param('~omega_poly')
        self.omega_polys = []
        for i in range(self.num_rotors):
            self.omega_polys.append((self.omega_poly_coeff[2*i+0],\
                                     self.omega_poly_coeff[2*i+1]))

    def signalCallback(self, msg):
        actuators_msg = Actuators()
        actuators_msg.header = msg.header
        for i in range(self.num_rotors):
            s = msg.values[i]
            w = self.omega_polys[i][0] * s + self.omega_polys[i][1] # calculate motor speed
            actuators_msg.angular_velocities.append(max(w,0.0))
        self.speedPub.publish(actuators_msg)

if __name__ == '__main__':
    rospy.init_node('motor_speed_calculator', anonymous=True)
    MSC = MotorSpeedCalculator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print 'Shutting down motor speed calculator node.'
