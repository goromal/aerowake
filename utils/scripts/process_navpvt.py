#!/usr/bin/python

import rospy
from ublox_msgs.msg import NavPVT
from rosflight_msgs.msg import GNSS
from math import cos, sin, pi, sqrt
import numpy as np
import tf.transformations as TR

class AbsGPSConverter(object):
    def __init__(self):
        self.subscriber = rospy.Subscriber("ublox_abs", NavPVT, self.callback, queue_size=1)
        self.publisher = rospy.Publisher("rf_gps_abs", GNSS, queue_size=1)

        self.A = 6378137.0 # WGS-84 Earth semimajor axis (m)
        self.B = 6356752.314245 # Derived Earth semiminor axis (m)
        self.F = (self.A - self.B) / self.A # Ellipsoid Flatness

        self.lat0 = None
        self.lon0 = None

    def lla_to_ecef(self, lat, lon, alt):
        sinp = sin(lat)
        cosp = cos(lat)
        sinl = sin(lon)
        cosl = cos(lon)
        e2 = self.F*(2.0-self.F)
        v = self.A/sqrt(1.0-e2*sinp*sinp)
        ecef = [(v+alt)*cosp*cosl, (v+alt)*cosp*sinl, (v*(1.0-e2)+alt)*sinp]
        return ecef

    def ned_to_ecef(self, n, e, d): # rotation only, for velocity vector
        x = -((-4*n*(abs(cos(0.785398 + 0.5*self.lat0))**2 + abs(sin(0.785398 + 0.5*self.lat0))**2)*(abs(cos(0.5*self.lon0))**2 + \
            abs(sin(0.5*self.lon0))**2) + 8*n*cos(0.5*self.lon0)**2*sin(0.785398 + 0.5*self.lat0)**2 + \
            8*e*cos(0.785398 + 0.5*self.lat0)**2*cos(0.5*self.lon0)*sin(0.5*self.lon0) + \
            4*(2*n*cos(0.785398 + 0.5*self.lat0)**2 - d*sin(1.5708 + self.lat0))*sin(0.5*self.lon0)**2 + \
            4*e*sin(0.785398 + 0.5*self.lat0)**2*sin(self.lon0) + \
            d/sin(0.5*self.lon0)**2*sin(1.5708 + self.lat0)*sin(self.lon0)**2)/(4*(abs(cos(0.785398 + 0.5*self.lat0))**2 + \
            abs(sin(0.785398 + 0.5*self.lat0))**2)*(abs(cos(0.5*self.lon0))**2 + abs(sin(0.5*self.lon0))**2)))
        y = (-2*e - d*cos(1.5708 + self.lat0 - self.lon0) + d*cos(1.5708 + self.lat0 + self.lon0) + \
            e*(2*cos(self.lon0) + cos(self.lat0 + self.lon0 - self.lat0 - self.lon0) + cos(self.lat0 - self.lon0 - self.lat0 + self.lon0)) - \
            n*sin(1.5708 + self.lat0 - self.lon0) + n*sin(1.5708 + self.lat0 + self.lon0))/(2*(abs(cos(0.785398 + 0.5*self.lat0))**2 + \
            abs(sin(0.785398 + 0.5*self.lat0))**2)*(abs(cos(0.5*self.lon0))**2 + abs(sin(0.5*self.lon0))**2))
        z = (-d + n*cos(self.lat0) + 0.5*d*cos(self.lat0 + self.lon0 - self.lat0 - self.lon0) + 0.5*d*cos(self.lat0 - self.lon0 - self.lat0 + self.lon0) - \
            d*sin(self.lat0))/((abs(cos(0.785398 + 0.5*self.lat0))**2 + abs(sin(0.785398 + 0.5*self.lat0))**2)*(abs(cos(0.5*self.lon0))**2 + \
            abs(sin(0.5*self.lon0))**2))
        return [x, y, z]

    def callback(self, msg):
        output = GNSS()
        output.fix = msg.fixType
        output.time = rospy.Time.now() # TODO: match with original message (not super high priority...)
        lat = self.intDegToRad(msg.lat)
        lon = self.intDegToRad(msg.lon)
        alt = 1.0e-3*msg.height
        if self.lat0 is None:
            self.lat0 = lat
            self.lon0 = lon
        ecef = self.lla_to_ecef(lat, lon, alt)
        output.position[0] = ecef[0]
        output.position[1] = ecef[1]
        output.position[2] = ecef[2]
        output.horizontal_accuracy = 1.0e-3*msg.hAcc
        output.vertical_accuracy = 1.0e-3*msg.vAcc
        vel_ecef = self.ned_to_ecef(1.0e-3*msg.velN, 1.0e-3*msg.velE, 1.0e-3*msg.velD)
        output.velocity[0] = vel_ecef[0]
        output.velocity[1] = vel_ecef[1]
        output.velocity[2] = vel_ecef[2]
        output.speed_accuracy = 1.0e-3*msg.sAcc
        self.publisher.publish(output)

    def intDegToRad(self, intdeg):
        return pi * 1.0e-7 * intdeg / 180.0

def main():
    '''Initialize and cleanup ros node'''
    rospy.init_node('abs_gps_conv', anonymous=True)
    agp = AbsGPSConverter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down absolute GPS converter node."

if __name__ == '__main__':
    main()
