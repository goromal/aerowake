#!/usr/bin/python
import rospy

from geometry_msgs.msg import Wrench

class WrenchAdder(object):
    def __init__(self):
        self.tethersub = rospy.Subscriber("tether_wrench", Wrench, self.tetherCallback)
        self.airsub = rospy.Subscriber("air_wrench", Wrench, self.airCallback)
        self.extpub = rospy.Publisher("uav_ext_wrench", Wrench, queue_size=1)

        self.tetherFx = 0.0
        self.tetherFy = 0.0
        self.tetherFz = 0.0
        self.tetherTx = 0.0
        self.tetherTy = 0.0
        self.tetherTz = 0.0

        self.airFx = 0.0
        self.airFy = 0.0
        self.airFz = 0.0
        self.airTx = 0.0
        self.airTy = 0.0
        self.airTz = 0.0

        self.update_timer_ = rospy.Timer(rospy.Duration(1.0/50.0), self.update)

    def tetherCallback(self, msg):
        self.tetherFx = msg.force.x
        self.tetherFy = msg.force.y
        self.tetherFz = msg.force.z
        self.tetherTx = msg.torque.x
        self.tetherTy = msg.torque.y
        self.tetherTz = msg.torque.z

    def airCallback(self, msg):
        self.airFx = msg.force.x
        self.airFy = msg.force.y
        self.airFz = msg.force.z
        self.airTx = msg.torque.x
        self.airTy = msg.torque.y
        self.airTz = msg.torque.z

    def update(self, event):
        wrench = Wrench()
        wrench.force.x = self.tetherFx + self.airFx
        wrench.force.y = self.tetherFy + self.airFy
        wrench.force.z = self.tetherFz + self.airFz
        wrench.torque.x = self.tetherTx + self.airTx
        wrench.torque.y = self.tetherTy + self.airTy
        wrench.torque.z = self.tetherTz + self.airTz
        self.extpub.publish(wrench)

if __name__ == "__main__":
    rospy.init_node('wrench_adder', anonymous=True)
    WA = WrenchAdder()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print 'Shutting down wrench adder Node.'
