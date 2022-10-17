#!/usr/bin/python
import rospy, re
from flight.srv import TetherLength, TetherLengthResponse
from std_msgs.msg import Float64

from flow_sockets import FLOWClient
import asyncore, threading

class TetherReelController(object):
    def __init__(self): # if sim, publish to tether_dynamics; if not, use controller thread
        self.sim = rospy.get_param('~sim', False)
        self.length = rospy.get_param('~tether_takeoff_length', 0.0)
        self.host = rospy.get_param('~host_ip', '192.168.0.89') # default from RAVEN_24_AEROWAKE
        self.port = 4004

        if self.sim:
            self.pub = rospy.Publisher('sim_tether_limit', Float64, queue_size=1)

        self.tltS = rospy.Service('tether_length_tether', TetherLength, self.TLTCallback)

    def TLTCallback(self, req):
        matchstr = re.findall(r'(\+|-)?((\d+(\.\d+))|(\d+))?',req.tether_length_string)[0][0:2]
        if matchstr[0] == '+':
            self.length += float(matchstr[1])
        elif matchstr[0] == '-':
            self.length -= float(matchstr[1])
        else:
            self.length = float(matchstr[1])
        lengthstr = '{0:07.3f}'.format(self.length)
        FLOWClient(self.host, self.port, lengthstr)
        if self.sim:
            comm = Float64()
            comm.data = self.length
            self.pub.publish(comm)

        asyncore.loop(count=1)

        return TetherLengthResponse(True)

    def shutdown(self):
        pass

if __name__ == "__main__":
    rospy.init_node("tether_reel_controller", anonymous=True)
    # asyncore_thread = threading.Thread(target=asyncore.loop, kwargs={'timeout':1})
    TRC = TetherReelController()
    # asyncore_thread.start()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down tether reel controller Node."
        TRC.shutdown()
        # asyncore_thread.join()
