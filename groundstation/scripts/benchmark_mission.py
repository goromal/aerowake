#!/usr/bin/python

import rospy
from flight.srv import MissionCommand, UavAltitude, TetherLength

if __name__ == "__main__":

    # initialize
    rospy.init_node("benchmark_mission")
    MC = rospy.ServiceProxy('mission_command', MissionCommand)
    UA = rospy.ServiceProxy('uav_altitude', UavAltitude)
    TL = rospy.ServiceProxy('tether_length', TetherLength)
    oneSecond = rospy.Rate(1.0)
    rospy.loginfo("[BENCHMARK MISSION] Mission control initialized.")

    # useful service proxy function
    def send_command(service_proxy, comm_name, comm_str):
        rospy.loginfo("[BENCHMARK MISSION] Commanding {}.".format(comm_name))
        try:
            resp = service_proxy(comm_str)
            if not resp.success:
                rospy.logerr("[BENCHMARK MISSION] {} failed.".format(comm_name))
                return False
        except rospy.ServiceException as exc:
            rospy.logerr("[BENCHMARK MISSION] {} failed: {}".format(comm_name, str(exc)))
            return False
        return True

    # ==========================================================================

    # wait for RC initialization
    for i in range(12): oneSecond.sleep()

    # command takeoff
    send_command(MC, 'Takeoff', 'takeoff')

    for i in range(3): oneSecond.sleep()

    send_command(TL, "Tether Length = 20", '20.0')

    for i in range(35): oneSecond.sleep()

    while not send_command(MC, 'Center', 'center'):
        for i in range(2): oneSecond.sleep()

    for i in range(4): oneSecond.sleep()

    while not send_command(MC, 'Sweep', 'sweep'):
        for i in range(2): oneSecond.sleep()
