import rospy
from flight.srv import MissionCommand, UavAltitude, TetherLength

class MissionComm(object):
    MISSION_COMMAND = 1
    ALTITUDE_COMMAND = 2
    TETHER_COMMAND = 3
    MC = rospy.ServiceProxy('mission_command', MissionCommand)
    UA = rospy.ServiceProxy('uav_altitude', UavAltitude)
    TL = rospy.ServiceProxy('tether_length_tether', TetherLength)
    # oneSecond = rospy.Rate(1.0)
    # def __init__(self):
    #     self.MC = rospy.ServiceProxy('mission_command', MissionCommand)
    #     self.UA = rospy.ServiceProxy('uav_altitude', UavAltitude)
    #     # self.TL = rospy.ServiceProxy('tether_length', TetherLength)
    #     self.TL = rospy.ServiceProxy('tether_length_tether', TetherLength)
    #     self.oneSecond = rospy.Rate(1.0)
    #     rospy.loginfo("[BENCHMARK MISSION] Mission control initialized.")

    @staticmethod
    def send_command(proxy_key, comm_name, comm_str):
        rospy.loginfo("[BENCHMARK MISSION] Commanding {}.".format(comm_name))
        try:
            if proxy_key == MissionComm.MISSION_COMMAND:
                resp = MissionComm.MC(comm_str)
            elif proxy_key == MissionComm.ALTITUDE_COMMAND:
                resp = MissionComm.UA(comm_str)
            elif proxy_key == MissionComm.TETHER_COMMAND:
                resp = MissionComm.TL(comm_str)
            if not resp.success:
                rospy.logerr("[BENCHMARK MISSION] {} failed.".format(comm_name))
                return False
        except rospy.ServiceException as exc:
            rospy.logerr("[BENCHMARK MISSION] {} failed: {}".format(comm_name, str(exc)))
            return False
        return True

    # def send_command(self, proxy_key, comm_name, comm_str):
    #     rospy.loginfo("[BENCHMARK MISSION] Commanding {}.".format(comm_name))
    #     try:
    #         if proxy_key == MissionComm.MISSION_COMMAND:
    #             resp = self.MC(comm_str)
    #         elif proxy_key == MissionComm.ALTITUDE_COMMAND:
    #             resp = self.UA(comm_str)
    #         elif proxy_key == MissionComm.TETHER_COMMAND:
    #             resp = self.TL(comm_str)
    #         if not resp.success:
    #             rospy.logerr("[BENCHMARK MISSION] {} failed.".format(comm_name))
    #             return False
    #     except rospy.ServiceException as exc:
    #         rospy.logerr("[BENCHMARK MISSION] {} failed: {}".format(comm_name, str(exc)))
    #         return False
    #     return True

    # def sleep_for(self, n_secs):
    #     rospy.loginfo("[BENCHMARK MISSION] Sleeping for")
    #     for i in range(n_secs):
    #         rospy.loginfo("[BENCHMARK MISSION] {}...".format(n_secs - i))
    #         self.oneSecond.sleep()
