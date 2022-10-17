#!/usr/bin/python

from mission_comm import MissionComm as MC
import rospy

if __name__ == "__main__":
    rospy.init_node("indoor_mission")
    mission = MC()

    tlength = rospy.get_param('~sweep_tether_length', 0.0)

    mission.sleep_for(9)
    mission.send_command(MC.TETHER_COMMAND, 'Tether Length = ' + str(tlength), str(tlength))
    # mission.sleep_for(100)
    # mission.send_command(MC.TETHER_COMMAND, 'Tether Length = ' + str(0.0), str(0.0))

    # mission.send_command(MC.MISSION_COMMAND, 'Takeoff', 'takeoff')
    # mission.sleep_for(4)
    # while not mission.send_command(MC.MISSION_COMMAND, 'Center', 'center'):
    #     mission.sleep_for(2)
    # mission.sleep_for(20)
    # while not mission.send_command(MC.MISSION_COMMAND, 'Sweep', 'sweep'):
    #     mission.sleep_for(2)
    # mission.sleep_for(20)
    # while not mission.send_command(MC.MISSION_COMMAND, 'Center', 'center'):
    #     mission.sleep_for(2)
    # mission.sleep_for(4)
    # mission.send_command(MC.TETHER_COMMAND, 'Tether Length = 0', '0.0')
    # mission.sleep_for(20)
    # while not mission.send_command(MC.MISSION_COMMAND, 'Land', 'land'):
    #     mission.sleep_for(2)
    # mission.sleep_for(4)
