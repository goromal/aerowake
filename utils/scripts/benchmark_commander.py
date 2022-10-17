#!/usr/bin/python

import rospy #, subprocess
from rosflight_msgs.msg import Command

def getCommandType(flag):
    if flag == 7:
        return Command.MODE_PITCH_YVEL_YAW_ALTITUDE
    if flag == 4:
        return Command.MODE_XPOS_YPOS_YAW_ALTITUDE
    if flag == 5:
        return Command.MODE_XVEL_YVEL_YAWRATE_ALTITUDE
    return 0

if __name__ == '__main__':

    # initialize
    rospy.init_node('benchmark_commander')
    rospy.loginfo('BENCHMARK COMMANDED ACTIVE')
    command_pub = rospy.Publisher('high_level_command', Command, queue_size=1)
    sleep_time = 0.05
    sleep_rate = 1.0 / sleep_time
    rate = rospy.Rate(sleep_rate)

    # retrieve benchmarks
    bench_times = rospy.get_param('~bench_times')
    bench_types = rospy.get_param('~bench_types')
    bench_com_x = rospy.get_param('~bench_com_x')
    bench_com_y = rospy.get_param('~bench_com_y')
    bench_com_z = rospy.get_param('~bench_com_z')
    bench_com_F = rospy.get_param('~bench_com_F')

    command_k = Command()
    command_k.mode   = Command.MODE_ROLL_PITCH_YAWRATE_THROTTLE
    command_k.ignore = 8
    command_k.F      = 0.0
    command_k.x      = 0.0
    command_k.y      = 0.0
    command_k.z      = 0.0

    time_k = bench_times.pop(0)
    type_k = getCommandType(bench_types.pop(0))
    comx_k = bench_com_x.pop(0)
    comy_k = bench_com_y.pop(0)
    comz_k = bench_com_z.pop(0)
    comF_k = bench_com_F.pop(0)

    idx = 0
    while not rospy.is_shutdown():
        command_k.header.stamp = rospy.Time.now()
        time_i = idx * sleep_time

        if time_i >= time_k:
            rospy.loginfo('New benchmark commanded.')
            command_k.mode = type_k
            command_k.ignore = 0
            command_k.F = comF_k
            command_k.x = comx_k
            command_k.y = comy_k
            command_k.z = comz_k
            try:
                time_k = bench_times.pop(0)
                type_k = getCommandType(bench_types.pop(0))
                comx_k = bench_com_x.pop(0)
                comy_k = bench_com_y.pop(0)
                comz_k = bench_com_z.pop(0)
                comF_k = bench_com_F.pop(0)
            except IndexError:
                rospy.loginfo('Benchmark Commander has reached the end of commands. Publishing last given command indefinitely...')
                time_k = 10e10

        command_pub.publish(command_k)
        rate.sleep()
        idx += 1
