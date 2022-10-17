import rospy
from rosflight_msgs.msg import Status, Command
from math import pi

class CommandPubSub():
    armed = False
    status_sub = None
    rf_com_pub = None
    rc_com_pub = None
    timer = None
    mode = 'ROSflight'
    command = None
    F = 0.0
    x = 0.0
    y = 0.0
    z = 0.0

    @staticmethod
    def initialize():
        CommandPubSub.status_sub = rospy.Subscriber('status', Status, CommandPubSub.status_callback)
        CommandPubSub.command = Command()
        CommandPubSub.rf_com_pub = rospy.Publisher('command', Command, queue_size=1)
        CommandPubSub.rc_com_pub = rospy.Publisher('high_level_command', Command, queue_size=1)
        CommandPubSub.timer = rospy.Timer(rospy.Duration(1.0/25.0), CommandPubSub.update)

    @staticmethod
    def setArmed(armed): # should only be called this way ONCE, at initialization to set to False
        CommandPubSub.armed = armed

    @staticmethod
    def getArmed():
        return CommandPubSub.armed

    @staticmethod
    def setMode(mode):
        CommandPubSub.mode = mode

    @staticmethod
    def setF(F):
        CommandPubSub.F = F

    @staticmethod
    def setx(x):
        CommandPubSub.x = x

    @staticmethod
    def sety(y):
        CommandPubSub.y = y

    @staticmethod
    def setz(z):
        CommandPubSub.z = z

    @staticmethod
    def status_callback(msg):
        CommandPubSub.armed = msg.armed and (not msg.rc_override)

    @staticmethod
    def update(event):
        # command = Command()
        CommandPubSub.command.header.stamp = rospy.Time.now()
        if CommandPubSub.armed:
            CommandPubSub.command.ignore = 0
        else:
            CommandPubSub.command.ignore = 8
        if CommandPubSub.mode == 'ROSflight':
            CommandPubSub.command.mode = Command.MODE_ROLL_PITCH_YAWRATE_THROTTLE
            CommandPubSub.command.F = CommandPubSub.F
            CommandPubSub.command.x = pi * CommandPubSub.x / 180.0
            CommandPubSub.command.y = pi * CommandPubSub.y / 180.0
            CommandPubSub.command.z = pi * CommandPubSub.z / 180.0
            CommandPubSub.publishCommand()

        elif CommandPubSub.mode == 'Aerowake':
            CommandPubSub.command.mode = Command.MODE_PITCH_YVEL_YAW_ALTITUDE
            CommandPubSub.command.F = CommandPubSub.F
            CommandPubSub.command.x = pi * CommandPubSub.x / 180.0
            CommandPubSub.command.y = CommandPubSub.y
            CommandPubSub.command.z = pi * CommandPubSub.z / 180.0
            CommandPubSub.publishHighLevelCommand()

        elif CommandPubSub.mode == 'ROScopterPOS':
            CommandPubSub.command.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE
            CommandPubSub.command.F = CommandPubSub.F
            CommandPubSub.command.x = CommandPubSub.x
            CommandPubSub.command.y = CommandPubSub.y
            CommandPubSub.command.z = pi * CommandPubSub.z / 180.0
            CommandPubSub.publishHighLevelCommand()

        elif CommandPubSub.mode == 'ROScopterVEL':
            CommandPubSub.command.mode = Command.MODE_XVEL_YVEL_YAWRATE_ALTITUDE
            CommandPubSub.command.F = CommandPubSub.F
            CommandPubSub.command.x = CommandPubSub.x
            CommandPubSub.command.y = CommandPubSub.y
            CommandPubSub.command.z = pi * CommandPubSub.z / 180.0
            CommandPubSub.publishHighLevelCommand()

    @staticmethod
    def publishCommand():
        # print 'GUI publish COMMAND'
        CommandPubSub.rf_com_pub.publish(CommandPubSub.command)

    @staticmethod
    def publishHighLevelCommand():
        CommandPubSub.rc_com_pub.publish(CommandPubSub.command)
