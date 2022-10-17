#!/usr/bin/python

from mission_comm import MissionComm as MC
import curses, multiprocessing, rospy

class TextWindow():

    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=10):
        self._screen = stdscr
        self._screen.nodelay(True)
        self._screen.keypad(1)
        curses.curs_set(0)

        self._num_lines = lines

    def read_key(self):
        keycode = self._screen.getch()
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message):
        if lineno < 0 or lineno >= self._num_lines:
            raise ValueError('lineno out of bounds')
        height, width = self._screen.getmaxyx()
        y = int((height / self._num_lines) * lineno)
        x = 10
        for text in message.split('\n'):
            text = text.ljust(width)
            self._screen.addstr(y, x, text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()

class MissionTeleop():
    def __init__(self, interface):
        self._interface = interface
        self._hz = rospy.get_param('~hz', 10)
        self._tlength = rospy.get_param('~sweep_tether_length', 0.0)
        self._acceptable_keys = [ord('f'), ord('a'), ord('t'), ord('c'), ord('l'), ord('r'), ord('z'), ord('q'), ord('g')]
        self._last_pressed = None

        # current thread
        self._curr_thread = None
        self._cmd_str = "Ready when you are..."

    def run(self):
        rate = rospy.Rate(self._hz)
        self._running = True
        while self._running:
            keycode = self._interface.read_key()
            if keycode and not (keycode == self._last_pressed) and keycode in self._acceptable_keys:
                self._send_cmd(keycode)
            self._last_pressed = keycode
            self._broadcast()
            rate.sleep()

    def _sweep_length_cmd(self):
        MC.send_command(MC.TETHER_COMMAND, 'Tether Length = ' + str(self._tlength), str(self._tlength))

    def _zero_length_cmd(self):
        MC.send_command(MC.TETHER_COMMAND, 'Tether Length = 0.0', '0.0')

    def _activate_cmd(self):
        MC.send_command(MC.MISSION_COMMAND, 'Activate', 'activate')

    def _takeoff_cmd(self):
        MC.send_command(MC.MISSION_COMMAND, 'Takeoff', 'takeoff')

    def _center_cmd(self):
        MC.send_command(MC.MISSION_COMMAND, 'Center', 'center')

    def _sweep_left_cmd(self):
        MC.send_command(MC.MISSION_COMMAND, 'Sweep Left', 'sweep_left')

    def _sweep_right_cmd(self):
        MC.send_command(MC.MISSION_COMMAND, 'Sweep Right', 'sweep_right')

    def _ground_cmd(self):
        MC.send_command(MC.MISSION_COMMAND, 'Land', 'land')

    def _send_cmd(self, keycode):
        # terminate current thread if one is running
        if not (self._curr_thread is None):
            self._curr_thread.terminate()

        # attach correct process
        if keycode == ord('f'):
            self._curr_thread = multiprocessing.Process(target=self._sweep_length_cmd)
            self._cmd_str = 'TETHER LENGTH TO FULL SWEEP'
        elif keycode == ord('a'):
            self._curr_thread = multiprocessing.Process(target=self._activate_cmd)
            self._cmd_str = 'ACTIVATE'
        elif keycode == ord('t'):
            self._curr_thread = multiprocessing.Process(target=self._takeoff_cmd)
            self._cmd_str = 'TAKEOFF'
        elif keycode == ord('c'):
            self._curr_thread = multiprocessing.Process(target=self._center_cmd)
            self._cmd_str = 'CENTER'
        elif keycode == ord('l'):
            self._curr_thread = multiprocessing.Process(target=self._sweep_left_cmd)
            self._cmd_str = 'SWEEP LEFT'
        elif keycode == ord('r'):
            self._curr_thread = multiprocessing.Process(target=self._sweep_right_cmd)
            self._cmd_str = 'SWEEP RIGHT'
        elif keycode == ord('z'):
            self._curr_thread = multiprocessing.Process(target=self._zero_length_cmd)
            self._cmd_str = 'TETHER LENGTH TO ZERO'
        elif keycode == ord('g'):
            self._curr_thread = multiprocessing.Process(target=self._ground_cmd)
            self._cmd_str = 'UAV LAND'
        elif keycode == ord('q'):
            self._shutdown()
            return

        # begin process
        self._curr_thread.start()

    def _broadcast(self):
        self._interface.clear()
        self._interface.write_line(2, 'CURRENT COMMAND [f a t c l r z g q]:')
        self._interface.write_line(5, self._cmd_str)
        self._interface.refresh()

    def _shutdown(self):
        self._running = False
        if not (self._curr_thread is None):
            self._curr_thread.terminate()
        rospy.signal_shutdown('Mission Teleop says Bye.')

def main(stdscr):
    rospy.init_node('mission_commander')
    app = MissionTeleop(TextWindow(stdscr))
    app.run()

if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        print 'Shutting down Mission Commander'
