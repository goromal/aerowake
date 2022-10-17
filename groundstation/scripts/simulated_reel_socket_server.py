import time, logging, ReelController
from multiprocessing import Process

from flow_sockets import FLOWServer
import asyncore, threading

HOST = '127.0.0.1' # loopback
PORT = 4004

logging.basicConfig(level=logging.INFO)

class Reel(Process):
    def __init__(self, server):
        Process.__init__(self)
        self._server = server
        self._dt_des = 1.0 / 20.0
        
    def run(self):
        self._rc = ReelController.ReelController()
        try:
            while True:
                # check for new length command
                cmd = self._server.get_msg_from_queue()
                if not cmd is None:
                    print('[Reel Server] Setting tether length to', cmd)
                    self._rc.setTetherLengthM(float(cmd))
                # loop update
                t0 = time.time()
                self._rc.update()
                dt = time.time() - t0
                sleep_time = self._dt_des - dt
                if sleep_time < 0:
                    sleep_time = 0
                time.sleep(sleep_time)
        except KeyboardInterrupt:
            print('Shutting down reel.')
            self._server.close()
            self._rc.stopMoving()
            del self._rc

if __name__ == "__main__":
    server = FLOWServer(HOST, PORT, 7)
    server_thread = threading.Thread(target=asyncore.loop, kwargs = {'timeout':1})
    reel = Reel(server)
    server_thread.start()
    reel.start()
    try:
        while True:
            if not reel:
                break
    except KeyboardInterrupt:
        print('SHUTTING DOWN TETHER REEL SERVER')
        server.close()
        server_thread.join()
        reel.join()
