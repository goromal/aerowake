#
## FLOW: Fixed-Length, One-Way socket communications.
## Andrew Torgesen, 2020
#

import asyncore, smtpd, socket, sys
from multiprocessing import Queue

if sys.version_info[0] == 2:
    from Queue import Empty
    NULLstr = ''
    to_buffer = lambda msg: str(msg)
else:
    from queue import Empty
    NULLstr = b''
    to_buffer = lambda msg: bytes(msg, 'utf-8')

class FLOWServer(smtpd.SMTPServer):
    def __init__(self, host, port, msg_len):
        asyncore.dispatcher.__init__(self)
        self.msg_len = msg_len
        self.msg_queue = Queue()
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        self.bind(('', port))
        self.listen(5)

    def handle_accept(self):
        socket, address = self.accept()
        address_str = "{0}:{1}".format(address[0], address[1])
        print('[Server] New connection established ({0}).'.format(address_str))
        FLOWMessageHandler(socket, address_str, self)

    def get_msg_len(self):
        return self.msg_len

    def put_msg_on_queue(self, data):
        self.msg_queue.put(data)

    def get_msg_from_queue(self):
        try:
            msg = self.msg_queue.get(False)
        except Empty:
            msg = None
        return msg

    def handle_close(self):
        print('[Server] Shutting down.')
        self.close()

class FLOWMessageHandler(asyncore.dispatcher):
    def __init__(self, socket, id, server):
        asyncore.dispatcher.__init__(self, socket)
        self.id = id
        self.server = server
        self.msg_len = self.server.get_msg_len()

    def handle_read(self):
        chunks = []
        bytes_recd = 0
        while bytes_recd < self.msg_len:
            chunk = self.recv(min(self.msg_len - bytes_recd, 2048))
            if chunk == NULLstr:
                raise RuntimeError("socket connection broken")
            chunks.append(chunk)
            bytes_recd = bytes_recd + len(chunk)
        msg = NULLstr.join(chunks)
        self.server.put_msg_on_queue(msg)

    def handle_error(self):
        print('[{0}] Client socket closed without warning.'.format(self.id))

    def handle_close(self):
        print('[{0}] Connection closed.'.format(self.id))
        self.close()

class FLOWClient(asyncore.dispatcher):
    def __init__(self, host, port, message):
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect((host, port))
        self.buffer = to_buffer(message)

    def writable(self):
        return (len(self.buffer) > 0)

    def handle_write(self):
        self.send(self.buffer)
        self.close()

    def handle_close(self):
        self.close()

##
## TESTING
##

import time, threading

def test_flow_client():
    FLOWClient('127.0.0.1', 4004, '100.001')
    time.sleep(2)
    FLOWClient('127.0.0.1', 4004, '200.002')
    time.sleep(2)
    FLOWClient('127.0.0.1', 4004, '300.003')
    time.sleep(2)
    FLOWClient('127.0.0.1', 4004, '400.004')
    time.sleep(2)

if __name__ == "__main__":
    server = FLOWServer('127.0.0.1', 4004, 7)
    asyncore_thread = threading.Thread(target=asyncore.loop, kwargs = {'timeout':1})
    client_thread = threading.Thread(target=test_flow_client)
    try:
        asyncore_thread.start()
        client_thread.start()
        while True:
            cmd = server.get_msg_from_queue()
            if not cmd is None: print(float(cmd))
            time.sleep(0.2)
    except KeyboardInterrupt:
        server.close()
        asyncore_thread.join()
        client_thread.join()