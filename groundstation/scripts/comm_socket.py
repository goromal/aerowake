import socket, sys

if sys.version_info[0] == 2:
    NULLSTR = ''
    BINARYCOMM = False
else:
    NULLSTR = b''
    BINARYCOMM = True

# A socket class with fixed message length
class CommSocket(object):
    def __init__(self, sock=None, msg_length=None):
        if sock is None:
            # IPv4 (host + port), TCP
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock
        if msg_length is None:
            self.MSGLEN = 10
        else:
            self.MSGLEN = msg_length

    def client_connect(self, host, port):
        self.sock.connect((host, port))

    def send_message(self, msg):
        totalsent = 0
        while totalsent < self.MSGLEN:
            sent = self.sock.send(msg[totalsent:])
            if sent == 0:
                raise RuntimeError("socket connection broken")
            totalsent = totalsent + sent

    def receive_message(self):
        chunks = []
        bytes_recd = 0
        while bytes_recd < self.MSGLEN:
            chunk = self.sock.recv(min(self.MSGLEN - bytes_recd, 2048))
            if chunk == NULLSTR:
                raise RuntimeError("socket connection broken")
            chunks.append(chunk)
            bytes_recd = bytes_recd + len(chunk)
        return NULLSTR.join(chunks)
