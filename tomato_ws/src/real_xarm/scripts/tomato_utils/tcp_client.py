import socket
import time 
import sys

class DollyClient():
    def __init__(self, ip="192.168.1.209", port=8080, retryAttempts=10 ):
        #this is the constructor that takes in host and port. retryAttempts is given 
        # a default value but can also be fed in.
        self.ip = ip
        self.port = port
        self.retryAttempts = retryAttempts
        self.socket = None
        self.tcp_client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    def connect(self):
        try:
            self.tcp_client.connect((self.ip, self.port))
            return "connection sucsess"
        except socket.error:
            time.sleep(1)
            return "Could not connect to tcp server"
            
        
    def diconnectSocket(self):
        #perform all breakdown operations
        self.tcp_client.close()
        self.socket = None

    def sendMessage(self, data = "Stop"):
        try:
            self.tcp_client.send(data.encode())
            return "Success send message"
        except:
            pass
            return "Could not send message"


    def waitForMessage(self):
        msg = self.tcp_client.recv(4096)
        t0 = time.time()
        while time.time()-t0 < 5:
            if msg.decode() == "Pick":
                return msg.decode()
        return "Base Stop Return Time Out"
    