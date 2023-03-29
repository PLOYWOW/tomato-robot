#!/usr/bin/env python

import socket
from threading import Thread
import rospy
from std_msgs.msg import String

class Server:
    def __init__(self, host, port):
        self.pub = rospy.Publisher('xarm2dolly', String, queue_size=1)
        rospy.init_node('xarm_dolly', anonymous=True)

        self.host = host
        self.port = port
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((self.host, self.port))
        self.server.listen(5)

    def listen_for_clients(self):
        print('Listening...')
        #while not rospy.is_shutdown:
        client, addr = self.server.accept()
        print(
            'Accepted Connection from: ' + str(addr[0]) + ':' + str(addr[1])
        )
        Thread(target=self.handle_client, args=(client, addr)).start()

    def handle_client(self, client_socket, address):
        size = 1024
        while True:
            try:
                data = client_socket.recv(size)
                if 'kill' in data.decode():    
                    print('Received request for exit from: ' + str(
                        address[0]) + ':' + str(address[1]))
                    break

                else:
                    # send getting after receiving from client
                    client_socket.sendall('Welcome to server'.encode())
                    dataText = data.decode()

                    print('Received: ' + data.decode() + ' from: ' + str(
                        address[0]) + ':' + str(address[1]))
                    self.pub.publish(dataText)

            except socket.error:
                client_socket.close()
                return False

        client_socket.sendall(
            'Received request for exit. Deleted from server threads'.encode()
        )

        # send quit message to client too
        client_socket.sendall(
            'kill^'.encode()
        )
        client_socket.close()
    
    def process(self):
        while not rospy.is_shutdown():
            self.listen_for_clients()
        



if __name__ == "__main__":
    
    host = '192.168.1.210'
    port = 9999
    main = Server(host, port)
    # start listening for clients
    main.process()