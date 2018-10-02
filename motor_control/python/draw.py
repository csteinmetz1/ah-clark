import time
import socket
import struct
import numpy as np
import matplotlib.pyplot as plt

class TivaController:
    def __init__(self, units_per_cm=1, arm1_cm=20, arm2_cm=10, x_offset_cm=0, y_offset_cm=0, bufsize=8):
        # Arm properties
        self.units_per_cm = units_per_cm
        self.a1 = arm1_cm * self.units_per_cm
        self.a2 = arm2_cm * self.units_per_cm
        self.x_offset = x_offset_cm * self.units_per_cm
        self.y_offset = y_offset_cm * self.units_per_cm

        # UDF configuration
        self.bufsize = bufsize
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sender.settimeout(1.0)
        self.send_addr = ("127.0.0.1", 12302)
        self.receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receiver.bind(("127.0.0.1", 12403))

    def send(self, message):
        # convert message to bytes
        n_floats = len(message)
        message = bytearray(struct.pack("{}f".format(n_floats), *message))

        # check if message fits within bufsize
        if len(message) != self.bufsize:
            raise ValueError("Invalid message size of {}. Must fit in buffer of size {}".format(len(message), self.bufsize))

        try:
            self.sender.sendto(message, self.send_addr)
        except:
            pass

    def recieve(self):
        try:
            data, server = self.receiver.recvfrom(self.bufsize)
        except socket.timeout:
            print('REQUEST TIMED OUT')

    def compute_kinematics(self, x, y, flip=False):

        x += self.x_offset
        y += self.y_offset

        q2 = np.arccos((x**2 + y**2 - self.a1**2 - self.a2**2) / (2 * self.a1 * self.a2))
        
        if flip:
            q2 = -q2
            q1 = np.arctan2(y, x) + np.arctan2( (self.a2 * np.sin(q2)), (self.a1 + (self.a2 * np.cos(q2))) )
        else:
            q1 = np.arctan2(y, x) - np.arctan2( (self.a2 * np.sin(q2)), (self.a1 + (self.a2 * np.cos(q2))) )
            
        return q1*(180/np.pi), q2*(180/np.pi)

if __name__ == "__main__":
    Tiva = TivaController(units_per_cm=1, arm1_cm=20, arm2_cm=10, x_offset_cm=0, y_offset_cm=0, bufsize=8) # set up comm channels
    
    # draw a circle
    n_steps = 25
    r = 15.0
    u = np.linspace(0, 2*np.pi, num=n_steps)
    x = r * np.cos(u)
    y = r * np.sin(u)

    for idx, point in enumerate(zip(x, y)):

        q1, q2 = Tiva.compute_kinematics(point[0], point[1])
        print(q1, q2)

        _x1 = Tiva.a1 * np.cos(q1)
        _y1 = Tiva.a1 * np.sin(q1)
        _x2 = Tiva.a2 * np.cos(q2)
        _y2 = Tiva.a2 * np.sin(q2)

        print(_x2, _y2)

        x1, y1 = [0, _x1], [0, _y1]
        x2, y2 = [_x1, _x2], [_y1, _y2]
        plt.plot(x1, y1, x2, y2, marker = 'o')
        plt.xlim(-20, 20)
        plt.ylim(-20, 20)
        plt.show()
        #comm.send([q1, q2])

