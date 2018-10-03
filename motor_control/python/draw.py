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

        q2 = np.arccos((x**2 + y**2 - self.a1**2 - self.a2**2) / (2.0 * self.a1 * self.a2))
        
        if flip: # this solution isn't working currently
            q2 = -q2
            q1 = np.arctan2(y, x) + np.arctan2( (self.a2 * np.sin(q2)), (self.a1 + (self.a2 * np.cos(q2))) )
        else:
            q1 = np.arctan2(y, x) - np.arctan2( (self.a2 * np.sin(q2)), (self.a1 + (self.a2 * np.cos(q2))) )
            
        return q1, q2
    
    def joints_to_hand(self, q1,q2):
        x1 = self.a1 * np.cos(q1)
        y1 = self.a1 * np.sin(q1)
        x2 = x1 + (self.a2 * np.cos(q1+q2))
        y2 = y1 + (self.a2 * np.sin(q1+q2))

        return x1, y1, x2, y2

if __name__ == "__main__":
    Tiva = TivaController(units_per_cm=1, arm1_cm=10, arm2_cm=10, x_offset_cm=0, y_offset_cm=0, bufsize=8) # set up comm channels
    
    # draw a circle
    n_steps = 25
    r = 10
    u = np.linspace(0, np.pi, num=n_steps)
    x = r * np.cos(u) + 0
    y = r * np.sin(u) + 5

    #x = np.linspace(-5, 5, 25)
    #y = np.linspace(10, 12, 25)

    #x = [1]
    #y = [1]

    fig, ax = plt.subplots()

    for idx, point in enumerate(zip(x, y)):

        #print(point[0], point[1])
        q1, q2 = Tiva.compute_kinematics(point[0], point[1])
        _x1, _y1, _x2, _y2 = Tiva.joints_to_hand(q1, q2)

        print("{0}: {1:3.3f} {2:3.3f} | {3:3.3f}".format(idx, q1*(180/np.pi), q2*(180/np.pi), q1*(180/np.pi)+q2*(180/np.pi)))

        if np.isnan(q1) or np.isnan(q2):
            break

        x1, y1 = [0, _x1], [0, _y1]
        x2, y2 = [_x1, _x2], [_y1, _y2]

        ax.plot(x1, y1, marker = 'o', color='b')
        ax.plot(x2, y2, marker = 'o', color='g')
        ax.plot(point[0], point[1], marker = 'o', color='r')
        plt.axis('equal')
        plt.savefig("img/{}.png".format(idx))
        
        #comm.send([q1, q2])

