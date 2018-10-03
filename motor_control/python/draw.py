import sys
import time
import socket
import struct
import numpy as np
import matplotlib.pyplot as plt

class TivaController:
    def __init__(self, units_per_cm=1, arm1_cm=20, arm2_cm=10, x_offset_cm=0, y_offset_cm=0, bufsize=8):
        # Arm properties
        self.units_per_cm = units_per_cm

        self.x_offset = x_offset_cm * self.units_per_cm
        self.y_offset = y_offset_cm * self.units_per_cm

        self.a1 = arm1_cm * self.units_per_cm
        self.a2 = arm2_cm * self.units_per_cm

        # Arm initialization
        self.q1 = 0.0
        self.q2 = 0.0
        self.find_arm_locations()

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

    def move_arm(self, x, y):
        self.q1, self.q2 = self.compute_kinematics(x, y)
        self.find_arm_locations()

    def compute_kinematics(self, x, y, flip=False):

        x += self.x_offset
        y += self.y_offset

        q2 = np.arccos((x**2 + y**2 - self.a1**2 - self.a2**2) / (2.0 * self.a1 * self.a2))
        
        if flip: # this solution isn't working currently
            q2 = -q2
            q1 = np.arctan2(y, x) + np.arctan2( (self.a2 * np.sin(q2)), (self.a1 + (self.a2 * np.cos(q2))) )
        else:
            q1 = np.arctan2(y, x) - np.arctan2( (self.a2 * np.sin(q2)), (self.a1 + (self.a2 * np.cos(q2))) )
            
        if np.isnan(q1) or np.isnan(q2):
            raise ValueError("Invalid arm position: x:{} y:{}".format(x, y))

        return q1, q2
    
    def find_arm_locations(self):
        self.x1 = self.a1 * np.cos(self.q1)
        self.y1 = self.a1 * np.sin(self.q1)
        self.x2 = self.x1 + (self.a2 * np.cos(self.q1+self.q2))
        self.y2 = self.y1 + (self.a2 * np.sin(self.q1+self.q2))

if __name__ == "__main__":
    Tiva = TivaController(units_per_cm=1, arm1_cm=20, arm2_cm=10, 
                          x_offset_cm=0, y_offset_cm=0, bufsize=8) # set up comm channels
    
    # draw a circle
    n_steps = 50
    r = 5
    u = np.linspace(0, 2*np.pi, num=n_steps)
    x = r * np.cos(u) + 3
    y = r * np.sin(u) + 18

    fig, ax = plt.subplots()

    for idx, point in enumerate(zip(x, y)):
        Tiva.move_arm(point[0], point[1])

        sys.stdout.write("Computing step {0}/{1}\r".format(idx, n_steps))
        sys.stdout.flush()

        x1, y1 = [0, Tiva.x1], [0, Tiva.y1]
        x2, y2 = [Tiva.x1, Tiva.x2], [Tiva.y1, Tiva.y2]
        ax.plot(x1, y1, marker = 'o', color='b')
        ax.plot(x2, y2, marker = 'o', color='b')
        ax.plot(point[0], point[1], marker = 'o', color='r')
        plt.axis('equal')
        #ax.set_xlim(0, 100)
        #ax.set_ylim(0, 300)
        ax.grid('on')
        plt.savefig("img/{}.png".format(idx))
        
        #comm.send([q1, q2])

