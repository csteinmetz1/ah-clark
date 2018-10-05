import time
import socket
import struct
import numpy as np

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

    def compute_kinematics(self, x, y, negative=True):

        x -= self.x_offset
        y -= self.y_offset
        
        if negative: # this solution isn't working currently
            q2 = -(np.arccos((x**2 + y**2 - self.a1**2 - self.a2**2) / (2.0 * self.a1 * self.a2)))
            q1 = np.arctan2(y, x) - np.arctan2( (self.a2 * np.sin(q2)), (self.a1 + (self.a2 * np.cos(q2))) )
        else:
            q2 = np.arccos((x**2 + y**2 - self.a1**2 - self.a2**2) / (2.0 * self.a1 * self.a2))
            q1 = np.arctan2(y, x) - np.arctan2( (self.a2 * np.sin(q2)), (self.a1 + (self.a2 * np.cos(q2))) )
            
        if np.isnan(q1) or np.isnan(q2):
            raise ValueError("Invalid arm position: x:{} y:{}".format(x, y))

        return q1, q2
    
    def find_arm_locations(self):
        self.x1 = self.a1 * np.cos(self.q1) + self.x_offset
        self.y1 = self.a1 * np.sin(self.q1) + self.y_offset
        self.x2 = self.x1 + (self.a2 * np.cos(self.q1+self.q2))
        self.y2 = self.y1 + (self.a2 * np.sin(self.q1+self.q2))
