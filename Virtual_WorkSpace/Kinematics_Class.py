import math
from math import *
import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
from spatialmath.base import *
from roboticstoolbox import DHLink, DHRobot, jtraj
import math
from math import *
import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
from spatialmath.base import *
from roboticstoolbox import DHLink, DHRobot, jtraj
import roboticstoolbox as rtb
from bplprotocol import BPLProtocol, PacketID
import time
import serial


class Kinematics:

    def __init__(self, Coordinates, COMPORT, data_queue):
        self.comport = COMPORT
        #self.serial_port = serial.Serial(self.comport, baudrate=115200, parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE, timeout=0)
        self.coordinates = np.array(Coordinates)
        self.data_queue = data_queue
        self.ModelRobot()
        self.CalculateandMove()
        

    def ModelRobot(self):
        self.ThetaA = tanh(145.3/40)
        Link0 = DHLink(d= 0.0462, a= 0.020, alpha= pi/2, qlim= [radians(0)+pi,radians(350)+pi]) # Base Link
        Link1 = DHLink(d= 0, a= 0.15071, alpha= pi, qlim= [1.5707-self.ThetaA,radians(200)-self.ThetaA]) 
        Link2 = DHLink(d= 0, a= 0.020, alpha= -pi/2, qlim= [radians(0)-self.ThetaA,radians(200)-self.ThetaA])
        Link3 = DHLink(d= -0.180, a= 0, alpha= pi/2, qlim= [radians(0)+pi/2,radians(350)+pi/2])
        Link4 = DHLink(d= 0, a= 0, alpha= 0, qlim= [radians(0), radians(90)]) # Grabber
        self.ReachAlpha5 = DHRobot([Link0 , Link1, Link2, Link3, Link4])
        self.ReachAlpha5.q = [0 + pi, 1.5707 - self.ThetaA, 0 - self.ThetaA , 0 + pi/2, 0]
        self.Origin = self.ReachAlpha5.q 
        print(self.ReachAlpha5.fkine(self.Origin))
    
    def CalculateandMove(self):
        self.steps = 100
        for self.index in range(len(self.coordinates)): #set the coordinates for each one inputted (Come back later to allow inputs to be determined )
            self.x = self.coordinates[self.index,0]
            self.y = self.coordinates[self.index,1]
            self.z = self.coordinates[self.index,2]
            self.outer_limits = pow(sqrt(pow(self.x,2)+pow(self.y,2)) - self.ReachAlpha5.a[0],2) + pow(self.z - self.ReachAlpha5.d[0],2) <= pow(self.ReachAlpha5.a[1]  + sqrt(pow(self.ReachAlpha5.d[3],2)+pow(self.ReachAlpha5.a[2],2)),2)
            self.inner_limits = pow(sqrt(pow(self.x,2)+pow(self.y,2)) - self.ReachAlpha5.a[0],2) + pow(self.z - self.ReachAlpha5.d[0],2) >= (pow(0.03994 + self.ReachAlpha5.a[2],2) + pow(0.1453 + self.ReachAlpha5.d[3],2))
            self.inner_lower_limits = pow(sqrt(pow(self.x,2)+pow(self.y,2)) - self.ReachAlpha5.a[0],2) + pow(self.z - self.ReachAlpha5.d[0] + 0.1453,2) >= pow(-self.ReachAlpha5.d[3],2)
            if self.outer_limits and self.inner_limits and self.inner_lower_limits :
                print('Reachable Position: ', self.coordinates[self.index])
                T1 = transl(self.coordinates[self.index])
                self.qdestination = self.ReachAlpha5.ikine_LM(T1,q0=self.ReachAlpha5.q).q
                self.trajectory = jtraj(self.ReachAlpha5.q, self.qdestination,self.steps).q
                fig = plt.figure(1)
                fig = self.ReachAlpha5.plot(self.ReachAlpha5.q,fig=fig)
                ax = plt.gca()
                for self.q in self.trajectory:
                    self.desired_position = [degrees(self.q[4]), self.q[3], self.q[2], self.q[1], self.q[0]]
                    self.ReachAlpha5.q = self.q
                    fig.step(0.05)
                    
                    packets = b''
                    #for index, position in enumerate(self.desired_position):
                        #device_id = index + 1
                        #packets += BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([position]))
                        #self.serial_port.write(packets)
                
                print(self.ReachAlpha5.fkine(self.ReachAlpha5.q))

                self.trajectoryback= jtraj(self.ReachAlpha5.q, self.Origin,self.steps).q
                for self.q in self.trajectoryback:
                    self.desired_position = [degrees(self.q[4]), self.q[3], self.q[2], self.q[1], self.q[0]]
                    self.ReachAlpha5.q = self.q
                    fig.step(0.05)
                    
                    packets = b''
                    #for index, position in enumerate(self.desired_position):
                        #device_id = index + 1
                        #packets += BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([position]))
                        #self.serial_port.write(packets)
                print(self.ReachAlpha5.fkine(self.ReachAlpha5.q))
            else:
                print('Unreachable Position: ', self.coordinates[self.index])
                continue

if __name__ == '__main__':
    print('hello world')
    Coordinates = [[0.0,0.0,0.0],[0.0,0.5,0.4],[-0.019, -0.138, 0.213]]
    Kinematics(Coordinates=Coordinates, COMPORT='COM3')