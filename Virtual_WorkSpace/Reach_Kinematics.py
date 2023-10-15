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

    def __init__(self, COMPORT, gui):
        self.comport = COMPORT
        self.gui = gui
        #self.serial_port = serial.Serial(self.comport, baudrate=115200, parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE, timeout=0)
        self.ModelRobot()
        
    def ModelRobot(self):
        self.ThetaA = atan(145.3/40)
        Link0 = DHLink(d= 0.0462, a= 0.020, alpha= pi/2, qlim= [radians(0),radians(350)],offset=pi) # Base Link
        Link1 = DHLink(d= 0, a= 0.15071, alpha= pi, qlim= [radians(90),radians(200)],offset=-self.ThetaA) 
        Link2 = DHLink(d= 0, a= 0.020, alpha= -pi/2, qlim= [radians(0),radians(200)],offset=-self.ThetaA)
        Link3 = DHLink(d= -0.180, a= 0, alpha= pi/2, qlim= [radians(0),radians(350)],offset=pi/2)
        Link4 = DHLink(d= 0, a= 0, alpha= 0, qlim= [radians(0), radians(90)],offset=-pi/2) # Grabber
        self.ReachAlpha5 = DHRobot([Link0 , Link1, Link2, Link3, Link4])
        self.ReachAlpha5.q = [0, 1.5707, 0 , 0 , 0]
        self.Origin = self.ReachAlpha5.q 
        print(self.ReachAlpha5.fkine(self.Origin))


    def twothreadsrunning(self):
        statement = True
        while statement:
            stime = time.time()
            print(stime)
            time.sleep(5)

    def InputCoordinates(self):
        coordinates = []
        num_points = int(input("Enter the number of Co-ordinates: "))
        
        for i in range(num_points):
            x = float(input(f"Enter the x-coordinate for point {i + 1}: "))
            y = float(input(f"Enter the y-coordinate for point {i + 1}: "))
            z = float(input(f"Enter the z-coordinate for point {i + 1}: "))
            coordinates.append([x, y, z])
            print(coordinates)
       
        return coordinates
    
    def Run(self):
        run = True
        while run:
            if self.gui.flag:
                coordinates = self.gui.coordinates
                self.CalculateandMove(coordinates=coordinates)
                self.gui.delete_coordinates()
    
    def CalculateandMove(self,coordinates):
        self.steps = 50
        self.coordinates = np.array(coordinates)
        
        for self.index in range(len(self.coordinates)):
            self.x = self.coordinates[self.index,0]
            self.y = self.coordinates[self.index,1]
            self.z = self.coordinates[self.index,2]
            self.outer_limits = pow(sqrt(pow(self.x,2)+pow(self.y,2)) - self.ReachAlpha5.a[0],2) + pow(self.z - self.ReachAlpha5.d[0],2) <= pow(self.ReachAlpha5.a[1]  + sqrt(pow(self.ReachAlpha5.d[3],2)+pow(self.ReachAlpha5.a[2],2)),2)
            self.inner_limits = pow(sqrt(pow(self.x,2)+pow(self.y,2)) - self.ReachAlpha5.a[0],2) + pow(self.z - self.ReachAlpha5.d[0],2) >= (pow(0.03994 + self.ReachAlpha5.a[2],2) + pow(0.1453 + self.ReachAlpha5.d[3],2))
            self.inner_lower_limits = pow(sqrt(pow(self.x,2)+pow(self.y,2)) - self.ReachAlpha5.a[0],2) + pow(self.z - self.ReachAlpha5.d[0] + 0.1453,2) >= pow(-self.ReachAlpha5.d[3],2)
            if self.outer_limits and self.inner_limits and self.inner_lower_limits :
                print('Reachable Position: ', self.coordinates[self.index])
                self.flag = True
                T1 = transl(self.coordinates[self.index])
                self.qdestination = self.ReachAlpha5.ikine_LM(T1,q0=self.ReachAlpha5.q).q
                self.trajectory = jtraj(self.ReachAlpha5.q, self.qdestination,self.steps).q
                #fig = plt.figure(1)
                #fig = self.ReachAlpha5.plot(self.ReachAlpha5.q,fig=fig)
                #ax = plt.gca()
                for self.q in self.trajectory:
                    self.desired_position = [degrees(self.q[4]) , self.q[3] , self.q[2], self.q[1] , self.q[0]]
                    self.ReachAlpha5.q = self.q
                    #fig.step(0.05)
                    print(self.q)
                    
                    #packets = b''
                    #for index, position in enumerate(self.desired_position):
                        #device_id = index + 1
                        #packets += BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([position]))
                        #self.serial_port.write(packets)
                    
                print(self.ReachAlpha5.fkine(self.ReachAlpha5.q))
                time.sleep(3)
                self.trajectoryback= jtraj(self.ReachAlpha5.q, self.Origin,self.steps).q
                for self.q in self.trajectoryback:
                    self.desired_position = [degrees(self.q[4]), self.q[3] , self.q[2] , self.q[1], self.q[0]]
                    self.ReachAlpha5.q = self.q
                    #fig.step(0.05)
                    
                    #packets = b''
                    #for index, position in enumerate(self.desired_position):
                        #device_id = index + 1
                        #packets += BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([position]))
                        #self.serial_port.write(packets)
                    
                print(self.ReachAlpha5.fkine(self.ReachAlpha5.q))
                time.sleep(3)
                
            else:
                print('Unreachable Position: ', self.coordinates[self.index])
                
                

if __name__ == '__main__':
    
    Coordinates = [[-0.019, -0.138, 0.213]]
    Kin = Kinematics(COMPORT='COM4')
    Kin.CalculateandMove(coordinates=Coordinates)
    

