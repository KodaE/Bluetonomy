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

    def __init__(self, Coordinates):
        self.coordinates = np.array(Coordinates)
        self.ModelRobot()
        self.CalculatePosition()

    def ModelRobot(self):
        self.ThetaA = tanh(145.3/40)
        Link0 = DHLink(d= 46.2, a= 20, alpha= pi/2, qlim= [radians(-175),radians(175)]) # Base Link
        Link1 = DHLink(d= 0, a= 150.71, alpha= pi, qlim= [-radians(100),radians(100)]) 
        Link2 = DHLink(d= 0, a= 20, alpha= -pi/2, qlim= [-radians(100),radians(100)])
        Link3 = DHLink(d= -180, a= 0, alpha= pi/2, qlim= [radians(-175),radians(175)])
        Link4 = DHLink(theta= -pi/2 ,d= 0, a= 0, alpha= 0, qlim= [0, radians(90)]) # Grabber
        self.ReachAlpha5 = DHRobot([Link0 , Link1, Link2, Link3, Link4])
        self.ReachAlpha5.q = [0, 1.5707 - self.ThetaA, 0 - self.ThetaA , 0 + pi/2, 0]
    
    def CalculatePosition(self):
        for self.index in range(len(self.coordinates)):
            if pow(sqrt(pow(self.coordinates[self.index,0],2)+pow(self.coordinates[self.index,1],2))- self.ReachAlpha5.a[0],2)+pow(self.coordinates[self.index,2]-self.ReachAlpha5.d[0],2) <= pow(self.ReachAlpha5.a[1]+sqrt(pow(self.ReachAlpha5.d[3],2)+pow(self.ReachAlpha5.a[2],2)),2) and pow(sqrt(pow(self.coordinates[self.index,0],2)+ pow(self.coordinates[self.index,1],2)) - self.ReachAlpha5.a[0],2)+pow(self.coordinates[self.index,2]-self.ReachAlpha5.d[0],2) >= (pow(39.94 + self.ReachAlpha5.a[2],2)+pow(145.3+self.ReachAlpha5.d[3],2)):
                print('true')

            else:
                print('false')