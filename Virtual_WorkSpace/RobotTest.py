import math
from math import *
import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
from spatialmath.base import *
from roboticstoolbox import DHLink, DHRobot, jtraj
from bplprotocol import BPLProtocol, PacketID
import time
import serial

ThetaA = atan(145.3/40)
Link0 = DHLink(d= 0.0462, a= 0.020, alpha= pi/2, qlim= [radians(0),radians(350)]) # Base Link
Link1 = DHLink(d= 0, a= 0.15071, alpha= pi, qlim= [-radians(100),radians(100)]) 
Link2 = DHLink(d= 0, a= 0.020, alpha= -pi/2, qlim= [-radians(80),radians(120)])
Link3 = DHLink(d= -0.180, a= 0, alpha= pi/2, qlim= [radians(-0),radians(350)])
Link4 = DHLink(d= 0, a= 0, alpha= 0, qlim= [-radians(45), radians(45)]) # Grabber

workspace = [-0.4, 0.4, -0.4, 0.4, -0.4, 0.4]                   # Set the size of the workspace
print('ThetaAlpha = ', degrees(ThetaA))
q = [0, 0, 0 , 0, 0]                                                       # Create a vector of initial joint angles

ReachAlpha5 = DHRobot([Link0 , Link1, Link2, Link3, Link4])

ReachAlpha5.teach(q,limits= workspace)
#input("Press Enter to play with teach and then press Enter again to finish\n") 