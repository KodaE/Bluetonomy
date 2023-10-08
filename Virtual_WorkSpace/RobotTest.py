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

#Class to be implemented
#input desired coordinates into 'Destination' Variable

#serial_port_name = "COM3"
#serial_port = serial.Serial(serial_port_name, baudrate=115200, parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE, timeout=0)
desired_positions = [0.0, 0.0, 0.0, 1.5707, 0.0] # A = grabber open/close (Link 4) B = Link 3 C = Link 2 B = Link 1 C = Link 0 
ThetaA = atan(145.3/40)
Link0 = DHLink(d= 0.0462, a= 0.020, alpha= pi/2, qlim= [radians(0),radians(350)]) # Base Link
Link1 = DHLink(d= 0, a= 0.15071, alpha= pi, qlim= [radians(90),radians(290)]) 
Link2 = DHLink(d= 0, a= 0.020, alpha= -pi/2, qlim= [-radians(0),radians(200)])
Link3 = DHLink(d= -0.180, a= 0, alpha= pi/2, qlim= [radians(0),radians(350)])
Link4 = DHLink(d= 0, a= 0, alpha= 0, qlim= [-radians(0), radians(90)]) # Grabber
# all links 0 - limit besides link 1
print('ThetaA = ', ThetaA)
workspace = [-0.4, 0.4, -0.4, 0.4, -0.4, 0.4]                   # Set the size of the workspace
ReachAlpha5 = DHRobot([Link0 , Link1, Link2, Link3, Link4])
ReachAlpha5.q = [0, 1.5707 - ThetaA, 0 - ThetaA , 0 + pi/2, 0]     #starting position                                                  # Create a vector of initial joint angles
Destination = transl(-0.019, -0.138, 0.213) #input coordiantes here
DesiredJoints = [radians(82.2), radians(82.0), radians(-8.0), radians(90), radians(7.8)]
Desired = ReachAlpha5.fkine(DesiredJoints).t
q1 = ReachAlpha5.ikine_LM(Destination, q0= ReachAlpha5.q).q
print('Desired Joints = ', DesiredJoints)
print('Actual Joints  = ', q1)
Actual = ReachAlpha5.fkine(q1)
Distance = norm(Desired - Actual.t)
print('Desired Fkine = ', Desired, 'Actual Fkine = ', Actual.t)
print('Distance between = ', Distance)

# Desired position from end effector to base A -> E
desired_positions = [degrees(q1[4]), q1[3], q1[2],q1[1],q1[0]]
#packets = b''
#for index, position in enumerate(desired_positions):
    #device_id = index + 1
    #packets += BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([position]))
    #serial_port.write(packets)
coordiantes = [0,2,1]
define = np.array(coordiantes)
x = define[0]
y = define[1]
z = define[2]
for index in range(len(define)):
    print(define[index])
outer_limits = pow(sqrt(pow(x,2)+pow(y,2)) - ReachAlpha5.a[0] ,2) + pow(z - ReachAlpha5.d[0],2) <= pow(ReachAlpha5.a[1]  + sqrt(pow(ReachAlpha5.d[3],2)+pow(ReachAlpha5.a[2],2)),2)
inner_limits = pow(sqrt(pow(x,2)+pow(y,2)) - ReachAlpha5.a[0],2) + pow(z - ReachAlpha5.d[0],2) >= (pow(0.03994 + ReachAlpha5.a[2],2) + pow(0.1453 + ReachAlpha5.d[3],2))

if inner_limits and outer_limits:
    print('Winner')

else:
    print('loser')
