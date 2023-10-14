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

serial_port_name = "COM4"
serial_port = serial.Serial(serial_port_name, baudrate=115200, parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE, timeout=0)
desired_positions = [0.0, 0.0, 0.0, 1.5707, 0.0] # A = grabber open/close (Link 4) B = Link 3 C = Link 2 B = Link 1 C = Link 0 
ThetaA = atan(145.3/40)
Link0 = DHLink(d= 0.0462, a= 0.020, alpha= pi/2, qlim= [radians(0),radians(350)]) # Base Link
Link1 = DHLink(d= 0, a= 0.15071, alpha= pi, qlim= [radians(0),radians(200)]) 
Link2 = DHLink(d= 0, a= 0.020, alpha= -pi/2, qlim= [radians(0),radians(200)])
Link3 = DHLink(d= -0.180, a= 0, alpha= pi/2, qlim= [radians(0),radians(350)])
Link4 = DHLink(d= 0, a= 0, alpha= 0, qlim= [radians(0), radians(90)]) # Grabber

workspace = [-0.4, 0.4, -0.4, 0.4, -0.4, 0.4]                   # Set the size of the workspace
ReachAlpha5 = DHRobot([Link0 , Link1, Link2, Link3, Link4])
Origin = [0, 1.5707 - ThetaA, 0 - ThetaA , 0 + pi/2, 0]     #starting position                                                 # Create a vector of initial joint angles
Destination = transl(-0.021, -0.152, 0.171) #input coordiantes here
q1 = ReachAlpha5.ikine_LM(Destination, q0= Origin).q
print('Actual Joints  = ', q1)
Actual = ReachAlpha5.fkine(q1)
print('Actual Fkine = ', Actual.t)
ReachAlpha5.teach(q1)
# Desired position from end effector to base A -> E
desired_positions = [10, q1[3], q1[2],q1[1],q1[0]]
packets = b''
for index, position in enumerate(desired_positions):
    device_id = index + 1
    packets += BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([position]))
    serial_port.write(packets)

time.sleep(10)

desired_positions = [0, 0, 0,1.5707,0]
packets = b''
for index, position in enumerate(desired_positions):
    device_id = index + 1
    packets += BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([position]))
    serial_port.write(packets)

time.sleep(10)

Destination = transl(-0.021, 0.152, 0.171)
q1 = ReachAlpha5.ikine_LM(Destination, q0= Origin).q
ReachAlpha5.teach(q1)
desired_positions = [10, q1[3], q1[2],q1[1],q1[0]]
packets = b''
for index, position in enumerate(desired_positions):
    device_id = index + 1
    packets += BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([position]))
    serial_port.write(packets)

time.sleep(10)

desired_positions = [0, 0, 0,1.5707,0]
packets = b''
for index, position in enumerate(desired_positions):
    device_id = index + 1
    packets += BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([position]))
    serial_port.write(packets)

