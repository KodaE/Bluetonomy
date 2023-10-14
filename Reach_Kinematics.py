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
from bplprotocol import BPLProtocol, PacketID, PacketReader
import time
import serial
import keyboard

class Kinematics:

    def __init__(self, COMPORT):
        self.comport = COMPORT
        self.serial_port = serial.Serial(self.comport, baudrate=115200, parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE, timeout=0)
        self.packets =b''
        self.device_id = [0x01, 0x02, 0x03, 0x04, 0x05]
        self.reachable = True
        self.ra5_base_id = 0x05
        self.request_timeout =  2
        self.stop_flag = False
        self.ModelRobot()
        

    def ModelRobot(self):
        self.ThetaA = atan(145.3/40)
        Link0 = DHLink(d= 0.0462, a= 0.020, alpha= pi/2, qlim= [radians(0),radians(350)],offset=pi) # Base Link
        Link1 = DHLink(d= 0, a= 0.15071, alpha= pi, qlim= [1.5707,radians(200)],offset=-self.ThetaA) 
        Link2 = DHLink(d= 0, a= 0.020, alpha= -pi/2, qlim= [radians(0),radians(200)],offset=-self.ThetaA)
        Link3 = DHLink(d= -0.180, a= 0, alpha= pi/2, qlim= [radians(0),radians(350)],offset=pi/2)
        Link4 = DHLink(d= 0, a= 0, alpha= 0, qlim= [radians(0), radians(90)],offset=-pi/2) # Grabber
        self.ReachAlpha5 = DHRobot([Link0 , Link1, Link2, Link3, Link4])
        self.ReachAlpha5.q = [0, 1.5707, 0 , 0 , 0]
        self.Origin = self.ReachAlpha5.q 
        # print(self.ReachAlpha5.fkine(self.Origin))
    
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
                    # print(self.q)
                    
                    packets = b''
                    for index, position in enumerate(self.desired_position):
                        device_id = index + 1
                        packets += BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([position]))
                        self.serial_port.write(packets)

                time.sleep(2)
                self.mode_status()
            
                # print(self.ReachAlpha5.fkine(self.ReachAlpha5.q))
                time.sleep(2)
                if self.stop_flag == False:
                    self.trajectoryback= jtraj(self.ReachAlpha5.q, self.Origin,self.steps).q
                    for self.q in self.trajectoryback:
                        self.desired_position = [degrees(self.q[4]), self.q[3] , self.q[2] , self.q[1], self.q[0]]
                        self.ReachAlpha5.q = self.q
                        #fig.step(0.05)
                        
                        packets = b''
                        for index, position in enumerate(self.desired_position):
                            device_id = index + 1
                            packets += BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([position]))
                            self.serial_port.write(packets)
                        
                    # print(self.ReachAlpha5.fkine(self.ReachAlpha5.q))
                    time.sleep(6)
                    self.reachable = True
                    return self.reachable
            else:
                print('Unreachable Position: ', self.coordinates[self.index])
                self.reachable = False
                return self.reachable
            
   

    def move_arm_to_pos(self, desired_positions):
        n = 0
        for position in desired_positions:
            self.packets += BPLProtocol.encode_packet(self.device_id[n], PacketID.POSITION, BPLProtocol.encode_floats([position]))
            n += 1
        self.serial_port.write(self.packets)
        

    def base_id_feedback_data(self):
        packet_reader = PacketReader()
        self.serial_port.write(BPLProtocol.encode_packet(self.ra5_base_id, PacketID.REQUEST, bytes([PacketID.KM_END_POS, PacketID.VOLTAGE, PacketID.TEMPERATURE])))
        start_time = time.time()
        position = []
        voltage = None
        temp = None
        d = dict()
        while True:
            time.sleep(0.0001)
            try:
                read_data = self.serial_port.read()
            except BaseException:
                read_data = b''
            if read_data != b'':
                packets = packet_reader.receive_bytes(read_data)
                if packets:
                    for packet in packets:
                        read_device_id, read_packet_id, data_bytes = packet
                        if read_device_id == self.ra5_base_id and read_packet_id in [PacketID.KM_END_POS, PacketID.VOLTAGE, PacketID.TEMPERATURE]:
                            if read_packet_id == PacketID.KM_END_POS:
                                position = np.array(BPLProtocol.decode_floats(data_bytes)[:3])
                            elif read_packet_id == PacketID.VOLTAGE:
                                voltage = BPLProtocol.decode_floats(data_bytes)
                            elif read_packet_id == PacketID.TEMPERATURE:
                                temp = BPLProtocol.decode_floats(data_bytes)


                    if position is not None and voltage is not None and temp is not None:
                        break

            # Timeout if no response is seen from the device.
            if time.time() - start_time > self.request_timeout:
                print("Request for Position and Velocity Timed out")
                break

        if position is not None and voltage is not None and temp is not None:
            d["end-effector-pos"] = position
            d["voltage"] = voltage
            d["temp"] = temp
            return d
        
    
    def joint_id_feedback_data(self):
        request_timeout = 1
        d = dict()
          
        for device_id in self.device_id:
          
            position = None
            packet_reader = PacketReader()  
            self.serial_port.write(BPLProtocol.encode_packet(device_id, PacketID.REQUEST, bytes([PacketID.POSITION]))) 
            start_time = time.time()
            while True:
                time.sleep(0.0001)
                try:
                    read_data = self.serial_port.read()
                except BaseException:
                    read_data = b''
                if read_data != b'':
                    packets = packet_reader.receive_bytes(read_data)
                    if packets:
                        for packet in packets:
                            read_device_id, read_packet_id, data_bytes = packet
                            if read_device_id == device_id and read_packet_id == PacketID.POSITION:
                                # Decode floats, because position is reported in floats
                                position = BPLProtocol.decode_floats(data_bytes)
                                d[device_id] = position
                                                          
                        if position is not None:
                            break

                # Timeout if no response is seen from the device.
                if time.time() - start_time > request_timeout:
                    print("Request for Position timed out")
                    break
                
        return d           



    def send_disable_comms(self):
        for device_id in self.device_id:
            self.packets += BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([0x01]))
        self.serial_port.write(self.packets)

    def stop_arm_movement(self):
        for device_id in self.device_id:
            self.packets += BPLProtocol.encode_packet(device_id, PacketID.VELOCITY, BPLProtocol.encode_floats([0]))
        self.serial_port.write(self.packets)


    # def e_stop(self):
    #     print(keyboard.is_pressed)
    #     if keyboard.is_pressed("a"):
    #         self.stop_arm_movement()
    #         self.send_disable_comms()
            

if __name__ == '__main__':
    
    Coordinates = [-0.019, -0.138, 0.213]
    Kinematics(Coordinates=Coordinates, COMPORT='COM4')