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
import csv
import os

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
        self.coordinates = []
        self.ModelRobot()
        

    def ModelRobot(self):

        # [13.300000190734863, 1.2999999523162842], gripper
        # [0.0, 0.0],  wrist
        # [3.4033920764923096, 0.01745329238474369], elbow 
        # [3.4033920764923096, 0.01745329238474369], lift
        # [6.14300012588501, 0.017453299835324287] pan

        self.ThetaA = atan(145.3/40)
        #print(self.ThetaA)
        Link0 = DHLink(d= 46.2, a= 20, alpha= pi/2, qlim= [radians(0),radians(350)],offset=pi) # Base Link
        Link1 = DHLink(d= 0, a= 150.71, alpha= pi, qlim= [(pi/2),radians(200)],offset=-self.ThetaA) 
        Link2 = DHLink(d= 0, a= 20, alpha= -pi/2, qlim= [radians(0),radians(200)],offset=-self.ThetaA)
        Link3 = DHLink(d= -180, a= 0, alpha= pi/2, qlim= [radians(-360),radians(360)],offset=pi/2)
        # Link4 = DHLink(d= 0, a= 0, alpha= 0, qlim= [radians(0), radians(20)],offset=-pi/2) # Grabber #put in degrees

        # Link0 = DHLink(d= 46.2, a= 20, alpha= pi/2, qlim= [0.017, 6.14],offset=0) # Base Link
        # Link1 = DHLink(d= 0, a= 150.71, alpha= pi, qlim= [0.017, 3.4],offset=0) 
        # Link2 = DHLink(d= 0, a= 20, alpha= -pi/2, qlim= [0.017, 3.4],offset=0)
        # Link3 = DHLink(d= -180, a= 0, alpha= pi/2, qlim= [radians(-360),radians(360)],offset=0)

        self.ReachAlpha5 = DHRobot([Link0,Link1, Link2,Link3])
        # self.ReachAlpha5 = DHLink([Link0 , Link1, Link2, Link3])
        
        self.ReachAlpha5.q = [radians(180), pi/2, 0 , 0 ]
        self.Origin = self.ReachAlpha5.q 
        
        #self.ReachAlpha5.teach(self.ReachAlpha5.q)
        # print(self.ReachAlpha5.fkine(self.Origin))
        self.move_arm_to_pos([0, 0.0, 0, self.ReachAlpha5.q[1], self.ReachAlpha5.q[0]])
    
    def CalculateandMove(self,coordinates,ikine):
        self.steps = 50
        self.coordinates = np.array(coordinates)
        print(self.coordinates)
        # self.move_arm_to_pos([0, 0.0, pi/2, pi, 0])
        # time.sleep(5)
        for self.index in range(len(self.coordinates)):
            self.x = self.coordinates[self.index,0]
            self.y = self.coordinates[self.index,1]
            self.z = self.coordinates[self.index,2]
           
            
            self.openincrementindex = 0
            self.outer_limits = pow(sqrt(pow(self.x,2)+pow(self.y,2)) - self.ReachAlpha5.a[0],2) + pow(self.z - self.ReachAlpha5.d[0],2) <= pow(self.ReachAlpha5.a[1]  + sqrt(pow(self.ReachAlpha5.d[3],2)+pow(self.ReachAlpha5.a[2],2)),2)
            self.inner_limits = pow(sqrt(pow(self.x,2)+pow(self.y,2)) - self.ReachAlpha5.a[0],2) + pow(self.z - self.ReachAlpha5.d[0],2) >= (pow(39.94 + self.ReachAlpha5.a[2],2) + pow(145.3 + self.ReachAlpha5.d[3],2))
            self.inner_lower_limits = pow(sqrt(pow(self.x,2)+pow(self.y,2)) - self.ReachAlpha5.a[0],2) + pow(self.z - self.ReachAlpha5.d[0] +145.3,2) >= pow(-self.ReachAlpha5.d[3],2)
            if self.outer_limits and self.inner_limits and self.inner_lower_limits :
                print('Reachable Position: ', self.coordinates[self.index])
                T1 = transl(self.coordinates[self.index])
                # print(f"T1 = {T1}")
                
                #self.qdestination = self.JointCalculated(self.x,self.y,self.z)
                if ikine == True:
                    #print("Ikine calculated")
                    self.qdestination = self.ReachAlpha5.ikine_LM(Tep=T1,q0=self.Origin,joint_limits=True,mask=[1,1,1,0,0,0]).q
                else:
                    self.qdestination = self.JointCalculated(self.x,self.y,self.z)

                #print(f"q = {self.qdestination}")
                self.trajectory = jtraj(q0=self.ReachAlpha5.q,qf=self.qdestination,t=self.steps).q
                #print(self.trajectory)
                

                #fig = plt.figure(1)
                #fig = self.ReachAlpha5.plot(self.ReachAlpha5.q,fig=fig)
                #ax = plt.gca()
                i = 0
                openposition = 20
                closedposition = 0
                incrementopen = (openposition - closedposition) / 10
                incrementclose = (closedposition - openposition)/10
                for self.q in self.trajectory:
                    i += 1
                    
                    if i >= 40:
                        q4 += incrementopen
                    else:
                        q4=0
                    self.desired_position = [q4 , self.q[3] , self.q[2], self.q[1] , self.q[0]]
                    self.ReachAlpha5.q = self.q
                    
                    #fig.step(0.05)
                    #print(self.q)
                    
                    packets = b''
                    for index, position in enumerate(self.desired_position):
                        device_id = index + 1
                        packets += BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([position]))
                        self.serial_port.write(packets)

              

                print(self.ReachAlpha5.q)
                
                d = self.base_id_feedback_data()
                print(f'Real End Effector: {d["end-effector-pos"]}')
            
                print(f"Simulated Coordiantes: {self.ReachAlpha5.fkine(self.ReachAlpha5.q)}")
                time.sleep(8)
                self.move_arm_to_pos(desired_positions=[0,self.ReachAlpha5.q[3],self.ReachAlpha5.q[2],self.ReachAlpha5.q[1],self.ReachAlpha5.q[0]])
                time.sleep(4)
                i = 0
                self.trajectoryback= jtraj(self.ReachAlpha5.q, self.Origin,self.steps).q
                for self.q in self.trajectoryback:
                    
                    self.desired_position = [0, self.q[3] , self.q[2] , self.q[1], self.q[0]]
                    self.ReachAlpha5.q = self.q
                    #fig.step(0.05)
                    
                    packets = b''
                    for index, position in enumerate(self.desired_position):
                        device_id = index + 1
                        packets += BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([position]))
                        self.serial_port.write(packets)
                    
                # print(self.ReachAlpha5.fkine(self.ReachAlpha5.q))
                # time.sleep(6)
                self.reachable = True
                return self.reachable
            else:
                print('Unreachable Position: ', self.coordinates[self.index])
                self.reachable = False
                return self.reachable
   

    def JointCalculated(self, x, y, z):
        a0 = 20
        a1 = 150.71
        a2 = 20

        d0 = 46.2
        d3 = -180

        R = sqrt(pow(x,2)+pow(y,2))

        l1 = a1
        l2 = sqrt(pow(a2,2)+pow(d3,2))
        l3 = sqrt(pow((R+a0),2)+pow((z-d0),2))

        theta0 = atan2(y,x)
        theta1 = 3*pi/2 - atan2((z-d0),(R+a0)) - acos((pow(l1,2)+pow(l3,2)-pow(l2,2))/(2*l1*l3)) - asin((2*a2)/l1)
        theta2 = acos((pow(l1,2)-pow(l3,2)+pow(l2,2))/(2*l1*l2)) - asin((2*a2)/l1) - asin(a2/l2)

        q = [theta0, theta1, theta2, 0]

        return q
        

    def move_arm_to_pos(self, desired_positions):
        n = 0
        for position in desired_positions:
            self.packets += BPLProtocol.encode_packet(self.device_id[n], PacketID.POSITION, BPLProtocol.encode_floats([position]))
            n += 1
        self.serial_port.write(self.packets)
    
    def km_move(self, desired_positions):
        n = 0
        for position in desired_positions:
            self.packets += BPLProtocol.encode_packet(self.device_id[n], PacketID.KM_END_POS, BPLProtocol.encode_floats([position]))
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
                print("Request for base ID timed out")
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
        print(f"Joint Angles - {d}")       
        return d   
    
    



    def send_disable_comms(self):
        for device_id in self.device_id:
            self.packets += BPLProtocol.encode_packet(device_id, PacketID.MODE, bytes(1))
        self.serial_port.write(self.packets)

    def send_standby_comms(self):
        for device_id in self.device_id:
            self.packets += BPLProtocol.encode_packet(device_id, PacketID.MODE, bytes(0))
        self.serial_port.write(self.packets)
        time.sleep(1)

    def stop_arm_movement(self):
        for device_id in self.device_id:
            self.packets += BPLProtocol.encode_packet(device_id, PacketID.VELOCITY, BPLProtocol.encode_floats([0.0]))
        self.serial_port.write(self.packets)
        time.sleep(1)

    # def reset_arm_velocity(self):
    #     for device_id in self.device_id:
    #         self.packets += BPLProtocol.encode_packet(device_id, PacketID.VELOCITY, BPLProtocol.encode_floats([1000.0]))
    #     self.serial_port.write(self.packets)
    #     time.sleep(1)

    
    def log_data(self, file_title):

        header = ["Time-Stamp", "Input-Coordinate", "Reachable"]
        data = None
        directory = "./datalog"

        if not os.path.exists(directory):
            os.makedirs(directory)


        with open(f"{directory}/{file_title}.csv", mode="a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(header)
    
            time_stamp = time.strftime("%H_%M_%S", time.localtime(time.time()))
            coordinate = self.coordinates
            reachable = self.reachable
            data = [time_stamp, coordinate, reachable]
            writer.writerow(data)
    

    
    def inverseKin(self,coordinates):
        coordinates = np.array(coordinates)
        pos = transl(coordinates[0])
        ikine  = self.ReachAlpha5.ikine_QP(Tep=pos,q0=self.Origin,joint_limits=True,mask=[1,1,1,0,0,0])
        print(ikine)

if __name__ == '__main__':
    kin = Kinematics( COMPORT='COM6')
    Coordinates = [[-100, 100, 150]]

    print(kin.base_id_feedback_data())


    
    # for coor in Coordinates:
    #     coor = [coor]

    # #kin.inverseKin(coordinates=Coordinates)
    #     kin.CalculateandMove(coordinates=coor,ikine=False)
  
      
    #print(kin.joint_id_feedback_data())