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
        

    def ModelRobot(self): # model the simulated Reach Alpha 5. Can help with calculations and simulations of the RA5 for BlueRov. 
        
        # Joint Limits of the Reach Alpha 5
        # [13.300000190734863, 1.2999999523162842], gripper
        # [0.0, 0.0],  wrist
        # [3.4033920764923096, 0.01745329238474369], elbow 
        # [3.4033920764923096, 0.01745329238474369], lift
        # [6.14300012588501, 0.017453299835324287] pan

        self.ThetaA = atan(145.3/40)
        
        Link0 = DHLink(d= 46.2, a= 20, alpha= pi/2, qlim= [radians(0),radians(350)],offset=pi) # Base Link
        Link1 = DHLink(d= 0, a= 150.71, alpha= pi, qlim= [(pi/2),radians(200)],offset=-self.ThetaA) 
        Link2 = DHLink(d= 0, a= 20, alpha= -pi/2, qlim= [radians(0),radians(200)],offset=-self.ThetaA)
        Link3 = DHLink(d= -180, a= 0, alpha= pi/2, qlim= [radians(-360),radians(360)],offset=pi/2)
        
        #self.ReachAlpha5.base = self.ReachAlpha5.base # use trotx, troty and trotz to rotate base around axis you want. If 

        self.ReachAlpha5 = DHRobot([Link0,Link1, Link2,Link3])
        
        
        self.ReachAlpha5.q = [radians(180), pi/2, 0 , 0 ]
        self.Origin = self.ReachAlpha5.q 
        
        self.move_arm_to_pos([0, 0.0, 0, self.ReachAlpha5.q[1], self.ReachAlpha5.q[0]])
    
    def CalculateandMove(self,coordinates,ikine): # Using Inverse Kinematics, it moves the Reach Alpha 5 to specified coordinates 
        self.steps = 50
        self.coordinates = np.array(coordinates)
        
        
        for self.index in range(len(self.coordinates)): # Goes through each array of coordinates
            self.x = self.coordinates[self.index,0]
            self.y = self.coordinates[self.index,1]
            self.z = self.coordinates[self.index,2]
           
            
            self.openincrementindex = 0
            # Limits from the Reach Alpha 5 Kinematics and Dynamics Properties pg. 2
            self.outer_limits = pow(sqrt(pow(self.x,2)+pow(self.y,2)) - self.ReachAlpha5.a[0],2) + pow(self.z - self.ReachAlpha5.d[0],2) <= pow(self.ReachAlpha5.a[1]  + sqrt(pow(self.ReachAlpha5.d[3],2)+pow(self.ReachAlpha5.a[2],2)),2)
            self.inner_limits = pow(sqrt(pow(self.x,2)+pow(self.y,2)) - self.ReachAlpha5.a[0],2) + pow(self.z - self.ReachAlpha5.d[0],2) >= (pow(39.94 + self.ReachAlpha5.a[2],2) + pow(145.3 + self.ReachAlpha5.d[3],2))
            self.inner_lower_limits = pow(sqrt(pow(self.x,2)+pow(self.y,2)) - self.ReachAlpha5.a[0],2) + pow(self.z - self.ReachAlpha5.d[0] +145.3,2) >= pow(-self.ReachAlpha5.d[3],2)
            if self.outer_limits and self.inner_limits and self.inner_lower_limits : # check the reach of the robot arm.
                print('Reachable Position: ', self.coordinates[self.index])
                T1 = transl(self.coordinates[self.index]) # converts coordinates to a transform matrix
                
                if ikine == True:
                    self.qdestination = self.ReachAlpha5.ikine_LM(Tep=T1,q0=self.Origin,joint_limits=True,mask=[1,1,1,0,0,0]).q
                else:
                    self.qdestination = self.JointCalculated(self.x,self.y,self.z) 

                self.trajectory = jtraj(q0=self.ReachAlpha5.q,qf=self.qdestination,t=self.steps).q # sets trajectory of robot using Peter Corkes Robotics Toolbox

                #fig = plt.figure(1) # uncomment any fig parts to look at simulation version
                #fig = self.ReachAlpha5.plot(self.ReachAlpha5.q,fig=fig)
                #ax = plt.gca()
                i = 0
                openposition = 20
                closedposition = 0
                incrementopen = (openposition - closedposition) / 10
                incrementclose = (closedposition - openposition)/10
                for self.q in self.trajectory: # robot moves to the coordinate position through the matrix of joint coordinates to joint angle for coordinates 
                    i += 1
                    
                    if i >= 40:
                        q4 += incrementopen
                    else:
                        q4=0
                    self.desired_position = [q4 , self.q[3] , self.q[2], self.q[1] , self.q[0]]
                    self.ReachAlpha5.q = self.q
                    
                    #fig.step(0.05)
                    
                    
                    packets = b''
                    for index, position in enumerate(self.desired_position): 
                        device_id = index + 1
                        packets += BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([position]))
                        self.serial_port.write(packets)

                #addition of rotation needed to allow for grabber to accurately pick up and grab. 

                print(self.ReachAlpha5.q)
                
                # d = self.base_id_feedback_data() # Repeat to get accurate reading of data. Claw is 35mm long, so data will be 35mm away from target. 
                                                   # Uncommented due to bug, will crash movement due to NONE variable once in a while. Not too sure how to fix. 
                # print(f'Real End Effector: {d["end-effector-pos"]}')
            
                print(f"Simulated Coordiantes: {self.ReachAlpha5.fkine(self.ReachAlpha5.q)}") #Prints expected position end effector should be in the simulation. 
                                                                                              # can be cross referenced with self.base_id_feedback to determine the tolerance of the readings. 
                                                                                              # from testing is within 5mm tolerance 
                time.sleep(8)
                self.move_arm_to_pos(desired_positions=[0,self.ReachAlpha5.q[3],self.ReachAlpha5.q[2],self.ReachAlpha5.q[1],self.ReachAlpha5.q[0]]) # Grab object.
                time.sleep(4)
                i = 0
                self.trajectoryback= jtraj(self.ReachAlpha5.q, self.Origin,self.steps).q
                for self.q in self.trajectoryback: # Moves back to original position to start next coordinates. 
                                                   # (Can be removed to make arm go immediately to next coordinates)
                    
                    self.desired_position = [0, self.q[3] , self.q[2] , self.q[1], self.q[0]]
                    self.ReachAlpha5.q = self.q
                    #fig.step(0.05)
                    
                    packets = b''
                    for index, position in enumerate(self.desired_position):
                        device_id = index + 1
                        packets += BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([position]))
                        self.serial_port.write(packets)
                    
                time.sleep(5)
                self.reachable = True
                return self.reachable
            else:
                print('Unreachable Position: ', self.coordinates[self.index])
                self.reachable = False
                return self.reachable
   

    def JointCalculated(self, x, y, z): # calculate joint angles for each joint for the final position of the robot. 
                                        # this calculation is for OVERARM SOLUTION ONLY. 
                                        # See Reach Alpha 5 - Kinematics and Dynamic Properites for more info. pg.2-3
                                        # Set up of testing couldnt allow for underarm testing due to collision issues. 
                                        # see UnderArmJointCalculated(self,x,y,z) for info on implementation 
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
    
    def UnderArmJointCalculated(self,x,y,z): # Under arm joint calculations. 
                                             # used to utilise the full reach of the Reach Alpha 5
                                             # see Reach Alpha 5 Kinematics and Dynamic Properties page 2-3 for more information 
        a0 = 20
        a1 = 150.71
        a2 = 20

        d0 = 46.2
        d3 = -180

        R = sqrt(pow(x,2)+pow(y,2))

        l1 = a1
        l2 = sqrt(pow(a2,2)+pow(d3,2))
        l3 = sqrt(pow((R-a0),2)+pow((z-d0),2))

        theta0 = atan2(y,x) + pi
        theta1 = pi/2 + atan2((z-d0),(R-a0)) - acos((pow(l1,2)+pow(l3,2)-pow(l2,2))/(2*l1*l3)) - asin((2*a2)/l1)
        theta2 = acos((pow(l1,2)-pow(l3,2)+pow(l2,2))/(2*l1*l2)) - asin((2*a2)/l1) - asin(a2/l2)

        

    def move_arm_to_pos(self, desired_positions): # sends packets to the Reach Alpha 5 to move to joint angles specified.
        n = 0
        for position in desired_positions:
            self.packets += BPLProtocol.encode_packet(self.device_id[n], PacketID.POSITION, BPLProtocol.encode_floats([position]))
            n += 1
        self.serial_port.write(self.packets)
    


    def base_id_feedback_data(self): # Gets data from the Reach Alpha 5, used in the EStop and Kinematics 
                                     # returns voltage, and temperature for use in the Estop and end effector position for the kinematics to verify position is accurate
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
        
    
    def joint_id_feedback_data(self): # gets the joint angles of the real Reach Alpha 5, can be used to verify position of end effector using forward kinematics
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
    
    



    def send_disable_comms(self): #packet sent to Reach Alpha 5 to turn off. Used in Estop
        for device_id in self.device_id:
            self.packets += BPLProtocol.encode_packet(device_id, PacketID.MODE, bytes(1))
        self.serial_port.write(self.packets)

    def send_standby_comms(self): # packet sent to reset the Reach Alpha 5 to be able to move again. Used in Estop
        for device_id in self.device_id:
            self.packets += BPLProtocol.encode_packet(device_id, PacketID.MODE, bytes(0))
        self.serial_port.write(self.packets)
        time.sleep(1)

    def stop_arm_movement(self): # packet sent to stop Reach Alpha 5. Turns velocity to 0.
        for device_id in self.device_id:
            self.packets += BPLProtocol.encode_packet(device_id, PacketID.VELOCITY, BPLProtocol.encode_floats([0.0]))
        self.serial_port.write(self.packets)
        time.sleep(1)

    
    def log_data(self, file_title): # logs data that can be used. Checks the inputted coordinates are reachable 

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

if __name__ == '__main__':
    kin = Kinematics(COMPORT='COM6')
    Coordinates = [[-100, 100, 150]]
    kin.CalculateandMove(coordinates=Coordinates,ikine=False)