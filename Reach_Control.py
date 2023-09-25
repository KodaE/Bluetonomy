from bplprotocol import BPLProtocol, PacketID
import time #used for time-related operations
import serial #provides functionality for serial communication

# TASK 1:
# Create function in Reach_Control_Class,
# 6 parameters (joint angles) & sends joint angle data (packet) to robot arm.

class Reach_Control_Class:
    # Initialise class, optional serial port argument) 
    def __init__(self, serial_port_name="COM4"): # Change the default port if needed for communication
        self.serial_port = serial.Serial(serial_port_name, buadrate=115200, parity=serial.PARITY_NONE, 
                                         stopbits=serial.STOPBITS_ONE, timeout=0) # Creates instance variable, represents serial port
    

    def send_joint_angles(self, joint_angles): #defines method send_joint_angles. Takes joint_angles parameter
        if len(joint_angles) != 6:
            raise ValueError("Exactly 6 joint angles ar required.") #Ensures only 6 parameters were received and read.
        packets = b''   #initialises empty bytes object. Used to accumulate encoded packets to be sent to the arm.
        for index, position in enumerate(joint_angles): #starts a for loop to iterate through joint angles from joint_angles list
            device_id = index + 1 
            packets +- BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([position])) 
            # ^^ Line encodes each joint angle and adds it to packets variable. BPLProctol part encodes joint angles and Device ID.

        self.serial_port.write(packets) 
        # ^^ After encoding all angles, line writes to serial port, sends angle to arm and joints to desired positions



# Runs code if Reach_Control.py is run directly
if __name__ == "__main__":
    ReachComObj = Reach_Control_Class()
    desired_positions = [10.0, 0.0, 0.0, 1.5707, 0.0]
    ReachComObj.send_joint_angles(desired_positions)

