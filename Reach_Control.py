from bplprotocol import BPLProtocol, PacketID
import time
import serial

class Reach_Control_Class:
    """This function is used control the arms, enter the following input desired positions,
    serial port name"""
    def __init__(self,serial_port_name): 
        self.serial_port_name = serial_port_name
      
        

    def move_arm_to_pos(self, desired_positions):
        device_id = ()
        serial_port = serial.Serial(serial_port_name = self.serial_port_name, baudrate=115200, parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE, timeout=0)

        for index, position in enumerate(desired_positions):
            device_id = index + 1
            packets += BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([position]))

        serial_port.write(packets)  

    

# Runs code if Reach_Control.py is run directly
if __name__ == "__main__":
    ReachComObj = Reach_Control_Class()

