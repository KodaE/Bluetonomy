from bplprotocol import BPLProtocol, PacketID, PacketReader


import time
import serial
from math import *

if __name__ == '__main__':

    serial_port_name = "COM6"
    #serial_port_name = "/dev/ttyUSB0"

    serial_port = serial.Serial(serial_port_name, baudrate=115200, parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE, timeout=0)

    # Desired position from end effector to base A -> G
    #desired_positions = [10.0, 0.5, 1.5707, 1.5707, 1.5707, 2.8, 3.1415]
    q0 = ((350/180.0)*3.14)
    # print(q0)
    print(degrees(pi/2))
    ThetaA = tanh(145.3/40)
    # desired_positions = [0.0, 0.0, 0, 0, pi]
    desired_positions = [0, 0.0, radians(180),radians(180) , radians(180)]
    # desired_positions = [0, pi *10, pi, pi, pi]
    # desired_positions = [0, 1, 0, 0, 0]
    #4.58535765, 1.96881245, 1.41269028, 1.90062291, 0.34648783
    #[3.92699039 3.51752243 0.66734183 6.25830984 2.38107041]

   # desired_positions = [0,0,0,0,0,0,0]

    #current_positions = [0,0,0,0,0]

    packets = b''
    for index, position in enumerate(desired_positions):
        device_id = index + 1
        packets += BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([position]))

    # Send joints to desired_positions
    serial_port.write(packets)

    request_packet = b''

    
    device_ids = [0x01, 0x02, 0x03, 0x04, 0x05]

    frequency = 5

    