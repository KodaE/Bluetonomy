from bplprotocol import BPLProtocol, PacketID

import time
import serial

if __name__ == '__main__':

    serial_port_name = "COM3"
    #serial_port_name = "/dev/ttyUSB0"

    serial_port = serial.Serial(serial_port_name, baudrate=115200, parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE, timeout=0)

    # Desired position from end effector to base A -> G
    #desired_positions = [10.0, 0.5, 1.5707, 1.5707, 1.5707, 2.8, 3.1415]
    desired_positions = [0.0, 0.0, 0.0, 1.5707, 0.0]


   # desired_positions = [0,0,0,0,0,0,0]

    packets = b''
    for index, position in enumerate(desired_positions):
        device_id = index + 1
        packets += BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([position]))

    # Send joints to desired_positions
    serial_port.write(packets)

