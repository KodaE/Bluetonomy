from bplprotocol import BPLProtocol, PacketID


# install pyserial with pip install pyserial
import time

import serial

if __name__ == '__main__':

    device_id = 4  # Joint B

    #serial_port_name = "COM10"
    serial_port_name = "COM6"

    serial_port = serial.Serial(serial_port_name, baudrate=115200, parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE, timeout=0)

    # Send to a position of 0.5 radians
    serial_port.write(BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([1.5])))

    print("Position changed")
    
    time.sleep(6)

    # Send a velocity command of 0.3
#    serial_port.write(BPLProtocol.encode_packet(device_id, PacketID.VELOCITY, BPLProtocol.encode_floats([0.3])))
 #   print("Velocity changed")

#    time.sleep(5)
    # Send a velocity of 0
 #   serial_port.write(BPLProtocol.encode_packet(device_id, PacketID.VELOCITY, BPLProtocol.encode_floats([0.0])))

#    print("Velocity stopped")

#    time.sleep(3)
    # Send a relative position command of 0
 #   serial_port.write(BPLProtocol.encode_packet(device_id, PacketID.RELATIVE_POSITION, BPLProtocol.encode_floats([-0.5])))
  #  print("relative position")

#    time.sleep(3)

    device_id_2 = 0x02

    #multiple joint control 
    serial_port.write(BPLProtocol.encode_packet(device_id_2, PacketID.RELATIVE_POSITION, BPLProtocol.encode_floats([-0.5])))
    print("New joint move")

