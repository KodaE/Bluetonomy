from bplprotocol import BPLProtocol, PacketID, PacketReader

import time
import serial

if __name__ == '__main__':

    serial_port_name = "COM4"
    #serial_port_name = "/dev/ttyUSB0"

    serial_port = serial.Serial(serial_port_name, baudrate=115200, parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE, timeout=0)

    # Desired position from end effector to base A -> G
    #desired_positions = [10.0, 0.5, 1.5707, 1.5707, 1.5707, 2.8, 3.1415]
    q0 = ((350/180.0)*3.14)
    # print(q0)
    desired_positions = [10.0, 0.0, 1.0, 1.5707,1.2]


   # desired_positions = [0,0,0,0,0,0,0]

    packets = b''
    for index, position in enumerate(desired_positions):
        device_id = index + 1
        packets += BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([position]))

    # Send joints to desired_positions
    serial_port.write(packets)

    request_packet = b''

    
    device_ids = [0x01, 0x02, 0x03, 0x04, 0x05]

    frequency = 5

    packet_reader = PacketReader()

    # Request packets can be concatenated
    for device_id in device_ids:
        request_packet += BPLProtocol.encode_packet(device_id, PacketID.REQUEST, bytes(PacketID.POSITION))
        while True:

            # Send request packet
            serial_port.write(request_packet)

            position_responses = {}
            # Read request packets
            
            start_time = time.time()
            while time.time() < start_time + 1/frequency:
                time.sleep(0.01)
                try:
                    read_data = serial_port.read()
                except BaseException:
                    
                    read_data = b''
                if read_data != b'':
                    
                    packets = packet_reader.receive_bytes(read_data)
                    if packets:
                        
                        for packet in packets:
                            print(packet)
                            read_device_id, read_packet_id, data_bytes = packet
                            if read_device_id in device_ids and read_packet_id == PacketID.POSITION:
                                position = BPLProtocol.decode_floats(data_bytes)[0]

                                position_responses[read_device_id] = position
            print(f"Positions {position_responses}")
