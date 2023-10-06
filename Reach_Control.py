from bplprotocol import BPLProtocol, PacketID, PacketReader
import time
import serial

class Reach_Control_Class:
    """This function is used control the arms, enter the following input desired positions,
    serial port name"""
    def __init__(self,serial_port_name): 
        self.serial_port_name = serial_port_name
        self.packets =b''
        self.serial_port = serial.Serial(self.serial_port_name, baudrate=115200, parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE, timeout=0)
        self.ra5_base_id = 0x05
        self.request_timeout =  0.5
      

    def move_arm_to_pos(self, desired_positions):
        device_id = ()
    

        for index, position in enumerate(desired_positions):
            device_id = index + 1
            self.packets += BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([position]))

        self.serial_port.write(self.packets)

    # def kinematics_pos(self):
    #     packet_reader = PacketReader()
    #     self.serial_port.write(BPLProtocol.encode_packet(self.ra5_base_id, PacketID.REQUEST, bytes([PacketID.KM_END_POS])))
    #     start_time = time.time()

    #     position = None
    #     while True:
    #         time.sleep(0.0001)
    #         try:
    #             read_data = self.serial_port.read()
    #         except BaseException:
    #             read_data = b''
    #         if read_data != b'':
    #             packets = packet_reader.receive_bytes(read_data)
    #             if packets:
    #                 for packet in packets:
    #                     read_device_id, read_packet_id, data_bytes = packet
    #                     if read_device_id == self.ra5_base_id and read_packet_id == PacketID.KM_END_POS:
    #                         # Decode floats, because position is reported in floats
    #                         position = BPLProtocol.decode_floats(data_bytes)[:3]
    #                         return position             
                                                
    #                 if position is not None:
    #                     break

    #     # Timeout if no response is seen from the device.
    #         if time.time() - start_time > self.request_timeout:
    #             print("Request for Position timed out")
    #             break

    def base_id_feedback_data(self):
        packet_reader = PacketReader()
        self.serial_port.write(BPLProtocol.encode_packet(self.ra5_base_id, PacketID.REQUEST, bytes([PacketID.KM_END_POS, PacketID.VOLTAGE, PacketID.TEMPERATURE])))
        start_time = time.time()
        position = None
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
                                position = BPLProtocol.decode_floats(data_bytes)[:3]
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
            d["km-pos"] = position
            d["voltage"] = voltage
            d["temp"] = temp
            return d



      
if __name__ == "__main__":
    RA5_control = Reach_Control_Class(serial_port_name="COM6")

    result = RA5_control.base_id_data

    print(result)
        
    

    