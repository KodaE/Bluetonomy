from bplprotocol import BPLProtocol, PacketID
import time #used for time-related operations
import serial #provides functionality for serial communication

# TASK 1:
# Create function in Reach_Control_Class,
# 6 parameters (joint angles) & sends joint angle data (packet) to robot arm.

class Reach_Control_Class:
    """This function is used control the arms, enter the following input desired positions,
    serial port name"""
    def __init__(self,serial_port_name): 
        self.serial_port_name = serial_port_name
        self.packets =b''
        self.device_id = [0x01, 0x02, 0x03, 0x04, 0x05]
        self.serial_port = serial.Serial(self.serial_port_name, baudrate=115200, parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE, timeout=0)
        self.ra5_base_id = 0x05
        self.request_timeout =  1
      
    def move_arm_to_pos(self, desired_positions):
        n = 0
        for position in desired_positions:
            self.packets += BPLProtocol.encode_packet(self.device_id[n], PacketID.POSITION, BPLProtocol.encode_floats([position]))
            n += 1
        self.serial_port.write(self.packets)
        print(self.packets)


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

 
    

   



# Runs code if Reach_Control.py is run directly
if __name__ == '__main__':
    ReachComObj = Reach_Control_Class()
    desired_positions = [10.0, 0.0, 0.0, 1.5707, 0.0]
    ReachComObj.send_joint_angles(desired_positions)

