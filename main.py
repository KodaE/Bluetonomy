import Reach_Control

RA5_control = Reach_Control.Reach_Control_Class(serial_port_name="COM6")



base_id = RA5_control.base_id_feedback_data()

print(base_id)

