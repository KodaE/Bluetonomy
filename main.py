import Reach_Control

RA5_control = Reach_Control.Reach_Control_Class(serial_port_name="COM4")

RA5_control.move_arm_to_pos([0.0, 0.0, 0.0, 1.5707, 0.0])


