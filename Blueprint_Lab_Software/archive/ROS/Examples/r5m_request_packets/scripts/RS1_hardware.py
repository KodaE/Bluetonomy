# containers constants and classes related specifically to RS1 hardware

import inspect

# from kivy.app import App
# from kivy.properties import NumericProperty

# class RS1_Hardware():
# 	serial=None
# 	deviceID=0x01
# 	name=''
# 	position=8.0
# 	velocity=NumericProperty(1.0)

class PacketID():
    MODE = 0x01
    VELOCITY = 0x02
    POSITION = 0x03
    OPENLOOP = 0x04
    CURRENT = 0x05

    RELATIVE_POSITION = 0x0E
    AUTO_LIMIT_CURRENT_DEMAND = 0x0F

    SUPPLYVOLTAGE = 0x90
    TEMPERATURE = 0x66
    REQUEST_PACKET = 0x60
    SERIAL_NUMBER = 0x61  # 97
    MODEL_NUMBER = 0x62  # 98
    VERSION = 0x63  # dec99
    DEVICE_ID = 0x64
    INTERNAL_HUMIDITY = 0x65
    INTERNAL_TEMPERATURE = 0x66
    DEVICE_TYPE = 0x67
    HARDWARE_STATUS = 0x68
    RUN_TIME = 0x69
    STATE_ESTIMATOR_STATUS = 0x71

    COMS_PROTOCOL = 0x80
    HEARTBEAT_FREQUENCY_SET = 0x92
    HEARTBEAT_SET = 0x91

    SAVE = 0x50
    LOAD = 0x51
    SET_DEFAULTS = 0x52
    FORMAT = 0x53

    CHANGE_PAGE = 0x54

    CURRENT_LIMIT = 0x12
    VELOCITY_LIMIT = 0x11
    POSITION_LIMIT = 0x10
    POSITION_GAIN = 0x13
    VELOCITY_GAIN = 0x14
    CURRENT_GAIN = 0x15

    POSITION_PARAMETERS = 0x20       # POSITION_SCALE, POSITION_OFFSET, POSITION_FILTER_COEF_ALPHA, POSITION_FILTER_COEF_BETA
    VELOCITY_PARAMETERS = 0x21      # VELOCITY_SCALE, VELOCITY_OFFSET, VELOCITY_FILTER_COEF_ALPHA, VELOCITY_FILTER_COEF_BETA
    CURRENT_PARAMETERS = 0x22        # CURRENT_SCALE, 	CURRENT_OFFSET, CURRENT_FILTER_COEF_ALPHA, CURRENT_FILTER_COEF_BETA
    INPUT_VOLTAGE_PARAMETERS = 0x23  # INPUT_VOLTAGE_SCALE, INPUT_VOLTAGE_OFFSET, INPUT_VOLTAGE_FILTER_COEF_ALPHA, INPUT_VOLTAGE_FILTER_COEF_BETA

    MOTOR_PARAMETERS = 0x30          # I_SE_MOTOR_VOLTAGE=140, I_SE_MOTOR_MAX_CURRENT, I_SE_MOTOR_RESISTANCE

    MAX_ACCELERATION = 0x40
    CURRENT_HOLD_THRESHOLD = 0x41
    COMPLIANCE_GAIN = 0x42
    COMPLIANCE_PARAMETERS = 0x44

    # 3 bytes for each version packet
    # [major, submaj, minor]
    ELECTRICAL_VERSION = 0x6A
    MECHANICAL_VERSION = 0x6B
    SOFTWARE_VERSION = 0x6C

    BOOTLOADER_STM = 0xFE
    BOOTLOADER = 0xFF

    KM_CONFIGURATION = 0xA0  # 4 bytes [km_enable, km_obstacle_enable, km_mount_orientation, km_frame]
    KM_END_POS = 0xA1  # 3x floats X_POS, Y_POS, Z_POS
    KM_END_VEL = 0xA2  # 3x floats X_VEL, Y_VEL, Z_VEL

    KM_BOX_OBSTACLE_00 = 0xA3  # 6 floats POINT1_X, POINT1_Y, POINT1_Z, POINT2_X, POINT2_Y, POINT2_Z
    KM_BOX_OBSTACLE_01 = 0xA4
    KM_BOX_OBSTACLE_02 = 0xA5
    KM_BOX_OBSTACLE_03 = 0xA6
    KM_BOX_OBSTACLE_04 = 0xA7
    KM_BOX_OBSTACLE_05 = 0xA8
    KM_CYLINDER_OBSTACLE_00 = 0xA9  # 7 floats POINT1_X, POINT1_Y, POINT1_Z, POINT2_X, POINT2_Y, POINT2_Z, RADIUS
    KM_CYLINDER_OBSTACLE_01 = 0xAA
    KM_CYLINDER_OBSTACLE_02 = 0xAB
    KM_CYLINDER_OBSTACLE_03 = 0xAC
    KM_CYLINDER_OBSTACLE_04 = 0xAD
    KM_CYLINDER_OBSTACLE_05 = 0xAE

    KM_FLOAT_PARAMETERS = 0xB0  # 6 floats:
    # M_ZERO, LAMBDA_TRANSLATE, LAMBDA_ROTATE, COLLISION_FWD_TIME_STEPS, SELF_COLLISION_RADIUS, END_EFF_COLLISION_TOLERANCE

    KM_JOINT_STATE = 0xB2
    KM_JOINT_STATE_REQUEST = 0xB3

    KM_DH_PARAMETERS_0 = 0xB8
    KM_DH_PARAMETERS_1 = 0xB9
    KM_DH_PARAMETERS_2 = 0xBA
    KM_DH_PARAMETERS_3 = 0xBB
    KM_DH_PARAMETERS_4 = 0xBC
    KM_DH_PARAMETERS_5 = 0xBD
    KM_DH_PARAMETERS_6 = 0xBE
    KM_DH_PARAMETERS_7 = 0xBF

    KM_POS_LIMIT_TRANSLATE  = 0xC0
    KM_VEL_LIMIT_TRANSLATE  = 0xC1
    KM_POS_LIMIT_YAW        = 0xC2
    KM_POS_LIMIT_PITCH      = 0xC3
    KM_POS_LIMIT_ROLL       = 0xC4
    KM_VEL_LIMIT_ROTATE     = 0xC5
    KM_POS_GAINS_TRANSLATE  = 0xC6
    KM_VEL_GAINS_TRANSLATE  = 0xC7
    KM_POS_GAINS_ROTATE     = 0xC8
    KM_VEL_GAINS_ROTATE     = 0xC9

    KM_JOINT_POS_0 = 0xD0
    KM_JOINT_POS_1 = 0xD1
    KM_JOINT_POS_2 = 0xD2
    KM_JOINT_POS_3 = 0xD3
    KM_JOINT_POS_4 = 0xD4
    KM_JOINT_POS_5 = 0xD5
    KM_JOINT_POS_6 = 0xD6
    KM_JOINT_POS_7 = 0xD7

    KM_COLLISION_FLAG = 0xAF
    KM_COLLISION_COORDS = 0xB1

    VELOCITY_DEMAND_INNER = 0x07
    POSITION_DEMAND_INNER = 0x08
    CURRENT_DEMAND_DIRECT = 0x09
    ICMU_INNER_PARAMETERS = 0x6F

    VELOCITY_LIMIT_INNER = 0x16
    POSITION_GAINS_INNER = 0x17
    VELOCITY_GAINS_INNER = 0x18
    CURRENT_GAINS_DIRECT = 0x19
    VELOCITY_INNER_PARAMETERS = 0x24

    ENCODER = 0xE0
    ENCODER_PARAMETERS = 0xE1

    TEST_PACKET = 0xE2

    MODE_SETTINGS = 0x43    # 2 ints: currentHold, compliance. 0: NC, 1:DISABLED, 2: ENABLED.

    RESET = 0xFD


class Mode():
    STANDBY = 0x00
    DISABLE = 0x01
    VELOCITY = 0x03
    POSITION = 0x02
    OPENLOOP = 0x05
    FACTORY = 0x08
    CURRENT = 0x04
    CALIBRATE = 0x07
    INIT = 0x06
    COMPLIANT = 0x09
    GRIP = 0x0A
    STALL_DRIVE = 0x0B
    CALIBRATE_TEST = 0x0C
    AUTO_LIMIT = 0x0D
    CALIBRATE_OUTER = 0x0E
    VELOCITY_CONTROL_INNER = 0x0F
    RELATIVE_POSITION = 0x12

