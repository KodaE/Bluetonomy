from Reach_Control import Reach_Control_Class as rc
import Reach_Kinematics as rk
from Reach_Estop import Reach_Estop_Class as re

class Reach_Int_Class:
    def __init__(self) -> None:
        self.x = 0
        self.flag = False

# Runs code if Reach_Interface.py is run directly
if __name__ == "__main__":
    print('Example')