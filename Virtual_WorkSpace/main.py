from Reach_Kinematics import Kinematics
import random
import time
import threading
import queue
from Reach_Sim_GUI import Reach_Sim_GUI_Class


Gui = Reach_Sim_GUI_Class()
Kin = Kinematics(COMPORT='COM4',gui=Gui)
GuiThread = threading.Thread(target=Gui.gui_run)
MovingRobotThread = threading.Thread(target=Kin.Run)


MovingRobotThread.start()
GuiThread.start()




