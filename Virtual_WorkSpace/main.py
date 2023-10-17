from Reach_Kinematics import Kinematics
import random
import time
import threading
import queue
from Reach_Sim_GUI import Reach_Sim_GUI_Class
from Reach_WatchDog import Reach_WatchDog_Class
from Reach_Estop import Reach_Estop_Class


Estop = Reach_Estop_Class()
WatchDog = Reach_WatchDog_Class(Estop=Estop)
Gui = Reach_Sim_GUI_Class()
Kin = Kinematics(COMPORT='COM4',gui=Gui,estop=Estop ,watchdog=WatchDog)
GuiThread = threading.Thread(target=Gui.run)
MovingRobotThread = threading.Thread(target=Kin.Run)
EstoppulseThread = threading.Thread(target=Estop.pulse)
WatchDogThread = threading.Thread(target=WatchDog.run)
MovingRobotThread.start()
GuiThread.start()
#EstoppulseThread.start()
#WatchDogThread.start()




