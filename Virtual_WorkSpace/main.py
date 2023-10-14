from Reach_Kinematics import *
import random
import time
import threading
import queue


Kin = Kinematics(COMPORT='COM4')

MovingRobotThread = threading.Thread(target=Kin.Run)
runningtime = threading.Thread(target=Kin.twothreadsrunning)

MovingRobotThread.start()
runningtime.start()




