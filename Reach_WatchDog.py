import time
from Reach_Estop import *
import threading



class Reach_WatchDog_Class:

    def __init__(self,Estop) -> None:
        
        self.estop = Estop
        self.flag = False
        self.i = 0

    def run(self):
        while True:
            self.isEstopRunning()

    def isEstopRunning(self):
        if not self.estop.event.wait(20):
            print("Not running")

        else:
            print("Running")



if __name__ == "__main__":
    estop = Reach_Estop_Class()
    Watch = Reach_WatchDog_Class(Estop=estop)
    EstopThread = threading.Thread(target=estop.run)
    WatchDogThread = threading.Thread(target=Watch.run)
    EstopThread.start()
    WatchDogThread.start()
    