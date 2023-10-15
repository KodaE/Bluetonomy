import time
from Reach_Estop import *
import threading



class Reach_WatchDog_Class:

    def __init__(self,Estop) -> None:
        self.time = time.time()
        self.estop = Estop
        self.flag = False
        self.i = 0

    def run(self):
        while True:
            self.isEstopRunning()

    def isEstopRunning(self):
        self.time = time.time()
        if self.time == self.estop.time:
            self.flag = True
            #print("valid")
            self.i = 0
        else:
            self.i += 1
            if self.i > 10:
                #print('not valid')
                self.i = 0



if __name__ == "__main__":
    estop = Reach_Estop_Class()
    Watch = Reach_WatchDog_Class(Estop=estop)
    EstopThread = threading.Thread(target=estop.pulse)
    WatchDogThread = threading.Thread(target=Watch.run)
    EstopThread.start()
    WatchDogThread.start()
    
    