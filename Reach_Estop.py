import time
import threading


class Reach_Estop_Class:
    def __init__(self) -> None:
        self.time = time.time()
        self.flag = False
        
    

    def run(self): 
        while True:
            self.time = time.time()
            
        

    def checktorque(self): #Check for the reading of torque during, will need to time it so that when checking for tolerance it isnt using the same path
        self.mutex.acquire()

        self.mutex.release()

    def checktemp(self):
        pass




if __name__ == "__main__":
    kin = Reach_Estop_Class()
    kin.run()