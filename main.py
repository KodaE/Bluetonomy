from Reach_Kinematics import Kinematics
import random
import time 
import keyboard
import threading

HOME = [0.0, 0.0, 0.0, 1.5207, 0.0]

RA_km = Kinematics(COMPORT="COM6")
# RA_km.move_arm_to_pos(HOME)

timeout = time.time() + 5


e_stop_event = threading.Event()


def generate_valid_coord():
    x = random.uniform(0, 0.3)
    y = random.uniform(0, 0.3)
    z = random.uniform(0, 0.3)
    co = [[x,y,z]]
    return co

def run_function():
    while not e_stop_event.is_set():
        co = generate_valid_coord()
        print(RA_km.stop_flag)
        RA_km.CalculateandMove(coordinates=co)
        print("\n")


def e_stop():
    while True:
        # time.sleep(1)
        print("e-stop running")
        if keyboard.is_pressed("a"):
            RA_km.send_disable_comms()
            RA_km.stop_arm_movement()
            e_stop_event.set()
            RA_km.stop_flag = True
            break


t1 = threading.Thread(target=e_stop)
t2 = threading.Thread(target=run_function)

t1.start()
t2.start()
t1.join()
t2.join()








