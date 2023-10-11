from Reach_Kinematics import Kinematics
import random
import time 

RA_km = Kinematics(COMPORT="COM6")


timeout_1_min = time.time() + 60
timeout_3_min = time.time() + 60 * 1
timeout_5_min = time.time() + 60 * 5

def generate_valid_coord():
    x = random.uniform(0, 0.3)
    y = random.uniform(0, 0.3)
    z = random.uniform(0, 0.3)
    co = [[x,y,z]]
    return co

def generate_invalid_coord():
    x = random.uniform(1.0, 2.0)
    y = random.uniform(1.0, 2.0)
    z = random.uniform(1.0, 2.0)
    co = [[x,y,z]]
    return co

chance_100 = 1
chance_75 = 0.75
chance_50 = 0.5

no_valid_coord = 0
no_invalid_coord = 0

no_arm_move = 0
no_invalid_coord_detect = 0

# 100 % valid input test

while True:

    if random.random() <= chance_100:
        co = generate_valid_coord
        no_valid_coord += 1

        RA_km.CalculateandMove(coordinates=co)
        no_arm_move +=1

    else:
        no_invalid_coord += 1
        if  RA_km.CalculateandMove(coordinates=co) == True:
            no_invalid_coord_detect -= 1


    if time.time() > timeout_5_min:

        print("Data For 5 Minute")
        print(f"no_valid_coord = {no_valid_coord}")
        print(f"no_invalid_coord = {no_invalid_coord}")
      
        print(f"no_arm_move = {no_arm_move}")
        print(f"no_invalid_coord_detect = {no_invalid_coord_detect}\n")

        break

    elif time.time() > timeout_3_min:
        
        print("Data For 3 Minute")
        print(f"no_valid_coord = {no_valid_coord}")
        print(f"no_invalid_coord = {no_invalid_coord}")
      
        print(f"no_arm_move = {no_arm_move}")
        print(f"no_invalid_coord_detect = {no_invalid_coord_detect}\n")

    elif time.time() > timeout_1_min:

        print("Data For 1 Minute")
        print(f"no_valid_coord = {no_valid_coord}")
        print(f"no_invalid_coord = {no_invalid_coord}")
      
        print(f"no_arm_move = {no_arm_move}")
        print(f"no_invalid_coord_detect = {no_invalid_coord_detect}\n")




