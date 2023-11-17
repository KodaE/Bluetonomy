# from Reach_Kinematics import Kinematics
from Reach_Kinematics_Test import Kinematics
import time 
import keyboard
import threading
import time
from math import pi, radians




#----------------------------------------------- Setting Up Constant ------------------------------------------#
HOME = [0.0, 0.0, 0.0, 1.5207, pi]
WAY_POINT =  [0.0, 0.0, radians(180),radians(180) , radians(180)]
DATA_RECORD_FREQUENCY = 5


program_is_on = True
coordinates = []
data_log_file_title = time.strftime("%Y-%m-%d_%H_%M_%S", time.localtime(time.time()))
e_stop_flag = False

RA_km = Kinematics(COMPORT="COM6")
time.sleep(0.001)
RA_km.send_standby_comms()
RA_km.move_arm_to_pos(HOME)


#-------------------------------------------- Program Title -----------------------------------------------#


print('''______   ___   _____   _____                _                _  ______                                          
| ___ \ / _ \ |  ___| /  __ \              | |              | | | ___ \                                         
| |_/ // /_\ \|___ \  | /  \/  ___   _ __  | |_  _ __  ___  | | | |_/ /_ __  ___    __ _  _ __  __ _  _ __ ___  
|    / |  _  |    \ \ | |     / _ \ | '_ \ | __|| '__|/ _ \ | | |  __/| '__|/ _ \  / _` || '__|/ _` || '_ ` _ \ 
| |\ \ | | | |/\__/ / | \__/\| (_) || | | || |_ | |  | (_) || | | |   | |  | (_) || (_| || |  | (_| || | | | | |
\_| \_|\_| |_/\____/   \____/ \___/ |_| |_| \__||_|   \___/ |_| \_|   |_|   \___/  \__, ||_|   \__,_||_| |_| |_|
                                                                                    __/ |                       
                                                                                   |___/                        ''')


#----------------------------------------------------------------------------------------------------------#


def run_data_log_program():
    while program_is_on:
        time.sleep(DATA_RECORD_FREQUENCY)
        RA_km.log_data(data_log_file_title)


def run_main_program():
 
    time.sleep(DATA_RECORD_FREQUENCY)

    global program_is_on

    while program_is_on: 
            
            global coordinates
            
            user_action = input("Please select following button to perform action. Press a to coordinates, b to submit coordinate and c to terminate program, d to home arm: ")
            
            if user_action == "d":
            
                RA_km.send_standby_comms()
                #  RA_km.move_arm_to_pos(HOME)
                RA_km.move_arm_to_pos(HOME ) 
            

            elif user_action == "a":
                    try:
                            x_coordinate = float(input("Please enter x - coordinate: "))
                            y_coordinate = float(input("Please enter y - coordinate: "))
                            z_coordinate = float(input("Please enter z - coordinate: "))
                            co = x_coordinate, y_coordinate, z_coordinate
                            coordinates.append(co)
                            print(coordinates)
                    except ValueError:
                            print("Please enter an accurate coordinate")
            
            if user_action == "b":
                    print(f"Reach Alpha 5 will move to the following coordinate: {coordinates}")
                    for index, coordinate in enumerate(coordinates):
                        print(f"Currently moving to: {coordinate}")
                        try:
                            reachable = RA_km.CalculateandMove([coordinate],ikine=False)
                         
                            # coordinates.pop(index)

                            while reachable is False:
                                replacement_cood= []

                                user_action = input("Coordinate is unreachable, press a to change or b to skip to another coordinate: ")
                                if user_action == "a":
                                    try:
                                        x_coor = float(input("Please enter x - coordinate: "))
                                        y_coor = float(input("Please enter y - coordinate: "))
                                        z_coor = float(input("Please enter z - coordinate: "))
                                        coord_ = [x_coor, y_coor, z_coor]
                                        replacement_cood.append(coord_)
                                    except ValueError:
                                        print("Please enter an accurate coordinate")
                                    print(f"Currently moving to {replacement_cood}")
                                    reachable = RA_km.CalculateandMove(coordinates=replacement_cood,ikine=False)

                                    if reachable is True:
                                        print("Coordinate was reachable, finishing the rest of the coordinate! ")
                                elif user_action == "b":
                                    if (index + 1) == (len(coordinates)):
                                            print("No backlog of coordinates to move, please enter new set of coordinates")
                                    break
                        except Exception as e:
                                
                                print(f"RA 5 is not available now - {e}")
                                user_action = input("Press a to home, b to restart program and c to terminate program: ")
                                if user_action == "a":
                                
                                    RA_km.send_standby_comms()
                                    RA_km.move_arm_to_pos(HOME)

                                elif user_action == "b":
                                    RA_km.send_standby_comms()
                                    
                                elif user_action == "c":
                                     program_is_on = False
                                
                            
                    coordinates.clear()              
                    
            
            if user_action == "c":
                program_is_on = False
                            

def e_stop():
    global program_is_on
    start = time.time()
    voltage = 23
    temp = 40
    try:
        time.sleep(5)
        voltage = RA_km.base_id_feedback_data()["voltage"][0]
        temp = RA_km.base_id_feedback_data()["temp"][0]
    except:
        voltage is None
        temp is None

   
    # temp = RA_km.base_id_feedback_data()["temp"]
    # voltage = Ra

    while program_is_on:
        current_time = time.time()
        if current_time - start >= 20:
            readable_time = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(current_time))
            time.sleep(2)
            print("\n#---------------------- System Status ---------------------------#")
            print(f"\nE - Stop is alive {readable_time}")
            print(f"The current voltage is: {voltage}")
            print(f"The current temperature is {temp}\n")
            print("#-----------------------------------------------------------------#\n")
            start = current_time
    
        if  keyboard.is_pressed("z") or temp > 100 or voltage > 50:
            try:
                print("\ne - stop pressed\n")
               
                RA_km.send_disable_comms()
                time.sleep(4)
            except:
                pass
       
              
t1 = threading.Thread(target=e_stop)
t2 = threading.Thread(target=run_main_program)
t3 = threading.Thread(target=run_data_log_program)




t1.start()
t2.start()
t3.start()


t1.join()
t2.join()
t3.join()
     
        
            
        