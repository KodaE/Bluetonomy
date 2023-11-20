# from Reach_Kinematics import Kinematics
from Reach_Kinematics_Test import Kinematics
import time 
import keyboard
import threading
import time
from math import pi, radians




#----------------------------------------------- Setting Up Constant ------------------------------------------#

# Initise variables necessary to use the program
HOME = [0.0, 0.0, 0.0, 1.5207, pi]
WAY_POINT =  [0.0, 0.0, radians(180),radians(180) , radians(180)]
SYSTEM_FEEDBACK_FREQUENCY = 5
DATA_RECORD_FREQUENCY = 5
COMPORT_SETUP = "COM6"


program_is_on = True
coordinates = []
data_log_file_title = time.strftime("%Y-%m-%d_%H_%M_%S", time.localtime(time.time()))
e_stop_flag = False

# Initalise kinematics object in order to use the arm 
RA_km = Kinematics(COMPORT=COMPORT_SETUP)
time.sleep(0.001)

# Send standby comms and move the RA5 to home position prior to use
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


# Define function to  start logging the performance data. 

def run_data_log_program():
    while program_is_on:
        time.sleep(DATA_RECORD_FREQUENCY) # setting up rate to record data
        RA_km.log_data(data_log_file_title) # log data requires a variable to be used for the title of each csv file. 

# Def main program

def run_main_program():
 
    time.sleep(DATA_RECORD_FREQUENCY) # Sleep for a few seconds and then start system for all processes to finish executing, just in case

    global program_is_on # Intialise global variable as a flag to control the start and end of the program 

    while program_is_on: 
            
            global coordinates # Intialise a global coordinate variable so other threads can tap into the recorded data list.
            
            user_action = input("Please select following button to perform action. Press a to coordinates, b to submit coordinate and c to terminate program, d to home arm: ") # Prompting user to perform action
            
            # If user presses and return, send standby comms and return home.
            if user_action == "d":
            
                RA_km.send_standby_comms()
                RA_km.move_arm_to_pos(HOME ) 
            
            
            # If user enter a and return, prompt them to enter x y z coordinates
            elif user_action == "a":
                    
                    # Try and except used here to check the validity of coordinate input.
                    try:
                            x_coordinate = float(input("Please enter x - coordinate: "))
                            y_coordinate = float(input("Please enter y - coordinate: "))
                            z_coordinate = float(input("Please enter z - coordinate: "))
                            co = x_coordinate, y_coordinate, z_coordinate
                            coordinates.append(co)
                            print(coordinates)
                    except ValueError:
                            print("Please enter an accurate coordinate")
            
            # If user enter b and return, the coordinates will be submitted to the arm for execution. 
            if user_action == "b":
                    print(f"Reach Alpha 5 will move to the following coordinate: {coordinates}")
                    for index, coordinate in enumerate(coordinates):
                        print(f"Currently moving to: {coordinate}")

                        # Attempt to calculate the kinematics for trajectory, if reachable, the arm will move there accordingly.
                        try:
                            reachable = RA_km.CalculateandMove([coordinate],ikine=False)
                         
                            # While reachable is false, prompt the user to enter a different set of coordinates until correct

                            while reachable is False:
                                replacement_cood= []

                                user_action = input("Coordinate is unreachable, press a to change or b to skip to another coordinate: ")

                                # If the user enter a and return, it will prompt them to enter a new set of coordinates, it also checks of the validity of the input
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
                                    reachable = RA_km.CalculateandMove(coordinates=replacement_cood,ikine=False) # Attempt to reach this "newly inputed coordinate"

                                    if reachable is True:
                                        print("Coordinate was reachable, finishing the rest of the coordinate! ")
                                
                                # If user enter b and return, it will skip the next inputted coordinate in the backlog
                                elif user_action == "b":
                                    if (index + 1) == (len(coordinates)):
                                            print("No backlog of coordinates to move, please enter new set of coordinates")
                                    break
                        
                        # If the coordinate is reachable but the arm fail to move any reasons, the system will display the error message and ask them for next set of action. 
                        # After picking the next set of action, the program will loop back to the start, at line 66
                        except Exception as e:
                                
                                print(f"RA 5 is not available now - {e}") # RA5 not available, what do you want to do next
                                user_action = input("Press a to home, b to restart program and c to terminate program: ")

                                # If user enter a and return, send standby comms to restablish control and move the arm back to the home position. 
                                if user_action == "a":
                                
                                    RA_km.send_standby_comms()
                                    RA_km.move_arm_to_pos(HOME)

                                # If user enter b and return, the system will try to reestablish control of the arm

                                elif user_action == "b":
                                    RA_km.send_standby_comms()

                                # If the user enter c and return, the program will terminate
                                    
                                elif user_action == "c":
                                     program_is_on = False
                                
                            
                    coordinates.clear()  # After executing the list of coordinates, clear the data backlog  
                    
            
            if user_action == "c":
                program_is_on = False # If user enter c and return, it will terminate the program

# Define function to use for the e stop                        

def e_stop():
    global program_is_on # Define a global variable, so this thread can control the start and end of the whole program
    start = time.time()
    # Assuming starting voltage and temp will be at these values, ideally it should tries to get it from the machine
    # It's done like this so not too many processes tries access the RA5 at once when starting the program, thus crashing the program
    voltage = 23 
    temp = 40



    # Attempt to retrieve the voltage and temperature data of the arm every 5 seconds.
    try:
        time.sleep(SYSTEM_FEEDBACK_FREQUENCY)
        voltage = RA_km.base_id_feedback_data()["voltage"][0]
        temp = RA_km.base_id_feedback_data()["temp"][0]
    except:
        print("System data not available at the moment, please wait for next pulse")

    # When the function gets used, it will start an infinite loop that constantly check for system status
    # Also has the ability to execute disable commands when needs to

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
    
    # While the loop is running, if user hit the key 'a', it will execute the disable commands to the arm
    # This line will have the program jump back to line 135 as the arm is unable to move.

        if  keyboard.is_pressed("z") or temp > 100 or voltage > 50:
            try:
                print("\ne - stop pressed\n")
               
                RA_km.send_disable_comms()
                time.sleep(4)
            except:
                pass
       

# This part of the code tap into the threading modules and function to have all three program running concurrently and not sequentially. 

t1 = threading.Thread(target=e_stop)
t2 = threading.Thread(target=run_main_program)
t3 = threading.Thread(target=run_data_log_program)

t1.start()
t2.start()
t3.start()

t1.join()
t2.join()
t3.join()
     
        
            
        