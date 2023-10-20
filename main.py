from Reach_Kinematics import Kinematics
import random
import time 
import keyboard
import threading
import time



#----------------------------------------------- Setting Up Constant ------------------------------------------#
HOME = [0.0, 0.0, 0.0, 1.5207, 0.0]
RA_km = Kinematics(COMPORT="COM6")
coord = [[100, 100, 100]]
program_is_on = True
coordinates = []


while program_is_on: 
        
        user_action = input("Please select following button to perform action. Press a to coordinates, b to submit coordinate and c to terminate program: ")
        
        if user_action == "a":
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
                    reachable = RA_km.CalculateandMove([coordinate])
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
                            reachable = RA_km.CalculateandMove(replacement_cood)

                            if reachable is True:
                                print("Coordinate was reachable, finishing the rest of the coordinate! ")
                        elif user_action == "b":
                              if (index + 1) == (len(coordinates)):
                                    print("No backlog of coordinates to move, please enter new set of coordinates")
                              break
                        
                coordinates.clear()              
                   
        
        if user_action == "c":
            program_is_on = False
                            
   
            
        
            
        