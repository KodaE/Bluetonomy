
from tkinter import *
from tkinter import messagebox
from Reach_Kinematics import Kinematics
import time
import threading

# ---------------------------- Constants-----------------------------------------#
coordinates = "fk u"
program_is_on = True
# ra5_kinematics = Kinematics(COMPORT="COM6")

# ---------------------------- Add Coordinate Function ---------------------------#

def add_coordinate():
    x = x_entry.get()
    y = y_entry.get()
    z = z_entry.get()
    
    try:
        coordinates.append([float(x), float(y), float(z)])
        coordinate_listbox.insert (END, f"({x}, {y}, {z})")
        x_entry.delete(0, END)
        y_entry.delete(0, END)
        z_entry.delete(0, END)
    except ValueError:
        
        messagebox.showinfo("INVALID COORDINATES!","Please re-enter coordinates")
        x_entry.delete(0, END)
        y_entry.delete(0, END)
        z_entry.delete(0, END)

# ----------------------------Home Function ----------------------------------------#

def return_home():
    # ra5_kinematics.move_arm_to_pos([0.0, 0.0, 0.0, 1.5207, 0.0])
    pass


# --------------------------- Run Coordinate --------------------------------------#

def run_coordinate():
    # for coord in coordinates:
    #     # ra5_kinematics.CalculateandMove(coord)
    #     pa

    while program_is_on:
        time.sleep(1)
        print(coordinates)

# --------------------------- Run Coordinate Thread -------------------------------#

def start_coordinate_thread():
    coordinate_thread = threading.Thread(target=run_coordinate)
    coordinate_thread.daemon = True
    coordinate_thread.start()
 

def run_thread():
    start_coordinate_thread()
# ---------------------------- Restart The Program -------------------------------#

def restart_program():
    global program_is_on
    program_is_on = True
   
    
# ---------------------------- UI SETUP -------------------------------------------#


window = Tk()
window.title("Reach Alpha 5 GUI Control")
window.geometry("400x400")

# --------------------------- E Stop Feature --------------------------------------#



def e_stop(event):
    global program_is_on
    # ra5_kinematics.stop_arm_movement()
    # ra5_kinematics.send_disable_comms()
    window.after_cancel(coordinates)
    print("E Stop pressed")
    program_is_on = False



# Creating label

x_label =Label(window, text="X (in meters):")
y_label =Label(window, text="Y (in meters):")
z_label =Label(window, text="Z (in meters):")

# Creating entry box

x_entry = Entry(window)
y_entry = Entry(window)
z_entry = Entry(window)

# Creating button 

add_button = Button(window, text="Add Coordinate", command=add_coordinate)

coordinate_listbox = Listbox(window)

restart_button = Button(window, text="Re Start", command=restart_program)

home_button = Button(window, text="Home", command=return_home)

run_coordinate_button = Button(window, text="Run", command=run_thread)



e_stop_button = Button(window, text="E - Stop")
e_stop_button.bind("<Button-1>", e_stop)



# Packing the element onto the canvas

x_label.pack()
x_entry.pack()

y_label.pack()
y_entry.pack()

z_label.pack()
z_entry.pack()


add_button.pack()
coordinate_listbox.pack()

home_button.pack()

restart_button.pack()
run_coordinate_button.pack()

e_stop_button.pack()



window.mainloop()