import tkinter as tk
from tkinter import ttk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import customtkinter
from tkinter import messagebox

class Reach_Sim_GUI_Class:
    def __init__(self):
        self.flag = False
        self.changeflag = False
        self.coordinates = []
        self.index = 0
        self.change = None
        
    def run(self):
        self.root = tk.Tk()
        self.root.title("Reach Simulation GUI")
        self.root.geometry("400x400")
        self.setup_gui()
        #self.root.after(0,self.checkflag)
        self.root.mainloop()



    def setup_gui(self):
        # Create labels for the coordinate input fields
        x_label = ttk.Label(self.root, text="X (in meters):")
        y_label = ttk.Label(self.root, text="Y (in meters):")
        z_label = ttk.Label(self.root, text="Z (in meters):")
        #range_label = ttk.Label(self.root,text="Range 0.4 meters")
        # Create entry fields for coordinates
        self.x_entry = ttk.Entry(self.root)
        self.y_entry = ttk.Entry(self.root)
        self.z_entry = ttk.Entry(self.root)

        # Create a button to add coordinates
        add_button = ttk.Button(self.root, text="Add Coordinate", command=self.add_coordinate)

        # Create a label above the listbox
        coordinates_label = ttk.Label(self.root, text="Coordinates:")

        # Create a listbox to display coordinates
        self.coordinates_listbox = tk.Listbox(self.root)

        # Create a "Submit" button
        submit_button = ttk.Button(self.root, text="Submit", command=self.submit_coordinates)

        # Place widgets in the GUI
        #range_label.pack()
        x_label.pack()
        self.x_entry.pack()
        y_label.pack()
        self.y_entry.pack()
        z_label.pack()
        self.z_entry.pack()
        add_button.pack()
        coordinates_label.pack()
        self.coordinates_listbox.pack()
        submit_button.pack()


    def add_coordinate(self):
        x = self.x_entry.get()
        y = self.y_entry.get()
        z = self.z_entry.get()
        
        try:
            self.coordinates.append([float(x), float(y), float(z)])
            self.coordinates_listbox.insert(tk.END, f"({x}, {y}, {z})")
            self.x_entry.delete(0, tk.END)
            self.y_entry.delete(0, tk.END)
            self.z_entry.delete(0, tk.END)
        except ValueError:
            messagebox.showinfo("INVALID COORDINATES!","Please re-enter coordinates")
            self.x_entry.delete(0, tk.END)
            self.y_entry.delete(0, tk.END)
            self.z_entry.delete(0, tk.END)
            
    def call_for_coordinate_change(self,index):
        self.index = index
        self.change_coordinates_gui()
        

    def change_coordinates_gui(self):
        
        self.change = tk.Toplevel()
        self.change.title("Change Coordinates")
        self.change.geometry("400x400")
        x_label = ttk.Label(self.change, text="X (in meters):")
        y_label = ttk.Label(self.change, text="Y (in meters):")
        z_label = ttk.Label(self.change, text="Z (in meters):")
        
        # Create entry fields for coordinates
        self.change_x_entry = ttk.Entry(self.change)
        self.change_y_entry = ttk.Entry(self.change)
        self.change_z_entry = ttk.Entry(self.change)
        submit_button = ttk.Button(self.change, text="Submit", command=self.change_coordinates)
        x_label.pack()
        self.change_x_entry.pack()
        y_label.pack()
        self.change_y_entry.pack()
        z_label.pack()
        self.change_z_entry.pack()
        submit_button.pack()
        self.change.mainloop()

    def change_coordinates(self):
        x = self.change_x_entry.get()
        y = self.change_y_entry.get()
        z = self.change_z_entry.get()
        print('This is what self.change prints out: ',[x,y,z])
        try:
            x = float(x)
            y = float(y)
            z = float(z)
            self.coordinates[self.index] = [x, y, z]
            self.coordinates_listbox.delete(self.index)
            self.coordinates_listbox.insert(self.index, f"({x}, {y}, {z})")
            self.changeflag = False
            self.change.destroy()
            
        except ValueError:
            messagebox.showinfo("Changes Complete","No Changes Made")
            self.changeflag = False
            self.change.destroy()

    def submit_coordinates(self):
        self.placecoordinates = self.coordinates
        self.flag = True
        
        

    def delete_coordinates(self):
        self.coordinates_listbox.delete(0, tk.END)
        self.coordinates = []
        messagebox.showinfo("Complete", "Coordinates input is complete.")
        self.flag = False

    def checkflag(self):
        
        if self.changeflag:
            print('Flag Raised')
            self.change_coordinates_gui()
        self.root.after(1000,self.checkflag)
            

if __name__ == "__main__":
    
    app = Reach_Sim_GUI_Class()
    app.gui_run()
    