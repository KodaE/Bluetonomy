import tkinter as tk
from tkinter import ttk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import customtkinter
from tkinter import messagebox

class Reach_Sim_GUI_Class:
    def __init__(self):
        self.flag = False
        self.coordinates = []
        
        
    def gui_run(self):
        self.root = tk.Tk()
        self.root.title("Reach Simulation GUI")
        self.root.geometry("400x400")
        self.setup_gui()
        self.root.mainloop()

    def setup_gui(self):
        # Create labels for the coordinate input fields
        x_label = ttk.Label(self.root, text="X (in meters):")
        y_label = ttk.Label(self.root, text="Y (in meters):")
        z_label = ttk.Label(self.root, text="Z (in meters):")
        range_label = ttk.Label(self.root,text="Range 0.4 meters")
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
        range_label.pack()
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

        if x and y and z:
            self.coordinates.append([float(x), float(y), float(z)])
            self.coordinates_listbox.insert(tk.END, f"({x}, {y}, {z})")
            self.x_entry.delete(0, tk.END)
            self.y_entry.delete(0, tk.END)
            self.z_entry.delete(0, tk.END)
        

    def submit_coordinates(self):
        self.placecoordinates = self.coordinates
        self.flag = True
        
        

    def delete_coordinates(self):
        self.coordinates_listbox.delete(0, tk.END)
        self.coordinates = []
        messagebox.showinfo("Complete", "Coordinates input is complete.")
        self.flag = False

if __name__ == "__main__":
    
    app = Reach_Sim_GUI_Class()
    app.gui_run()
    