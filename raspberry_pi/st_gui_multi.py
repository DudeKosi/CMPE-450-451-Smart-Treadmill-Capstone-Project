'''
Author: Nick Myers
Email: nmyers3@umbc.edu
Date: 12/10/2023
Class: CMPE 450/451
Description: Smart Treadmill Graphical User Interface
'''

import customtkinter
import time
from serial_interface import *
import multiprocessing

# Main class for the GUI that sets default parameters
class TreadmillGUI:
    def __init__(self, master, timer_instance, metrics_instance) -> None:
        self.master = master

        customtkinter.set_appearance_mode("dark")
        customtkinter.set_default_color_theme("dark-blue")

        # Define the size of the window
        # width = self.master.winfo_screenwidth()
        # height = self.master.winfo_screenheight()
        width = 600
        height = 350
        self.master.geometry("%dx%d"% (width, height))

        # Title shown on window
        self.master.title("Smart Treadmill")

        self.timer = timer_instance
        self.metrics = metrics_instance

        # Define a frame
        self.frame = customtkinter.CTkFrame(master=master)
        self.frame.pack(fill="both", expand=True)

        # Start button
        # TODO: Want the start button to also start the treadmill
        self.start_button = customtkinter.CTkButton(self.frame, text="Start", fg_color="green", width=250, height=50, font=("Helvetica", 24), command=lambda: [self.timer.start_timer(), self.metrics.update_distance(), self.metrics.update_speed()])
        self.start_button.pack(padx=10, pady=10)

        # Stop button
        # TODO: Want the stop button to also stop the treadmill
        self.stop_button = customtkinter.CTkButton(self.frame, text="Stop", fg_color="red", width=250, height=50, font=("Helvetica", 24), command=lambda: [self.timer.stop_timer(), self.metrics.reset_distance()])
        self.stop_button.pack(padx=10, pady=10)

# Live elapsed time display
class LiveTimer:
    def __init__(self, master) -> None:
        self.master = master

        # Define a frame
        self.frame = customtkinter.CTkFrame(master=master)
        self.frame.pack(fill="both", expand=True)

        # Time display
        # TODO: Figure out how to export the time as needed for elevation parsing
        self.timer_label = customtkinter.CTkLabel(self.frame, text="Elapsed Time: 00:00:00", font=("Helvetica", 24))
        self.timer_label.pack(pady=10)

        self.running = False
        self.start_time = 0
        self.current_time = 0
    
    def start_timer(self):
        if not self.running:
            self.running = True
            self.start_time = time.time()
            self.update_timer()

    def stop_timer(self):
        if self.running:
            self.running = False

    def update_timer(self):
        if self.running:
            elapsed_time = int(time.time() - self.start_time)
            self.current_time = elapsed_time
            hours, remainder = divmod(elapsed_time, 3600)
            minutes, seconds = divmod(remainder, 60)
            time_str = "Elapsed Time: {:02}:{:02}:{:02}".format(hours, minutes, seconds)
            self.timer_label.configure(text=time_str)
            self.master.after(1000, self.update_timer)

class UserMetrics:
    def __init__(self, master, timer_instance, elevation) -> None:
        self.master = master

        self.timer = timer_instance
        #self.speed_mph = get_speed()
        self.speed_mph = 0
        self.distance = 0

        # Define a frame
        self.frame = customtkinter.CTkFrame(master=master)
        self.frame.pack(fill="both", expand=True)

        # Speed display
        self.speed_label = customtkinter.CTkLabel(self.frame, text="Speed: 0.0 MPH", font=("Helvetica", 24))
        self.speed_label.pack(pady=10)

        # Distance display
        self.distance_label = customtkinter.CTkLabel(self.frame, text="Distance: 0.00 Mi", font=("Helvetica", 24))
        self.distance_label.pack(pady=10)

        # Elevation display
        self.elevation_label = customtkinter.CTkLabel(self.frame, text="Elevation: 0% Grade", font=("Helvetica", 24))
        self.elevation_label.pack(pady=10)

    def update_distance(self):
        if self.timer.running:
            speed_mps = self.speed_mph / 3600.0
            # TODO: Need to fork so this can get the new speed every half second
            self.distance = self.distance + (speed_mps * 0.5)
            distance_str = "Distance: {:.2f} Mi".format(self.distance)
            self.distance_label.configure(text=distance_str)
            self.master.after(500, self.update_distance)

    def reset_distance(self):
        if not self.timer.running:
            self.distance = 0

    def update_speed(self):
        if self.timer.running:
            manager = multiprocessing.Manager()
            speed_dict = manager.dict()

            process = multiprocessing.Process(target=get_speed, args=(speed_dict,))
            process.start()
            process.join()
            print(speed_dict.values())

            self.speed_mph = float(speed_dict.values()[0])
            speed_str = "Speed: {:.1f} MPH".format(self.speed_mph)
            self.speed_label.configure(text=speed_str)
            self.master.after(500, self.update_speed)


if __name__ == "__main__":
    root = customtkinter.CTk()
    
    timer = LiveTimer(root)
    metrics = UserMetrics(root, timer, 2)

    setup = TreadmillGUI(root, timer, metrics)
    root.mainloop()
