#!/usr/bin/env python3.7

import tkinter as Tk
# Implement the default Matplotlib key bindings.
import numpy as np
import time
import datetime
from PIL import Image, ImageTk
from sympy import E, EX
import rospy
from pub_classes import move_class
from std_msgs.msg import String
import os
import pandas as pd
from tkinter import ttk
import rosnode

os.chdir(os.path.expanduser("~/catkin_ws/src/Shaed_control/bci_user_pkg/scripts"))


class GUI:
    def __init__(self):
        # Create GUI
        self.root = Tk.Tk()
        # self.root = Toplevel
        self.root.wm_title("Fake BCI Control GUI")
        self.root.resizable(True, True)
        self.move_obj = move_class(queue=10)

        self.create_system_frame()

        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(0, weight=1)

    def create_system_frame(self):
        self.sys_frame = Tk.Frame(master=self.root, bg="dodger blue")
        self.sys_frame.grid(row=0, column=0, sticky="nsew")

        # Uni logo
        load = Image.open("logo.jpg")
        imsize = 100
        resized = load.resize((imsize, imsize), Image.ANTIALIAS)
        render = ImageTk.PhotoImage(image=resized)
        self.img = Tk.Label(master=self.sys_frame, image=render)
        self.img.image = render
        self.img.grid(row=0, column=0, columnspan=3)

        # Blue block
        self.object_button = Tk.Button(master=self.sys_frame, text="Blue Block", command=lambda: self.bring_object("blue_block"), bg="blue", padx=50, pady=20)
        self.object_button.grid(row=1, column=0, sticky="nsew")
        # Green block
        self.object_button = Tk.Button(master=self.sys_frame, text="Wood Block", command=lambda: self.bring_object("wood_block"), bg="brown", padx=50, pady=20)
        self.object_button.grid(row=1, column=1, sticky="nsew")
        # Red block
        self.object_button = Tk.Button(master=self.sys_frame, text="Red Tin", command=lambda: self.bring_object("red_tin"), bg="red", padx=50, pady=20)
        self.object_button.grid(row=1, column=2, sticky="nsew")

        # Home
        self.home_button = Tk.Button(master=self.sys_frame, text="Home", command=self.home, bg="green", padx=50, pady=20)
        self.home_button.grid(row=2, column=0, columnspan=3, sticky="nsew")

        # Quit button
        self.quit_button = Tk.Button(master=self.sys_frame, text="Quit", command=self._quit, bg="maroon", padx=50, pady=20)
        self.quit_button.grid(row=3, column=0, columnspan=3, sticky="nsew")

        # Adjust spacing of objects
        self.sys_frame.grid_columnconfigure(0, weight=1)
        self.sys_frame.grid_columnconfigure(1, weight=1)
        self.sys_frame.grid_columnconfigure(2, weight=1)

        self.sys_frame.grid_rowconfigure(0, weight=0)
        self.sys_frame.grid_rowconfigure(1, weight=1)
        self.sys_frame.grid_rowconfigure(2, weight=1)
        self.sys_frame.grid_rowconfigure(3, weight=1)

    def _quit(self):
        rospy.signal_shutdown('Quit Button')
        self.root.quit()     # stops mainloop
        self.root.destroy()  # this is necessary on Windows to prevent

    def home(self):
        self.move_obj.publish('home')

    def bring_object(self, object):
        self.move_obj.publish(f'bring_{object}')

    def update_gui(self):
        # Update gui
        #self.root.update_idletasks()
        self.root.update()


def run_gui():
    # Run ROS node
    frame_id = 'bci_gui_node'
    rospy.init_node(frame_id, anonymous=True)

    gui = GUI()

    while not rospy.is_shutdown():
        try:
            gui.update_gui()
        except Exception as e:
            print(e)


if __name__ == '__main__':
    # Run GUI
    try:
        run_gui()
    except rospy.ROSInterruptException:
        pass
