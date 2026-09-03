from Tkinter import *
import tkFont
import cv2
from PIL import ImageTk, Image
import os
import time
import threading

ASSET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "project_gui")

spaces = " " * 10
class project_gui(object):
    def __init__(self, drone_controller):
        self.drone_controller = drone_controller
        self.gui_found_tags = set()
        self.gui_is_flying = False
        self.flight_timer = 0
        self.old_photo_panel = None
        self.top = Tk()
        self.top.title("ros project")
        self.top.geometry("1350x900")
        names_str = "Itzchak Harel, Kobie Hazon and Roy Naor." + spaces
        reg_font = tkFont.Font(family="Sans", size=15)
        self.names_label = Label(self.top, text=names_str, font=reg_font)
        self.names_label.place(x=500, y=150)

        startup_str = "not flying yet! , what are you waiting for?" + spaces
        startup_str = Label(self.top, text=startup_str, font=reg_font)
        startup_str.place(x=500, y=200)

        status_header = "tag status:" + spaces
        self.status_label = Label(self.top, text=status_header, font=reg_font)
        self.status_label.place(x=700, y=350)

        gui_found_tags_header = "list of found tags:" + spaces
        self.gui_found_tags_label = Label(self.top, text=gui_found_tags_header, font=reg_font)
        self.gui_found_tags_label.place(x=700, y=450)

        self.error_font = None
        self.error_label = None
        self.battery_label = None
        self.height_label = None
        self.fly_label = None
        self.land_label = None

        self.drone_img = ImageTk.PhotoImage(
            Image.open(os.path.join(ASSET_DIR, "tello.png")).resize((650, 650), Image.ANTIALIAS))
        self.battery_img = ImageTk.PhotoImage(
            Image.open(os.path.join(ASSET_DIR, "battery.png")).resize((150, 100), Image.ANTIALIAS))
        self.drone_img_panel = Label(self.top, image=self.drone_img)
        self.battery_img_panel = Label(self.top, image=self.battery_img)
        self.drone_img_panel.pack(side="bottom", fill="both", expand="yes")
        self.battery_img_panel.pack(side="bottom", fill="both", expand="yes")
        self.fly_button = Button(self.top, text="fly", command=self.code_to_fly, height=3, width=7)
        self.land_button = Button(self.top, text="land", command=self.code_to_land, height=3, width=7)

        self.fly_button.pack()
        self.land_button.pack()
        self.fly_button.place(x=200, y=20)
        self.land_button.place(x=350, y=20)
        self.drone_img_panel.place(x=0, y=260)
        self.battery_img_panel.place(x=0, y=0)
        self.print_error("No Errors: Fly Safe." + spaces)
        self.print_the_battery(None)  # Update in app
        self.print_the_height(None)
        self.top.update()

    def update_photo(self, photo):
        if self.old_photo_panel:
            to_destroy = self.old_photo_panel
        else:
            to_destroy = None
        b, g, r = cv2.split(photo)
        img = cv2.merge((r, g, b))
        im = Image.fromarray(img)
        img_tk = ImageTk.PhotoImage(image=im)
        self.drone_img_panel = Label(self.top, image=img_tk)
        self.drone_img_panel.image = img_tk
        self.drone_img_panel.pack(side="bottom", fill="both", expand="yes")
        self.drone_img_panel.place(x=0, y=260)
        self.old_photo_panel = self.drone_img_panel
        if to_destroy is not None:
            to_destroy.destroy()

    def print_error(self, error_msg):
        self.error_font = tkFont.Font(family="Sans", size=10)
        self.error_label = Label(self.top, text=error_msg + " " * 100, font=self.error_font)
        self.error_label.place(x=700, y=300)

    def change_status(self, new_status):
        fnt = tkFont.Font(family="Sans", size=10)
        status_label = Label(self.top, text=new_status, font=fnt)
        status_label.place(x=700, y=410)

    def print_gui_found_tags(self):
        tag_location = 500
        for tag in self.gui_found_tags:
            tag_str = "we found tag number " + str(tag) + "." + spaces
            tag_font = tkFont.Font(family="Sans", size=11)
            tag_label = Label(self.top, text=tag_str, font=tag_font)
            tag_label.place(x=700, y=tag_location)
            tag_location += 38

    def print_the_battery(self, battery):
        if battery is None:
            battery_str = "?"
        else:
            battery_str = str(battery)
        self.battery_label = Label(self.top, text=battery_str, bg="lawn green")
        self.battery_label.place(x=56, y=38)

    def print_the_height(self, height):
        if height is None:
            height_str = "Height = " + "?"
        else:
            height_str = "Height = " + str(height) + spaces
        height_font = tkFont.Font(family="Sans", size=10, weight="bold")
        self.height_label = Label(self.top, text=height_str, font=height_font)
        self.height_label.place(x=1120, y=800)

    def place_nothing(self):
        some_str = "                                                            "
        some_font = tkFont.Font(family="Sans", size=15)
        some_label = Label(self.top, text=some_str, font=some_font)
        some_label.place(x=500, y=250)

    def code_to_fly(self):
        if self.gui_is_flying:
            fly_str = "already flying, sorry dude" + spaces
            fly_font = tkFont.Font(family="Sans", size=15)
            self.fly_label = Label(self.top, text=fly_str, font=fly_font)
            self.fly_label.place(x=500, y=250)
            self.top.after(1000, self.place_nothing)
        else:
            self.gui_is_flying = True
            self.drone_controller.takeoff_drone()
        self.top.update()

    def code_to_land(self):
        if self.gui_is_flying:
            self.gui_is_flying = False
            land_str = "The flight took " + str(self.flight_timer) + " seconds and it was fun." + spaces
            land_font = tkFont.Font(family="Sans", size=15)
            self.land_label = Label(self.top, text=land_str, font=land_font, fg="red")
            self.land_label.place(x=500, y=200)
            self.drone_controller.land_drone()
        else:
            land_str = "You need to fly before you can land :)" + spaces
            land_font = tkFont.Font(family="Sans", size=15)
            self.land_label = Label(self.top, text=land_str, font=land_font, fg="red")
            self.land_label.place(x=500, y=200)


if __name__ == "__main__":
    project_gui = project_gui()
    curr_time = time.time()
    i = 0
    while True:
        time.sleep(0.1)
        if project_gui.gui_is_flying:
            if time.time() - curr_time >= 1:
                timer_str = "time since launch: " + str(i) + " seconds!" + spaces
                project_gui.flight_timer = i
                timer_font = tkFont.Font(family="Sans", size=15)
                timer_label = Label(project_gui.top, text=timer_str, font=timer_font)
                timer_label.place(x=500, y=200)
                i += 1
                curr_time += 1
        project_gui.top.update()
