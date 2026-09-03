#!/usr/bin/env python

import time
import rospy
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from flock_msgs.msg import Flip
from flock_msgs.msg import FlightData
import sensor_msgs.msg
import threading
from tag_manager import tag_manager
from tag_manager import tag_state
from project_gui import project_gui
from Tkinter import *
import tkFont
import Queue
from enum import Enum

# To Do:
# 2. Apply cleanup function for ctrl-C  V
# 3. Organize the code fully

ceiling_height = 2
ceiling_dist = 0.2
altitude_epsilon = 0.12

# Drone specific parameters
tello_to_meter = 1.5
height_error_ratio = -0.2143

crawl_speed = 0.2

idle_speed = 1 / 2.
time_to_rotation = 4  # seconds
idle_rotation = 0.2

image_bridge = CvBridge()


class gui_operations(Enum):
    update_battery = 1
    update_altitude = 2
    add_found_tag = 3
    print_error = 4
    change_status = 5


class drone_controller(object):
    @property
    def is_overheat(self):
        return self._is_overheat

    @property
    def altitude(self):
        return self._altitude

    @property
    def battery(self):
        return self._battery

    def __init__(self):
        self.drone_control = {}
        self.curr_vel = (0, 0, 0, 0)
        self.gui_queue = Queue.Queue()
        self.battery = None
        self.is_overheat = False
        self.flight_time = None
        self.is_flying = False
        self.altitude = None
        self.idle_thread = threading.Thread(target=self.idle_movement)
        self.next_stop_timer = None
        self.tag_manager = None

        try:
            drone_id = rospy.get_param('ID')
        except KeyError:
            drone_id = 0
        topic_prefix = "/tello{}/".format(drone_id)
        self.drone_control['takeoff'] = rospy.Publisher(topic_prefix + 'takeoff', Empty, queue_size=10)
        self.drone_control['velocity'] = rospy.Publisher(topic_prefix + "cmd_vel", Twist, queue_size=10)
        self.drone_control['land'] = rospy.Publisher(topic_prefix + 'land', Empty, queue_size=10)
        self.drone_control['flip'] = rospy.Publisher(topic_prefix + 'flip', Flip, queue_size=10)
        rate = rospy.Rate(2)
        for name, pub in self.drone_control.items():
            while pub.get_num_connections() == 0 and not rospy.is_shutdown():
                rospy.loginfo("Waiting for subscriber " + name + " to connect...")
                rate.sleep()
        if rospy.is_shutdown():
            self.cleanup()
            exit(-1)
        self.gui = project_gui(self)
        self.gui_image = None
        rospy.Subscriber(topic_prefix + "flight_data", FlightData, self.drone_state_callback)
        rospy.Subscriber("/tag_detections_image", sensor_msgs.msg.Image, self.drone_image_callback)
        while self.battery is None and not rospy.is_shutdown():
            rate.sleep()
        self.tag_manager = tag_manager(self)
        self.bg_loop()
        rospy.spin()

    def bg_loop(self):
        curr_time = time.time()
        i = 0
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            if self.gui.gui_is_flying and time.time() - curr_time >= 1:
                timer_str = "Time since launch: " + str(i) + " seconds!             "
                self.gui.flight_timer = i
                timer_font = tkFont.Font(family="Sans", size=15)
                timer_label = Label(self.gui.top, text=timer_str, font=timer_font)
                timer_label.place(x=500, y=200)
                i += 1
                curr_time += 1
            if not self.gui_queue.empty():
                op_type, value = self.gui_queue.get()
                if op_type is gui_operations.update_battery:
                    self.gui.print_the_battery(value)
                elif op_type is gui_operations.update_altitude:
                    self.gui.print_the_height(value)
                elif op_type is gui_operations.print_error:
                    self.gui.print_error(value)
                elif op_type is gui_operations.add_found_tag:
                    self.gui.gui_found_tags = self.tag_manager.found_tags
                    self.gui.print_gui_found_tags()
                elif op_type is gui_operations.change_status:
                    self.gui.change_status(value)
            if self.gui_image is not None:
                self.gui.update_photo(self.gui_image)
            rate.sleep()
            self.gui.top.update()

    def target_changed(self, new_val):
        print("The state is now: " + str(new_val))
        if not self.is_flying:
            return
        status_string = new_val.value if new_val.value[-1] != " " else new_val.value + str(
            self.tag_manager.target_id) + ","
        self.gui_queue.put((gui_operations.change_status, status_string))
        if new_val is tag_state.finished_target:
            self.gui_queue.put((gui_operations.add_found_tag, self.tag_manager.target_id))
        if new_val is tag_state.no_target or new_val is tag_state.finished_target:
            if self.idle_thread.is_alive():
                self.idle_thread.do_run = False
                self.idle_thread.join()
                self.idle_thread = threading.Thread(target=self.idle_movement)
            self.idle_thread.do_run = True
            self.idle_thread.start()
        else:
            if self.idle_thread.is_alive():
                self.idle_thread.do_run = False
                self.idle_thread.join()
                self.idle_thread = threading.Thread(target=self.idle_movement)

    def drone_image_callback(self, data):
        cv_image = image_bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        cv_image = cv2.resize(cv_image, (640, 630))
        self.gui_image = cv_image

    def drone_state_callback(self, data):
        self.battery = data.battery_percent
        self.is_overheat = data.high_temperature
        print("Is Now Flying: " + str(self.is_flying))
        if self.flight_time is not None:
            if self.flight_time[1] + 1.1 < time.time():
                self.is_flying = False
            elif self.flight_time[0] != data.flight_time:
                self.is_flying = True
            if self.flight_time[0] != data.flight_time:
                self.flight_time = (data.flight_time, time.time())
        else:
            self.flight_time = (data.flight_time, time.time())
        self.altitude = data.altitude

    def idle_movement(self):  # Check if is_flying if not wait with option to exit
        print("starting idle movement")
        t = threading.currentThread()
        rate = rospy.Rate(20)
        top_border = ceiling_height - ceiling_dist - altitude_epsilon
        bottom_border = (ceiling_height - ceiling_dist) / 2. - altitude_epsilon
        altitude_change = (self.altitude, time.time())
        while True:
            self.publish_vel((idle_speed, 0, 0, 0), -1)
            while self.altitude < top_border:
                if not getattr(t, "do_run") or rospy.is_shutdown():
                    self.stop_movement()
                    print("Exiting Idle")
                    return
                rate.sleep()
                if altitude_change[1] + 2 < time.time() and altitude_change[0] == self.altitude:
                    self.publish_vel((idle_speed, 0, 0, 0), -1)
                    altitude_change = (self.altitude, time.time())
            self.publish_vel((-idle_speed, 0, 0, 0), -1)
            while self.altitude > bottom_border:
                if not getattr(t, "do_run") or rospy.is_shutdown():
                    self.stop_movement()
                    print("Exiting Idle")
                    return
                rate.sleep()
                if altitude_change[1] + 2 < time.time() and altitude_change[0] == self.altitude:
                    self.publish_vel((-idle_speed, 0, 0, 0), -1)
                    altitude_change = (self.altitude, time.time())
            self.publish_vel((0, 0, 0, idle_rotation), (1 / 2.) * (4. / idle_rotation))
            end_rotation_time = time.time() + (1 / 3.) * (4. / idle_rotation)
            while time.time() < end_rotation_time:
                if not getattr(t, "do_run") or rospy.is_shutdown():
                    self.stop_movement()
                    print("Exiting Idle")
                    return
                rate.sleep()

    @battery.setter
    def battery(self, value):
        if value is not None and value != self._battery:
            self.gui_queue.put((gui_operations.update_battery, value))
        if value is not None and value < 30:
            self.gui_queue.put((gui_operations.print_error, "Error: Battery is too low, charge before flying."))
        self._battery = value

    @is_overheat.setter
    def is_overheat(self, value):
        if value is True:
            self.gui_queue.put(gui_operations.print_error, "Error: Drone landed due to overheating.")
        self._is_overheat = value

    @altitude.setter
    def altitude(self, value):
        if value is not None and value != self._battery:
            self.gui_queue.put((gui_operations.update_altitude, value))
        self._altitude = value
        if value > ceiling_height - altitude_epsilon and self.curr_vel[0] > 0:
            self.stop_movement()
            print("Saved from crashing to ceiling")

    def publish_vel(self, command, time_to_stop):
        if not self.is_flying:
            print("Error: You need to fly before you can move!")
            return
        up, forward, right, spin = command
        print("Sending velocity " + str(command))
        self.curr_vel = command
        self.drone_control['velocity'].publish(create_vel(up, forward, right, spin))
        if time_to_stop > 0:
            if self.next_stop_timer is not None and self.next_stop_timer[1] >= time.time():
                self.next_stop_timer[0].cancel()
            self.next_stop_timer = (threading.Timer(time_to_stop, self.stop_movement), time.time() + time_to_stop)
            self.next_stop_timer[0].setDaemon(True)
            self.next_stop_timer[0].start()

    def publish_flip(self, flip_string):
        if not self.is_flying:
            print("Error: You need to fly before you can flip!")
            return
        if self.altitude > ceiling_height - ceiling_dist - altitude_epsilon:
            print("Error: Can't flip because you are too high!")
            return
        pub_flip = Flip()
        if flip_string == "flip-forward":
            pub_flip.flip_command = Flip.flip_forward
            self.drone_controller.publish_vel((-crawl_speed, 0, 0, 0), 1. / (crawl_speed * tello_to_meter))
            rospy.sleep(1. / (crawl_speed * tello_to_meter))
        if flip_string == "flip-back":
            pub_flip.flip_command = Flip.flip_back
        if flip_string == "flip-right":
            pub_flip.flip_command = Flip.flip_right
        if flip_string == "flip-left":
            pub_flip.flip_command = Flip.flip_left
        self.drone_control['flip'].publish(pub_flip)
        rospy.sleep(2)

    def takeoff_drone(self):
        if self.is_flying:
            return
        rate = rospy.Rate(0.5)
        self.stop_movement()
        rate.sleep()
        old_altitude = (self.altitude, time.time())
        while not rospy.is_shutdown():
            self.drone_control['takeoff'].publish()
            rate.sleep()
            if old_altitude[0] < self.altitude:
                break
            else:
                old_altitude = (self.altitude, time.time())
        rospy.sleep(2)
        self.target_changed(tag_state.no_target)

    def land_drone(self):
        if not self.is_flying:
            return
        if self.idle_thread.is_alive():
            self.idle_thread.do_run = False
            self.idle_thread.join()
            self.idle_thread = threading.Thread(target=self.idle_movement)
        rate = rospy.Rate(0.5)
        self.stop_movement()
        rate.sleep()
        old_altitude = (self.altitude, time.time())
        while not rospy.is_shutdown():
            self.drone_control['land'].publish()
            rate.sleep()
            if self.altitude == 0 or old_altitude[0] > self.altitude:
                break
            else:
                old_altitude = (self.altitude, time.time())

    def stop_movement(self):
        t = threading.currentThread()
        if self.next_stop_timer is not None and t != self.next_stop_timer[0]:
            self.next_stop_timer[0].cancel()
        if self.idle_thread.is_alive() and \
                (self.tag_manager.state is tag_state.no_target or self.tag_manager.state is tag_state.finished_target):
            return
        print("Stopping movement")
        print("Thread " + str(self.idle_thread.is_alive()))
        print("State " + str(self.tag_manager.state))
        self.curr_vel = (0, 0, 0, 0)
        self.drone_control['velocity'].publish(create_vel(0, 0, 0, 0))


def create_vel(up, forward, right, spin):
    new_speed = Twist()
    new_speed.linear.y = right
    new_speed.linear.x = forward
    new_speed.linear.z = up
    new_speed.angular.x = 0
    new_speed.angular.y = 0
    new_speed.angular.z = spin
    return new_speed
