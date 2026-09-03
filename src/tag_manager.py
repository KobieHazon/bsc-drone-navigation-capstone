#!/usr/bin/env python

import math
from aux import RangeDict
from apriltag_ros.msg import AprilTagDetectionArray
from flock_msgs.msg import Flip
import rospy
import time
import threading
from enum import Enum

# Flips need 50 percent battery
# To takeoff and fly you need more than 10 percent
# Less then 25 percent gives you reduced stability

# TO ADD:
# Target Lost Functionality V
# Stop velocity when it is not time limited: happens in the following:


# Drone specific parameters
tello_to_meter = 1.5
height_error_ratio = -0.2143
# camera_angle = 10.12 # Measured by hand, was replaced in this version for better accuracy

# Tag reaching parameters
tag_epsilon = 0.1
tag_rotate_epsilon = 0.02
stopping_distance = 1
target_height = 0.1

# Optimization parameters
TARGET_TIMEOUT = 0.5
step_fraction = 1 / 4.  # Maybe can be made higher
z_speed_points = [(2, 0.2), (3, 0.3), (4, 0.4), (5, 0.4), (6, 0.7)]
target_rotation_speed = 0.2  # 1 rotation speed is 1 round per 4 seconds
target_aligning_speed = 0.1
crawl_speed = 0.1
default_stopping_time = 0.2


def load_tags_file():
    ret = {}
    tags = rospy.get_param('/apriltag_ros_continuous_node/standalone_tags')
    for tag in tags:
        tag_id = tag['id']
        del tag['id']
        ret[tag_id] = tag
    return ret


def get_speed_functions():
    ret = RangeDict()
    ret[(0, z_speed_points[0][0])] = lambda x: -z_speed_points[0][1] if x < stopping_distance else z_speed_points[0][1]
    ret[(z_speed_points[-1][0], 1000)] = lambda x: z_speed_points[-1][1]
    for i in range(len(z_speed_points) - 1):
        slope = (z_speed_points[i + 1][1] - z_speed_points[i][1]) / (z_speed_points[i + 1][0] - z_speed_points[i][0])
        ret[(z_speed_points[i][0], z_speed_points[i + 1][0])] = \
            lambda x, bound_i=i: slope * (x - z_speed_points[bound_i][0]) + z_speed_points[bound_i][1]
    return ret


def target_reached(pose):
    if pose.z < (stopping_distance + tag_epsilon) and abs(get_true_height(pose)) <= tag_epsilon and abs(
            pose.x) <= tag_epsilon:
        return True
    return False


def get_true_height(pose):
    height_offset = pose.z * height_error_ratio
    true_height = pose.y - height_offset
    return true_height - target_height


class tag_state(Enum):
    no_target = "No target in view. Searching..."
    rotating_target = "Rotating to target "
    moving_target = "Moving towards target "
    aligning_target = "Aligning to the front tag "
    perform_command = "Performing command of tag "
    lost_target_right = "Lost target, rotating right."
    lost_target_left = "Lost target, rotating left."
    finished_target = "Finished handling tag "


class tag_manager(object):
    def __init__(self, controller):
        self.drone_controller = controller  # To publish velocity to drone
        self.target_id = None  # Current drone target
        self.found_tags = set()  # All the tag ids the drone reached
        self.last_command_finish_time = None  # The last time a tag command was sent
        self.last_vel = None  # The last velocity I sent, used for refocusing on lost tags
        self.state = tag_state.no_target  # The current state of the tag_manager. From above Enum
        self.z_speed_functions = get_speed_functions()  # RangeDict (see in aux.py file) for speed in z axis
        self.readable_tags = load_tags_file()  # Parsing of the apriltag's tag.yaml file to execute commands
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_detector_callback)
        # Assigning callback function for apriltag

    # Defining state as property so I can let the drone_controller know when it changed
    @property
    def state(self):  # Defining state as property so I can let the drone_controller know
        return self._state

    @state.setter
    def state(self, value):
        # Inform the drone_controller
        changed_thread = threading.Thread(target=self.drone_controller.target_changed, args=(value,))
        changed_thread.daemon = True
        changed_thread.start()
        self._state = value
        # Need to add handling here of losing target
        if value is tag_state.finished_target:  # Finished everything with tag
            self.found_tags.add(self.target_id)
            self.target_id = None
        if value is tag_state.perform_command:  # Perform flip
            self.perform_command()

    # Perform the flip assigned for the tag, won't work below 50% battery
    def perform_command(self):
        command_str = self.readable_tags[self.target_id]['name']
        if command_str != "":
            self.drone_controller.publish_flip(command_str)
        self.state = tag_state.finished_target

    # Write the apriltag detection message as a dictionary for our use
    def process_tags_list(self, tags):
        ret = {}
        if tags.detections:
            for tag in tags.detections:
                if tag.id[0] not in self.found_tags:
                    ret[tag.id[0]] = (tag.pose.pose.pose.position, tag.pose.pose.pose.orientation.z)
                    # tuple of position and spin angle
        return ret

    # Calculate the time and speed for movement to target
    def get_time_speed(self, distance):
        meter_speed = self.z_speed_functions[distance](distance)
        time_to_target = (distance / meter_speed) * step_fraction if distance >= z_speed_points[0][0] else \
            default_stopping_time
        tello_speed = meter_speed / tello_to_meter
        return time_to_target, tello_speed

    # Get closer to tag
    def move_to_tag(self, pose):
        y, z = 0, 0
        stopping_time = default_stopping_time
        if abs(pose.z - stopping_distance) > tag_epsilon:
            stopping_time, z = self.get_time_speed(pose.z)
        pose_y = get_true_height(pose)
        if stopping_time <= 1 and abs(pose_y) > tag_epsilon:
            y = -crawl_speed * math.copysign(1, pose_y)
        elif stopping_time > 1 and abs(pose_y) > tag_epsilon:
            y = -1 * (pose_y / stopping_time)
        self.last_command_finish_time = time.time() + stopping_time
        self.last_vel = (y, z, 0, 0)
        self.drone_controller.publish_vel((y, z, 0, 0), stopping_time)

    def rotate_drone(self, pose):  # If lost target during rotation
        angle = pose.x
        if abs(angle) < tag_rotate_epsilon:
            self.state = tag_state.moving_target
            self.drone_controller.stop_movement()
            return
        w = -target_rotation_speed * math.copysign(1, angle)
        pose_y = get_true_height(pose)
        y = 0
        if abs(pose_y) > tag_epsilon:
            y = -crawl_speed * math.copysign(1, pose_y)
        self.last_vel = (y, 0, 0, w)
        if self.drone_controller.curr_vel != self.last_vel:
            self.drone_controller.publish_vel(self.last_vel, 4.0 / target_rotation_speed)
        # need to add something to stop the command if stopped seeing the tag

    def align_drone(self, pose, wall_angle):
        if abs(wall_angle) < tag_epsilon:  # finished aligning
            if target_reached(pose):
                self.state = tag_state.perform_command
                self.drone_controller.stop_movement()
            else:
                self.state = tag_state.rotating_target
            return
        time_to_rotation = 4.0 / target_aligning_speed
        speed_in_radians = (2 * math.pi) / time_to_rotation
        motion_time = abs(wall_angle) / speed_in_radians
        full_rotation_time = motion_time * ((2 * math.pi) / abs(wall_angle))
        tangent_speed = (2 * math.pi * pose.z) / full_rotation_time
        rotation_speed = target_aligning_speed if wall_angle > 0 else -target_aligning_speed
        tangent_speed = -tangent_speed if wall_angle > 0 else tangent_speed
        self.last_vel = (0, 0, tangent_speed, rotation_speed * 2)
        self.drone_controller.publish_vel(self.last_vel, (motion_time * 1 / 2.))
        self.last_command_finish_time = time.time() + (motion_time * 1 / 2.)

    def tag_detector_callback(self, data):
        if self.state is tag_state.lost_target_left or self.state is tag_state.lost_target_right:
            tags = self.process_tags_list(data)
            if self.target_id in tags:  # Found lost tag
                self.state = tag_state.rotating_target
                self.drone_controller.stop_movement()
                self.last_command_finish_time = time.time()
            elif self.last_command_finish_time < time.time():
                self.state = tag_state.no_target
                self.last_command_finish_time = time.time()
        if self.last_command_finish_time is not None and self.last_command_finish_time >= time.time():
            return
        tags = self.process_tags_list(data)
        if tags:
            print("~~~~~~~~~~~~~~~~")
            if self.target_id not in tags.keys():  # Lost old target, assign a new one
                if self.state is tag_state.moving_target or self.state is tag_state.aligning_target or self.state is \
                        tag_state.rotating_target:
                    self.state = tag_state.lost_target_left if self.last_vel[2] > 0 else tag_state.lost_target_right
                    self.last_command_finish_time = time.time() + 4.0 / target_rotation_speed
                elif self.state is not tag_state.lost_target_left and self.state is not tag_state.lost_target_right:
                    self.state = tag_state.rotating_target
                    self.target_id = min(tags.keys())
            if self.target_id not in tags.keys():
                if self.state is tag_state.lost_target_right:
                    self.drone_controller.publish_vel((0, 0, 0, target_rotation_speed), 4.0 / target_rotation_speed)
                elif self.state is tag_state.lost_target_left:
                    self.drone_controller.publish_vel((0, 0, 0, target_rotation_speed), 4.0 / target_rotation_speed)
                return
            target_pos, wall_angle = tags[self.target_id]
            print("Wall Angle: " + str(wall_angle))
            if target_reached(target_pos) and self.state is not tag_state.aligning_target and self.state is not \
                    tag_state.perform_command and self.state is not tag_state.finished_target:  # Reached the tag
                self.state = tag_state.aligning_target
                return
            if self.state is not tag_state.aligning_target and wall_angle > 0.3:
                self.state = tag_state.aligning_target
            elif self.state is tag_state.moving_target and abs(target_pos.x) > tag_epsilon:
                self.state = tag_state.rotating_target
            elif self.state is tag_state.rotating_target:
                self.rotate_drone(target_pos)
            elif self.state is tag_state.moving_target:
                self.move_to_tag(target_pos)
            elif self.state is tag_state.aligning_target:
                self.align_drone(target_pos, wall_angle)
        elif self.target_id is not None and self.last_command_finish_time is not None and time.time() > \
                self.last_command_finish_time + TARGET_TIMEOUT:
            self.target_id = None
            self.state = tag_state.no_target
