"""Live ROS nodes and a real Tk window; no physical drone or network needed."""
from __future__ import print_function
import math
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest

import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Twist
from std_msgs.msg import Bool, Empty
from flock_msgs.msg import FlightData

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))
from project_gui import project_gui
from tello_slam_simulation import telloSlamSimulation
from tello_load_trajectory import TelloLoadTrajectory

def wait_until(predicate, timeout=12):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError('ROS condition timed out')

class SimulationTests(unittest.TestCase):
    def test_01_velocity_helpers_and_trajectory_files(self):
        sim = telloSlamSimulation.__new__(telloSlamSimulation)
        self.assertEqual(sim.clip_thresold_value(2, 0.05, 1), 1)
        self.assertEqual(sim.clip_thresold_value(-2, 0.05, 1), -1)
        self.assertEqual(sim.clip_thresold_value(0.01, 0.05, 1), 0)
        loader = TelloLoadTrajectory.__new__(TelloLoadTrajectory)
        from std_msgs.msg import String
        for filename in ['back_and_forth.csv', 'drone_lab_path.csv', 'salon_route_from_entrance.csv']:
            rows = loader.load_from_csv(String(os.path.join(ROOT, 'src', filename)))
            self.assertTrue(rows)
            self.assertTrue(all(len(row) == 4 and all(np.isfinite(x) for x in row) for row in rows))

    def test_02_real_tk_widgets(self):
        class Driver(object):
            taken_off = 0
            landed = 0
            def takeoff_drone(self): self.taken_off += 1
            def land_drone(self): self.landed += 1
        driver = Driver()
        gui = project_gui(driver)
        try:
            gui.fly_button.invoke()
            self.assertTrue(gui.gui_is_flying)
            self.assertEqual(driver.taken_off, 1)
            gui.fly_button.invoke()
            self.assertEqual(driver.taken_off, 1)
            gui.print_the_battery(73)
            self.assertEqual(str(gui.battery_label.cget('text')), '73')
            gui.print_the_height(0.8)
            gui.update_photo(np.zeros((32, 32, 3), dtype=np.uint8))
            gui.top.update()
            gui.land_button.invoke()
            self.assertFalse(gui.gui_is_flying)
            self.assertEqual(driver.landed, 1)
        finally:
            gui.top.destroy()

    def test_03_live_simulator_and_closed_loop_controller(self):
        work = tempfile.mkdtemp(prefix='drone-simulation-')
        children, logs = [], []
        values = {}
        def remember(key):
            return lambda message: values.update({key: message})
        def launch(filename):
            log = tempfile.TemporaryFile()
            logs.append(log)
            child = subprocess.Popen([sys.executable, os.path.join(ROOT, 'src', filename)], cwd=work, stdout=log, stderr=log)
            children.append(child)
            return child
        subscriptions = [
            rospy.Subscriber('/ccmslam/PoseOutClient', PoseStamped, remember('pose')),
            rospy.Subscriber('/tello/flight_data', FlightData, remember('flight')),
            rospy.Subscriber('/tello/real_world_pos', PoseStamped, remember('world')),
        ]
        velocity = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        takeoff = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        allow = rospy.Publisher('/tello/allow_slam_control', Bool, queue_size=1, latch=True)
        target = rospy.Publisher('/tello/command_pos', Pose, queue_size=1, latch=True)
        try:
            simulator = launch('tello_slam_simulation.py')
            wait_until(lambda: 'pose' in values and takeoff.get_num_connections() > 0)
            takeoff.publish(Empty())
            wait_until(lambda: 'flight' in values and values['flight'].altitude >= 0.7)
            start = values['pose'].pose.position.x
            command = Twist()
            command.linear.x = 0.4
            deadline = time.time() + 1.5
            while time.time() < deadline:
                velocity.publish(command)
                time.sleep(0.05)
            velocity.publish(Twist())
            time.sleep(0.2)
            self.assertGreater(values['pose'].pose.position.x - start, 0.07)
            controller = launch('tello_slam_control.py')
            wait_until(lambda: 'world' in values and target.get_num_connections() > 0 and allow.get_num_connections() > 0)
            goal = Pose()
            goal.position.x = values['world'].pose.position.x + 0.6
            goal.position.y = values['world'].pose.position.y
            goal.position.z = values['world'].pose.position.z
            goal.orientation.w = 1
            target.publish(goal)
            allow.publish(Bool(True))
            wait_until(lambda: abs(goal.position.x - values['world'].pose.position.x) < 0.20, timeout=18)
            allow.publish(Bool(False))
            velocity.publish(Twist())
            self.assertIsNone(simulator.poll())
            self.assertIsNone(controller.poll())
            print('Live takeoff, motion feedback, and closed-loop target convergence passed.')
        finally:
            for child in reversed(children):
                if child.poll() is None:
                    child.send_signal(signal.SIGINT)
            deadline = time.time() + 3
            while time.time() < deadline and any(c.poll() is None for c in children):
                time.sleep(0.05)
            for child in children:
                if child.poll() is None: child.kill()
                child.wait()
            for log in logs:
                log.seek(0)
                output = log.read()
                if b'Traceback' in output: print(output.decode('utf-8', 'replace'))
                log.close()
            for subscription in subscriptions: subscription.unregister()
            shutil.rmtree(work)

if __name__ == '__main__':
    rospy.init_node('drone_simulation_test', anonymous=True)
    unittest.main(verbosity=2)
