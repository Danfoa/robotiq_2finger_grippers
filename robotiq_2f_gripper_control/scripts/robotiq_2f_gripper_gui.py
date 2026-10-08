#! /usr/bin/env python3
"""--------------------------------------------------------------------
Simple Tkinter GUI to open/close a Robotiq 2F gripper through the
`CommandRobotiqGripperAction` action server.

Parameters:
    action_name: Name of the action advertised by the gripper `ActionServer`.
    stroke: Max open finger distance of gripper [m] (0.085 for 2F-85).
    default_speed: Initial value of the speed slider [m/s].
    default_force: Initial value of the force slider.
--------------------------------------------------------------------"""

import threading

try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk

import rospy
import actionlib

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperAction, CommandRobotiqGripperGoal


class RobotiqGripperGUI(object):

    SPEED_MIN, SPEED_MAX = 0.01, 0.2
    FORCE_MIN, FORCE_MAX = 0.0, 200.0

    def __init__(self):
        action_name = rospy.get_param('~action_name', 'command_robotiq_action')
        self.stroke = rospy.get_param('~stroke', 0.085)
        default_speed = rospy.get_param('~default_speed', 0.1)
        default_force = rospy.get_param('~default_force', 100.0)

        self.client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
        self.server_ready = False
        self.status_text = 'Waiting for action server "%s"...' % action_name
        self.position = None
        self.lock = threading.Lock()

        self.root = tk.Tk()
        self.root.title('Robotiq 2F-85 Gripper')
        self.root.resizable(False, False)

        self.speed_var = tk.DoubleVar(value=default_speed)
        self.force_var = tk.DoubleVar(value=default_force)

        tk.Label(self.root, text='Speed [m/s]').grid(row=0, column=0, padx=10, pady=(10, 0), sticky='w')
        tk.Scale(self.root, variable=self.speed_var, from_=self.SPEED_MIN, to=self.SPEED_MAX,
                 resolution=0.01, orient=tk.HORIZONTAL, length=300).grid(row=1, column=0, columnspan=2, padx=10)

        tk.Label(self.root, text='Force').grid(row=2, column=0, padx=10, pady=(10, 0), sticky='w')
        tk.Scale(self.root, variable=self.force_var, from_=self.FORCE_MIN, to=self.FORCE_MAX,
                 resolution=1, orient=tk.HORIZONTAL, length=300).grid(row=3, column=0, columnspan=2, padx=10)

        self.open_btn = tk.Button(self.root, text='OPEN', width=12, height=2, state=tk.DISABLED,
                                  command=lambda: self.send(self.stroke))
        self.open_btn.grid(row=4, column=0, padx=10, pady=15)
        self.close_btn = tk.Button(self.root, text='CLOSE', width=12, height=2, state=tk.DISABLED,
                                   command=lambda: self.send(0.0))
        self.close_btn.grid(row=4, column=1, padx=10, pady=15)

        self.status_label = tk.Label(self.root, text=self.status_text, anchor='w', justify=tk.LEFT)
        self.status_label.grid(row=5, column=0, columnspan=2, padx=10, pady=(0, 10), sticky='w')

        threading.Thread(target=self._wait_for_server, daemon=True).start()
        self.root.protocol('WM_DELETE_WINDOW', self.root.quit)
        self.root.after(100, self._refresh)

    def _wait_for_server(self):
        while not rospy.is_shutdown():
            if self.client.wait_for_server(rospy.Duration(1.0)):
                with self.lock:
                    self.server_ready = True
                    self.status_text = 'Ready'
                return

    def _feedback_cb(self, feedback):
        with self.lock:
            self.position = feedback.position
            self.status_text = 'Moving' if feedback.is_moving else 'Holding object' if feedback.obj_detected else 'Idle'
            if feedback.fault_status:
                self.status_text = 'Fault: 0x%02X' % feedback.fault_status

    def _done_cb(self, state, result):
        if result is None:
            return
        with self.lock:
            self.position = result.position
            self.status_text = 'Done (object detected)' if result.obj_detected else 'Done'

    def send(self, position):
        goal = CommandRobotiqGripperGoal()
        goal.emergency_release = False
        goal.stop = False
        goal.position = position
        goal.speed = self.speed_var.get()
        goal.force = self.force_var.get()
        rospy.loginfo('Gripper goal: pos=%.3f speed=%.2f force=%.0f', goal.position, goal.speed, goal.force)
        self.client.send_goal(goal, done_cb=self._done_cb, feedback_cb=self._feedback_cb)

    def _refresh(self):
        if rospy.is_shutdown():
            self.root.quit()
            return
        with self.lock:
            text = self.status_text
            if self.position is not None:
                text += '   |   width: %.1f mm' % (self.position * 1000.0)
            state = tk.NORMAL if self.server_ready else tk.DISABLED
        self.status_label.config(text=text)
        self.open_btn.config(state=state)
        self.close_btn.config(state=state)
        self.root.after(100, self._refresh)

    def run(self):
        self.root.mainloop()


if __name__ == '__main__':
    rospy.init_node('robotiq_2f_gripper_gui')
    RobotiqGripperGUI().run()
