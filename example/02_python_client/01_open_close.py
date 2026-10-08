#!/usr/bin/env python3
"""Tutorial 02-1: open and close the gripper with a raw action goal.

Builds a CommandRobotiqGripperGoal by hand and sends it with a SimpleActionClient.
This is the lowest-level way to command the gripper; the other examples wrap it.

Usage:
    python3 01_open_close.py
"""

import actionlib
import rospy

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperAction, CommandRobotiqGripperGoal

ACTION_NAME = 'command_robotiq_action'
STROKE = 0.085  # [m] Maximum finger distance of the 2F-85


def make_goal(position, speed=0.05, force=50.0):
    goal = CommandRobotiqGripperGoal()
    goal.emergency_release = False
    goal.stop = False
    goal.position = position  # [m] Distance between the fingers
    goal.speed = speed        # [m/s] Valid range: 0.013 - 0.1
    goal.force = force        # [%] Valid range: 0 - 100
    return goal


if __name__ == '__main__':
    rospy.init_node('example_open_close')

    client = actionlib.SimpleActionClient(ACTION_NAME, CommandRobotiqGripperAction)
    rospy.loginfo('Waiting for action server "%s"...', ACTION_NAME)
    client.wait_for_server()

    for name, position in (('close', 0.0), ('open', STROKE)):
        rospy.loginfo('Sending goal: %s (position=%.3f m)', name, position)
        client.send_goal(make_goal(position))
        client.wait_for_result()  # Blocks until the gripper reaches the goal
        result = client.get_result()
        rospy.loginfo('Reached %.1f mm (object detected: %s)', result.position * 1000.0, result.obj_detected)
