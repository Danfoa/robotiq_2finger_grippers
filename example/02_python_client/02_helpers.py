#!/usr/bin/env python3
"""Tutorial 02-2: command the gripper with the driver's static helpers.

`Robotiq2FingerGripperDriver` provides goto(), open() and close(), which build and send
the goal for you. They accept a SimpleActionClient and an optional `block` flag.

Usage:
    python3 02_helpers.py
"""

import actionlib
import rospy

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperAction
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq

ACTION_NAME = 'command_robotiq_action'

if __name__ == '__main__':
    rospy.init_node('example_helpers')

    client = actionlib.SimpleActionClient(ACTION_NAME, CommandRobotiqGripperAction)
    client.wait_for_server()

    Robotiq.close(client, speed=0.1, force=50, block=True)
    Robotiq.open(client, speed=0.1, force=50, block=True)

    # Move to an intermediate width, slowly and with low force.
    Robotiq.goto(client, pos=0.04, speed=0.02, force=10, block=True)

    # block=False returns immediately, so the script can do other work while the gripper moves.
    Robotiq.goto(client, pos=0.085, speed=0.05, force=10, block=False)
    rospy.loginfo('Goal sent, doing other work while the gripper moves...')
    client.wait_for_result()
    rospy.loginfo('Done')
