#!/usr/bin/env python3
"""Tutorial 04: command two grippers that live in different namespaces.

Each gripper has its own action server under its namespace
(/right_gripper/command_robotiq_action and /left_gripper/command_robotiq_action).
Goals are sent without blocking so both grippers move at the same time.

Usage:
    python3 dual_client.py
"""

import actionlib
import rospy

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperAction, CommandRobotiqGripperGoal

NAMESPACES = ('right_gripper', 'left_gripper')


def make_goal(position):
    goal = CommandRobotiqGripperGoal()
    goal.position = position
    goal.speed = 0.05
    goal.force = 30.0
    return goal


if __name__ == '__main__':
    rospy.init_node('example_dual_client')

    clients = {}
    for ns in NAMESPACES:
        clients[ns] = actionlib.SimpleActionClient('/%s/command_robotiq_action' % ns, CommandRobotiqGripperAction)
        clients[ns].wait_for_server()

    for position in (0.0, 0.085):
        for client in clients.values():
            client.send_goal(make_goal(position))  # Non-blocking
        for client in clients.values():
            client.wait_for_result()
        rospy.loginfo('Both grippers reached %.3f m', position)
