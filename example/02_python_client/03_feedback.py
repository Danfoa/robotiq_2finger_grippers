#!/usr/bin/env python3
"""Tutorial 02-3: monitor the gripper while it moves.

Sends a non-blocking goal with feedback and done callbacks. The feedback and result messages
share the same fields: position, requested_position, is_moving, obj_detected, fault_status.

Parameters:
    ~position: Target finger distance [m] (default 0.0, i.e. fully closed)
    ~speed:    Motion speed [m/s] (default 0.02)
    ~force:    Grip force [%] (default 30)

Usage:
    python3 03_feedback.py _position:=0.02 _speed:=0.02
"""

import actionlib
import rospy

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperAction, CommandRobotiqGripperGoal

ACTION_NAME = 'command_robotiq_action'


def feedback_cb(feedback):
    rospy.loginfo('moving=%-5s position=%5.1f mm (target %5.1f mm)',
                  feedback.is_moving, feedback.position * 1000.0, feedback.requested_position * 1000.0)


def done_cb(state, result):
    if result.obj_detected:
        rospy.loginfo('Object detected, fingers stopped at %.1f mm', result.position * 1000.0)
    else:
        rospy.loginfo('Goal reached at %.1f mm', result.position * 1000.0)


if __name__ == '__main__':
    rospy.init_node('example_feedback')

    client = actionlib.SimpleActionClient(ACTION_NAME, CommandRobotiqGripperAction)
    client.wait_for_server()

    goal = CommandRobotiqGripperGoal()
    goal.position = rospy.get_param('~position', 0.0)
    goal.speed = rospy.get_param('~speed', 0.02)
    goal.force = rospy.get_param('~force', 30.0)

    client.send_goal(goal, done_cb=done_cb, feedback_cb=feedback_cb)
    client.wait_for_result()
