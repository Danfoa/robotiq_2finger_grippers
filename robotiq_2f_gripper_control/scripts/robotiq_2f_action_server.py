#!/usr/bin/env python
"""--------------------------------------------------------------------
COPYRIGHT 2014 Stanley Innovation Inc.

Software License Agreement:

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 file   robotiq_85_driver

 brief  Node for Robotiq 85 communication

 Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
import rospy
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver, Robotiq2FingerSimulatedGripperDriver 
import actionlib
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction

class CommandGripperActionServer(object):

    def __init__(self, name, driver):
        self._action_name = name
        self._action_server = actionlib.SimpleActionServer(self._action_name, CommandRobotiqGripperAction, execute_cb=self.execute_cb, auto_start = False)
        self._driver = driver       # Get handle to the Gripper Control Server
        
        # Wait until gripper driver is ready to take commands.
        watchdog = rospy.Timer(rospy.Duration(15.0), self._connection_timeout, oneshot=True)
        while not rospy.is_shutdown() and not self._driver.is_ready:
            rospy.sleep(0.5)
            rospy.loginfo( self._action_name + ": Waiting for gripper to be ready...")
        
        watchdog.shutdown() 
        if not rospy.is_shutdown():
            self._processing_goal = False
            self._is_stalled = False

            self._action_server.start()
            rospy.loginfo("Robotiq server started")
    
    def _connection_timeout(self, event):
        rospy.logfatal("Gripper on port %s seems not to respond" % (self._driver._comport))
        rospy.signal_shutdown("Gripper on port %s seems not to respond" % (self._driver._comport))
      
    def execute_cb(self, goal_command):
        rospy.loginfo( (self._action_name + ": New goal received Pos:%.3f Speed: %.3f Force: %.3f Force-Stop: %r") % (goal_command.position, goal_command.speed, goal_command.force, goal_command.stop) )
        # Send incoming command to gripper driver
        self._driver.update_gripper_command(goal_command)
        # Wait until command is received by the gripper 
        rospy.sleep(rospy.Duration(0.1))
        # Set Action Server as active === processing goal...
        self._processing_goal = True        
        
        feedback = CommandRobotiqGripperFeedback()
        result = CommandRobotiqGripperResult()

        # Set timeout timer 
        watchdog = rospy.Timer(rospy.Duration(5.0), self._execution_timeout, oneshot=True)

        # Wait until goal is achieved and provide feedback
        rate = rospy.Rate( rospy.get_param('~rate', 30) )

        while not rospy.is_shutdown() and self._processing_goal and not self._is_stalled:             # While moving and not stalled provide feedback and check for result
            feedback = self._driver.get_current_gripper_status()
            self._action_server.publish_feedback( feedback )
            rospy.logdebug("Error = %.5f Requested position = %.3f Current position = %.3f" % (abs(feedback.requested_position - feedback.position), feedback.requested_position, feedback.position))
            # Check for completition of action 
            if( feedback.fault_status != 0 and not self._is_stalled):               # Check for errors
                rospy.logerr(self._action_name + ": fault status (gFLT) is: %d", feedback.fault_status)
                self._is_stalled = True
                self._action_server.set_aborted( feedback , (self._action_name + ": fault status (gFLT) is: %d" % feedback.fault_status))
                break
            if( abs(feedback.requested_position - feedback.position) < 0.005 or feedback.obj_detected):    # Check if position has been reached 
                watchdog.shutdown()                         # Stop timeout watchdog.
                self._processing_goal = False 
                self._is_stalled = False              
            rate.sleep()
        
        result = feedback                                   # Message declarations are the same 
        # Send result 
        if not self._is_stalled:
            rospy.logdebug(self._action_name + ": goal reached or object detected Pos: %.3f PosRequested: %.3f ObjectDetected: %r" % (goal_command.position, feedback.requested_position, feedback.obj_detected) )
            self._action_server.set_succeeded(result)  
        else:
            rospy.logerr(self._action_name + ": goal aborted Pos: %.3f PosRequested: %.3f ObjectDetected: %r" % (goal_command.position, feedback.requested_position, feedback.obj_detected) )
            self._action_server.set_aborted(result)  

        self._processing_goal = False 
        self._is_stalled = False 
    
    def _execution_timeout(self, event):
        rospy.logerr("%s: Achieving goal is taking too long, dropping current goal")
        self._is_stalled = True
        self._processing_goal = False

if __name__ == "__main__":

    rospy.init_node('robotiq_2f_action_server')

    # Get Node parameters
    comport = rospy.get_param('~comport','/dev/ttyUSB0')
    baud = rospy.get_param('~baud','115200')
    stroke = rospy.get_param('~stroke', 0.085)                # Default stroke is 85mm (Small C / 2 finger adaptive gripper model)
    joint_name = rospy.get_param('~joint_name', 'finger_joint')    
    sim = rospy.get_param('~sim', False)    

    # Create instance of Robotiq Gripper Driver
    if sim: # Use simulated gripper
        gripper_driver = Robotiq2FingerSimulatedGripperDriver( stroke=stroke, joint_name=joint_name)    
    else:   # Try to connect to a real gripper 
        gripper_driver = Robotiq2FingerGripperDriver( comport=comport, baud=baud, stroke=stroke, joint_name=joint_name)
    # Start action server 
    server = CommandGripperActionServer(rospy.get_namespace() + 'command_robotiq_action', gripper_driver)
    
    # Send and Request data from gripper and update joint state every `r`[Hz]
    r = rospy.Rate(rospy.get_param('~rate', 50))
    while not rospy.is_shutdown():
        gripper_driver.update_driver()
        r.sleep()

    rospy.spin()
    
    
