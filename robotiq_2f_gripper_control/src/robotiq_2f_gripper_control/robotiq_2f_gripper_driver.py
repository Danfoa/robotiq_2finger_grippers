#! /usr/bin/env python
"""--------------------------------------------------------------------
COPYRIGHT 2015 Stanley Innovation Inc.

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
 
 file   robotiq_85_driver.py

 brief  Driver for Robotiq 85 communication

 Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
from robotiq_2f_gripper import Robotiq2FingerGripper
from robotiq_2f_gripper_msgs.msg import RobotiqGripperCommand, RobotiqGripperStatus, CommandRobotiqGripperGoal
from sensor_msgs.msg import JointState
import numpy as np
import rospy
from enum import Enum

class Robotiq2FingerGripperDriver:
    def __init__(self, comport = '/dev/ttyUSB0', baud = '115200', stroke = 0.085, joint_name='finger_joint'):
        self._comport = comport
        self._baud = baud
        self._joint_name = joint_name          

        # Instanciate and open communication with gripper.
        self._gripper = Robotiq2FingerGripper(device_id=0, stroke=stroke, comport=self._comport, baud=self._baud)

        if not self._gripper.init_success:
            rospy.logerr("Unable to open commport to %s" % self._comport)
            return
        else:
            rospy.loginfo("Connection to gripper with stroke %.3f[m] on port %s successful" % ( self._gripper.stroke, self._comport))

        # # Subscribe to gripper commands  
        # rospy.Subscriber(rospy.get_namespace()+"cmd", RobotiqGripperCommand, self.update_gripper_command, queue_size=10)
        # # Instantiate publisher of gripper status
        # self._gripper_pub = rospy.Publisher(rospy.get_namespace()+"stat", RobotiqGripperStatus, queue_size=10)
        # # Instantiate publisher of joint state 
        self._gripper_joint_state_pub = rospy.Publisher("/joint_states" , JointState, queue_size=10)        

        self._seq = 0
        self._prev_joint_pos = 0.0
        self._prev_joint_state_time = rospy.get_time() 
        self._driver_state = 0
        self.is_ready = False
        
        if not self._gripper.process_status_cmd():
            rospy.logerr("Failed to contact gripper %d ... ABORTING" % self._gripper.device_id)
            return                
                
        self._run_driver()
        
    def _clamp_position(self,cmd):
        out_of_bouds = False
        if (cmd <= 0.0):
            out_of_bouds = True
            cmd_corrected = 0.0
        elif (cmd > self._gripper.stroke):
            out_of_bouds = True
            cmd_corrected = self._gripper.stroke
        if(out_of_bouds):
            rospy.loginfo("Position (%.3f[m]) out of limits for %d[mm] gripper: \n- New position: %.3f[m]\n- Min position: %.3f[m]\n- Max position: %.3f[m]" % (cmd, self._gripper.stroke*1000, cmd_corrected, 0.0, self._gripper.stroke))
            cmd = cmd_corrected
        return cmd

    def _clamp_speed(self,cmd):
        out_of_bouds = False
        if (cmd <= 0.013):
            out_of_bouds = True
            cmd_corrected = 0.013
        elif (cmd > 0.101):
            out_of_bouds = True
            cmd_corrected = 0.1
        if(out_of_bouds):
            # rospy.loginfo("Speed (%.3f[m/s]) out of limits for %d[mm] gripper: \n- New speed: %.3f[m/s]\n- Min speed: %.3f[m/s]\n- Max speed: %.3f[m/s]" % (cmd, self._gripper.stroke*1000, cmd_corrected, 0.013, 0.1))
            cmd = cmd_corrected
        return cmd
    
    def _clamp_force(self,cmd):
        out_of_bouds = False
        max_force = 220 # [N]  85mm gripper
        if(self._gripper.stroke == 0.140):
            max_force = 120 # [N] 140mm gripper
        if (cmd < 0.0):
            out_of_bouds = True
            cmd_corrected = 0.0
        elif (cmd > max_force):
            out_of_bouds = True
            cmd_corrected = max_force
        if(out_of_bouds):
            rospy.loginfo("Force (%.3f[N]) out of limits for %d[mm] gripper: \n- New force: %.3f[N]\n- Min force: %.3f[N]\n- Max force: %.3f[N]" % (cmd, self._gripper.stroke*1000, cmd_corrected, 0, max_force))
            cmd = cmd_corrected
        return cmd
    
    def update_gripper_command(self,cmd):
        if (True == cmd.emergency_release):
            self._gripper.activate_emergency_release(open_gripper=cmd.emergency_release_dir)
            return
        else:
            self._gripper.deactivate_emergency_release()

        if (True == cmd.stop):
            self._gripper.stop()
        else:
            pos = self._clamp_position(cmd.position)
            vel = self._clamp_speed(cmd.speed)
            force = self._clamp_force(cmd.force)
            # rospy.loginfo(" %.3f %.3f %.3f " % (pos,vel,force))
            self._gripper.goto(pos=pos,vel=vel,force=force)
            
    def get_current_gripper_status(self):
        status = RobotiqGripperStatus()
        status.header.stamp = rospy.get_rostime()
        status.header.seq = self._seq
        status.is_ready = self._gripper.is_ready()
        status.is_reset = self._gripper.is_reset()
        status.is_moving = self._gripper.is_moving()
        status.obj_detected = self._gripper.object_detected()
        status.fault_status = self._gripper.get_fault_status()
        status.position = self._gripper.get_pos()
        status.requested_position = self._gripper.get_req_pos()
        status.current = self._gripper.get_current()
        return status
        
    def _update_gripper_joint_state(self):
        js = JointState()
        js.header.frame_id = ''
        js.header.stamp = rospy.get_rostime()
        js.header.seq = self._seq
        js.name = [self._joint_name]
        max_joint_limit = 0.8
        if( self._gripper.stroke == 140 ):
            max_joint_limit = 0.7
        pos = np.clip(max_joint_limit - ((max_joint_limit/self._gripper.stroke) * self._gripper.get_pos()), 0., max_joint_limit)
        js.position = [pos]
        dt = rospy.get_time() - self._prev_joint_state_time
        self._prev_joint_state_time = rospy.get_time()
        js.velocity = [(pos-self._prev_joint_pos)/dt]
        self._prev_joint_pos = pos
        return js
        
    def _run_driver(self):
        last_time = rospy.get_time()
        r = rospy.Rate(rospy.get_param('~rate', 100))
        while not rospy.is_shutdown() and self._driver_state != 2:
            # Check if communication is failing or taking too long
            dt = rospy.get_time() - last_time
            if (0 == self._driver_state):
                if (dt < 0.5):
                    self._gripper.deactivate_gripper()
                else:
                    self._driver_state = 1
            # If driver is not running, activate gripper 
            elif (1 == self._driver_state):
                is_gripper_activated = True
                self._gripper.activate_gripper()
                is_gripper_activated &= self._gripper.is_ready()
                if (is_gripper_activated):
                    rospy.loginfo("Gripper on port %s Activated" % self._comport)
                    self._driver_state = 2
                        
            success = True
            success &= self._gripper.process_action_cmd()
            success &= self._gripper.process_status_cmd()
            if not success and not rospy.is_shutdown():
                rospy.logerr("Failed to contact gripper %d"% self._gripper.device_id)
            else:
                stat = RobotiqGripperStatus()
                js = JointState()
                stat = self.get_current_gripper_status()
                js = self._update_gripper_joint_state()
                if stat.is_ready:
                    self.is_ready = True 
                # self._gripper_pub.publish(stat)
                # self._gripper_joint_state_pub.publish(js)
                            
            r.sleep()


    def update_driver(self):
        success = True
        success &= self._gripper.process_action_cmd()
        success &= self._gripper.process_status_cmd()
        if not success and not rospy.is_shutdown():
            rospy.logerr("Failed to contact gripper %d"% self._gripper.device_id)
        else:          
            js = JointState()
            js = self._update_gripper_joint_state()
            self._gripper_joint_state_pub.publish(js)

    ######################################################################################################
    # STATIC functions for fast control of the gripper.

    @staticmethod
    def goto( client, pos, speed=0.1, force=120, block = True ):
        goal = CommandRobotiqGripperGoal()
        goal.emergency_release = False
        goal.stop = False
        goal.position = pos
        goal.speed = speed
        goal.force = force

        # Sends the goal to the gripper.
        if block:   
            client.send_goal(goal)
            client.wait_for_result()
        else:
            client.send_goal(goal)
    
    @staticmethod
    def close( client, speed=0.1, force=120, block = True,):
        goal = CommandRobotiqGripperGoal()
        goal.emergency_release = False
        goal.stop = False
        goal.position = 0.0
        goal.speed = speed
        goal.force = force

        # Sends the goal to the gripper.
        if block:
            client.send_goal_and_wait(goal)
        else:
            client.send_goal(goal)

    @staticmethod
    def open( client, speed=0.1, force=120, block = True):
        goal = CommandRobotiqGripperGoal()
        goal.emergency_release = False
        goal.stop = False
        goal.position = 255 # Use max value to make it stroke independent
        goal.speed = speed
        goal.force = force

        # Sends the goal to the gripper.
        if block:
            client.send_goal_and_wait(goal)
        else:
            client.send_goal(goal)

    @staticmethod
    def stop( client, block = True):
        goal = CommandRobotiqGripperGoal()
        goal.emergency_release = False
        goal.stop = False

        # Sends the goal to the gripper.
        if block:
            client.send_goal_and_wait(goal)
        else:
            client.send_goal(goal)

    @staticmethod
    def emergency_release( client ):
        goal = CommandRobotiqGripperGoal()
        goal.emergency_release = True
        client.send_goal_and_wait(goal)

class Robotiq2FingerSimulatedGripperDriver:

    def __init__(self, stroke=0.85, joint_name='finger_joint'):
        self._stroke = stroke
        self._joint_name = joint_name
        self._current_pos = self._stroke
        self._prev_time = rospy.get_time()
        self._current_goal = CommandRobotiqGripperGoal()
        self._current_goal.position = self._stroke
        self._gripper_joint_state_pub = rospy.Publisher("/joint_states" , JointState, queue_size=10)  
        self.is_ready = True
        self._is_moving = False
        self._max_joint_limit = 0.8
        if( self._gripper.stroke == 140 ):
            self._max_joint_limit = 0.7


    def update_driver(self):
        delta_time = rospy.get_time() - self._prev_time
        pos = np.clip(max_joint_limit - ((max_joint_limit/self._gripper.stroke) * self._gripper.get_pos()), 0., max_joint_limit)
        
        position_increase = delta_time * self._current_goal.speed
        if( abs(self._current_goal.position - self._current_pos) > position_increase ):
            self._current_pos += position_increase if (self._current_goal.position - self._current_pos) > 0 else -position_increase
            self._is_moving = True
        else:
            self._current_pos = self._current_goal.position
            self._is_moving = False
        js = self._update_gripper_joint_state()
        self._gripper_joint_state_pub.publish(js)
        self._prev_time = rospy.get_time()


    def update_gripper_command(self, goal_command):
        self._current_goal = goal_command
        self._current_goal.position = self._clamp_position(goal_command.position)
        self._current_goal.speed = self._clamp_speed(goal_command.speed)

        self.rPR = int(np.clip((3. - 230.)/self.stroke * pos + 230., 0, 255))
        self.rSP = int(np.clip(255./(0.1 - 0.013) * vel-0.013, 0, 255))
        self.rFR = int(np.clip(255./(self._max_force - 5.) * force - 5., 0, 255))

    def get_current_gripper_status(self):
        status = RobotiqGripperStatus()
        status.header.stamp = rospy.get_rostime()
        status.header.seq = 0
        status.is_ready = True
        status.is_reset = False
        status.is_moving = self._is_moving
        status.obj_detected = False
        status.fault_status = False
        status.position = self._current_pos
        status.requested_position = self._current_goal.position
        status.current = 0.0
        return status

    def get_joint_position_from_mm(self, linear_pose ):
        return np.clip(self._max_joint_limit - ((self._max_joint_limit/self._stroke) * linear_pose, 0.0, self._max_joint_limit)

    def _update_gripper_joint_state(self):
        js = JointState()
        js.header.frame_id = ''
        js.header.stamp = rospy.get_rostime()
        js.header.seq = 0
        js.name = [self._joint_name]
        pos = np.clip(self._max_joint_limit - ((self._max_joint_limit/self._stroke) * self._current_pos), 0.0, self._max_joint_limit)
        js.position = [pos]
        js.velocity = [self._current_goal.speed]
        self._prev_joint_pos = pos
        return js

    def _clamp_position(self,cmd):
        if (cmd <= 0.0):
            cmd = 0.0
        elif (cmd >= self._stroke):
            cmd = self._stroke
        return cmd
    
    def _clamp_speed(self,cmd):
        if (cmd < 0.0):
            cmd = 0.01
        elif (cmd > 0.101):
            cmd = 0.1
        return cmd

