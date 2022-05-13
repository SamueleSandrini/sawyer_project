#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
Command-line interface for sending simple commands to a ROS node controlling a 2F gripper.

This serves as an example for publishing messages on the 'Robotiq2FGripperRobotOutput' topic using the 'Robotiq2FGripper_robot_output' msg type for sending commands to a 2F gripper.
"""

from __future__ import print_function

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep

class Gripper:
    def __init__(self):
        self.pub_ = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=10)
    
    def genCommandInside(self,char):
        """Update the command according to the character entered by the user."""

        if char == 'a':
            self.command = outputMsg.Robotiq2FGripper_robot_output();
            self.command.rACT = 1
            self.command.rGTO = 1
            self.command.rSP  = 255
            self.command.rFR  = 150

        if char == 'r':
            self.command = outputMsg.Robotiq2FGripper_robot_output();
            self.command.rACT = 0

        if char == 'c':
            self.command.rPR = 255

        if char == 'o':
            self.command.rPR = 0

        #If the command entered is a int, assign this value to rPRA
        try:
            self.command.rPR = int(char)
            if self.command.rPR > 255:
                self.command.rPR = 255
            if self.command.rPR < 0:
                self.command.rPR = 0
        except ValueError:
            pass

        if char == 'f':
            self.command.rSP += 25
            if self.command.rSP > 255:
                self.command.rSP = 255

        if char == 'l':
            self.command.rSP -= 25
            if self.command.rSP < 0:
                self.command.rSP = 0


        if char == 'i':
            self.command.rFR += 25
            if self.command.rFR > 255:
                self.command.rFR = 255

        if char == 'd':
            self.command.rFR -= 25
            if self.command.rFR < 0:
                self.command.rFR = 0

        return self.command
       

    def genCommand(self,char):
        # self.command = outputMsg.Robotiq2FGripper_robot_output();
        self.command = self.genCommandInside(char)
        self.pub_.publish(self.command)
        
