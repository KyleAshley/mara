#!/usr/bin/env python
# Copyright (c) 2017, Kyle Ashley
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys, time, os

import rospy
from mara_utils import *

import moveit_msgs.msg
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from moveit_msgs.msg import MoveGroupActionResult, ExecuteTrajectoryActionResult


# combines joint states published by the two JTS nodes

l_joint_states_curr = {'left_joint1':0.0, 'left_joint2':0.0,'left_joint3':0.0,'left_joint4':0.0,'left_joint5':0.0,'left_joint6':0.0}
r_joint_states_curr = {'right_joint1':0.0, 'right_joint2':0.0,'right_joint3':0.0,'right_joint4':0.0,'right_joint5':0.0,'right_joint6':0.0}
both_joint_states_curr = {'left_joint1':0.0, 'left_joint2':0.0,'left_joint3':0.0,'left_joint4':0.0,'left_joint5':0.0,'left_joint6':0.0, \
						  'right_joint1':0.0, 'right_joint2':0.0,'right_joint3':0.0,'right_joint4':0.0,'right_joint5':0.0,'right_joint6':0.0}

def updateLeftJointStates(msg):
	global l_joint_states_curr
	for name, theta in zip(msg.name, msg.position):
		l_joint_states_curr[name] = theta

def updateRightJointStates(msg):
	global r_joint_states_curr
	for name, theta in zip(msg.name, msg.position):
		r_joint_states_curr[name] = theta


success = rospy.init_node('joint_state_combiner')

left_sub = rospy.Subscriber('/mara/limb/left/joint_states', JointState, updateLeftJointStates, queue_size=1)
right_sub = rospy.Subscriber('/mara/limb/right/joint_states', JointState, updateRightJointStates, queue_size=1)
joint_state_pub = rospy.Publisher("/mara/joint_states", JointState, queue_size=1)

rate = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
	msg = JointState()
	msg.header = Header()
	msg.header.stamp = rospy.Time.now()	

	names = []
	positions = []
	for name in r_joint_states_curr:
		names.append(name)
		positions.append(r_joint_states_curr[name])
	for name in l_joint_states_curr:
		names.append(name)
		positions.append(l_joint_states_curr[name])

	msg.name = names
	msg.position = positions
	joint_state_pub.publish(msg)


	rate.sleep()


