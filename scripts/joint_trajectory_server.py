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

import serial
import cv2
import sys, time, os
from subprocess import call
import rospy
from mara_utils import *
import datetime
import math 
import stat

import actionlib
from actionlib_msgs.msg import GoalStatusArray, GoalStatus, GoalID
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryActionFeedback,
    FollowJointTrajectoryResult,
    FollowJointTrajectoryActionResult
)
import moveit_msgs.msg
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Int32
from moveit_msgs.msg import MoveGroupActionResult, ExecuteTrajectoryActionResult

import matplotlib.pyplot as plt
import numpy as np

import threading


# NOTES
# - On startup:
# 	- gripper auto closes, need to command positive velocity to open before sending more commands 
# (Done automatically now):
# - $ sudo chmod +777 /dev/ttyUSB0
# - $ sudo modprobe usbserial vendor=0x067b product=0x2303 
#
# To kill all mara processes:
# 	$ sudo kill -9 `ps -ef | grep mara | awk '{ print $2 }'`


# USAGE:
# $ rosrun mara joint_trajectory_server.py <left/right limb (l/r)> (device ID (0/1)) 

class JointTrajectoryServer():
	def __init__(self, device_num=None, left_right='l'):

		self.is_initialized = False 		# gets set after constructer successfully finishes
		self.encoder_decode_error = False	# gets set when theres an issue decoding serial responses... not sure how to fix 
		self.interface = None 				# python serial interface

		# get the device names from /dev folder
		device_list = self.getUSBDeviceNames() 
		if len(device_list) != 0:
			if device_num == None:
				self.device_name = device_list[0] 					# linux device name for serial interface
			else:
				self.device_name = device_list[device_num] 					# linux device name for serial interface
		else:
			print "Error initializing serial interface... (make sure the arms are plugged in!)"
			return

		print "------------------------------------------------------------------------------------------"
		print "sudo priveleges are needed to load the kernel module and change USB write permissions..."
		print "------------------------------------------------------------------------------------------"
		call(["sudo", "chmod", "+777", self.device_name]) 	# give permissions to access device file
		self.initializeSerialInterface(self.device_name)
		self.resetSerialInterface()

		# TODO: need a better method here for determining r/l arms
		if 'r' in left_right:
			self.arm_name = "right"
		else:
			self.arm_name = "left"

		print "Initializing", self.arm_name, "arm..."

		self.rate = 0.25 					# pulse width period for sending joint commands through USB  (Dont change it) (0.3 is safe, 0.2 is extreme)
		self.max_trajectory_velocity = 0.3  # maximum allowed joint velocity (rad/sec)
		self.pause_control_loop = False		# flag to be set when you want to use serial commands outside of the control loop
		self.planning_failed = False 		# subscribes to the status topic sent from the motion planner 
											# (THIS FIXES A KNOWN BUG: path plans that result in arm collision are executed (because planning == execution))

		# serial communication results
		self.joint_array = None 	# most recent hex array from encoders
		self.joint_angles = None 	# most recent decoded joint angles
		self.trajectory_queue = [] 	# trajectory queue contains incoming arm trajectory goals
		self.trajectory_queue_len = 1  # holds multiple trajectories if we want to execute queued path plans

		# joint names from ee -> base
		self.ordered_joints = ['g', 'j6', 'j5', 'j4', 'j3', 'j2', 'j1']
		self.full_joint_names = [self.arm_name +'_joint1', self.arm_name +'_joint2', self.arm_name +'_joint3', \
								 self.arm_name +'_joint4', self.arm_name +'_joint5', self.arm_name +'_joint6']

		self.joint_states_curr = {self.arm_name+'_joint1':0.0, self.arm_name+'_joint2':0.0,self.arm_name+'_joint3':0.0, \
								  self.arm_name+'_joint4':0.0,self.arm_name+'_joint5':0.0,self.arm_name+'_joint6':0.0}

		# project directory
		self.project_path = "/home/carrt/Dropbox/catkin_ws/src/mara/"

		# ros subs and pubs
		self.rviz_trajectory_sub = rospy.Subscriber('/move_group/result', MoveGroupActionResult, self.updateMoveitTrajectory, queue_size=4)
		self.rviz_status_sub = rospy.Subscriber('/move_group/status', GoalStatusArray, self.updateMoveitStatus, queue_size=1)
		self.gripper_sub = rospy.Subscriber('/mara/limb/' + self.arm_name +'/gripper_action/position', Int32, self.updateGripperPosition, queue_size=1)

		self.joint_state_pub = rospy.Publisher("/mara/limb/" +self.arm_name +"/joint_states", JointState, queue_size=1)
		self.gripper_result_pub = rospy.Publisher("/mara/limb/" +self.arm_name +"/gripper_action/result", GoalStatus, queue_size=1)
		self.arm_result_pub = rospy.Publisher("/mara/limb/" +self.arm_name +"/follow_joint_trajectory/result", FollowJointTrajectoryActionResult, queue_size=1)
		
		self.server = actionlib.SimpleActionServer(
            '/mara/limb/' + self.arm_name + '/follow_joint_trajectory',
            FollowJointTrajectoryAction,
            execute_cb=self.updateMoveitTrajectory,
			auto_start=False)
		self.trajectory_status = -1
		self.server.start()
		self.fdbk = FollowJointTrajectoryActionFeedback() 		# feedback msg
		self.result = FollowJointTrajectoryActionResult()		# result message
		

	#--------------------------------------------------------------------------------------------------------------------------------------------------#
	# Callbacks
	#--------------------------------------------------------------------------------------------------------------------------------------------------#
	# commands gripper to published position
	def updateGripperPosition(self, msg):
		print "Commanding", self.arm_name, 'gripper to', str(msg.data)
		self.pause_control_loop = True 	# pause the control loop to free the serial bus
		rospy.sleep(1.0)

		angles = {self.arm_name+'_joint1':0.0, self.arm_name+'_joint2':0.0,self.arm_name+'_joint3':0.0, \
				  self.arm_name+'_joint4':0.0,self.arm_name+'_joint5':0.0,self.arm_name+'_joint6':0.0,self.arm_name+'_gripper':0.0}
		vels = {self.arm_name+'_joint1':0.0, self.arm_name+'_joint2':0.0,self.arm_name+'_joint3':0.0, \
				self.arm_name+'_joint4':0.0,self.arm_name+'_joint5':0.0,self.arm_name+'_joint6':0.0,self.arm_name+'_gripper':0.0}

		angles[self.arm_name+"_gripper"] = int(msg.data)
		vels[self.arm_name+"_gripper"] = 20.0

		msg = GoalStatus()
		msg.status = msg.ACTIVE
		self.gripper_result_pub.publish(msg)
		self.commandAllJointAngles(angles, vels)

		msg.status = msg.SUCCEEDED
		self.gripper_result_pub.publish(msg)

		rospy.sleep(0.1)
		msg.status = msg.PENDING
		self.gripper_result_pub.publish(msg)

	# sends joint state feedback on the simple action server
	def updateFeedback(self, desired_point, angles, error_dict, curr_time):
		print self.fdbk
		self.fdbk.header.stamp = rospy.Duration.from_sec(rospy.get_time())
		self.fdbk.joint_names = self.full_joint_names
		self.fdbk.desired = desired_point
		self.fdbk.desired.time_from_start = rospy.Duration.from_sec(curr_time)
		self.fdbk.actual.positions = angles
		self.fdbk.actual.time_from_start = rospy.Duration.from_sec(curr_time)
		self.fdbk.error.positions = error_dict
		self.fdbk.error.time_from_start = rospy.Duration.from_sec(curr_time)
		self.server.publish_feedback(self.fdbk)

	# recognizes ABORTED motion plans
	def updateMoveitStatus(self, msg):
		if len(msg.status_list) > 0:
			if msg.status_list[0].status == msg.status_list[0].ABORTED:
				print "Error, planning failed!"
				self.planning_failed = True
			else:
				self.planning_failed = False


	# receives trajectories from the moveit topic and adds them to the execution queue
	def updateMoveitTrajectory(self, msg):
		self.pause_control_loop = True 	# pause the control loop to free the serial bus (otherwise its constantly queried and we could get bad data)
		rospy.sleep(0.1)

		# first get the most recent joint states
		angles = self.getJointAngles() 
		self.updateJointPositions(angles)

		self.pause_control_loop = False
		rospy.sleep(0.1)

		# determine what kind of message we received
		self.trajectory_status = -1
		traj = None
		if type(msg) is MoveGroupActionResult:
			traj = msg.result.planned_trajectory.joint_trajectory
		else:
			traj = msg.trajectory

		# execute trajectory if its for the correct arm
		if self.arm_name in traj.joint_names[0]:
			print 'Received new RVIZ joint trajectory for', self.arm_name 
			trajectory = traj
			short_names = {'joint1':'j1', 'joint2':'j2', 'joint3':'j3', 'joint4':'j4', 'joint5':'j5', 'joint6':'j6', 'gripper':'g'}
			new_names = []
			for n in trajectory.joint_names:
				name = n.strip('left_')
				name = name.strip('right_')
				new_names.append(short_names[name])
			trajectory.joint_names = new_names

			# slow down the trajectory and add it to the queue
			trajectory = self.enforceMaxTrajectoryVelocity(trajectory)
			self.trajectory_queue.append(trajectory)

			# wait for execution to complete
			while self.trajectory_status != self.result.result.SUCCESSFUL:
				rospy.sleep(0.05)

			print "SUCCESS, alerting the moveit client"
			# publish the result
			self.result.header = Header()
			self.result.header.stamp = rospy.Time.now()
			self.result.result.error_code = self.result.result.SUCCESSFUL
			self.arm_result_pub.publish(self.result)
		
			#self.trajectory_queue = self.trajectory_queue[-self.trajectory_queue_len:]


	# publish the robot joint states
	def updateJointPositions(self, angles):
		msg = JointState()
		msg.header = Header()
		msg.header.stamp = rospy.Time.now()	

		if self.arm_name == "left":
			full_names = {'j1':'left_joint1', 'j2':'left_joint2', 'j3':'left_joint3', 'j4':'left_joint4', 'j5':'left_joint5', 'j6':'left_joint6'}
		elif self.arm_name == "right":
			full_names = {'j1':'right_joint1', 'j2':'right_joint2', 'j3':'right_joint3', 'j4':'right_joint4', 'j5':'right_joint5', 'j6':'right_joint6'}
		else:
			full_names = {'j1':'joint1', 'j2':'joint2', 'j3':'joint3', 'j4':'joint4', 'j5':'joint5', 'j6':'joint6'}

		names = []
		positions = []
		for name, theta in angles.iteritems():
			if name == 'g':
				pass
			else:
				self.joint_states_curr[full_names[name]] = (theta - 180.0) * (math.pi/180.0)

		for name, theta in self.joint_states_curr.iteritems():
			names.append(name)
			positions.append(theta)

		msg.name = names
		msg.position = positions
		self.joint_state_pub.publish(msg)

	
	#--------------------------------------------------------------------------------------------------------------------------------------------------#
	# Utilities
	#--------------------------------------------------------------------------------------------------------------------------------------------------#
	def initializeSerialInterface(self, device_name):
		# open the serial interface to the arm
		try:
			self.interface = serial.Serial(device_name, 9600,xonxoff=True, timeout=0.1)
			print self.interface.name
			self.is_initialized = True
		except:
			print("Could not open usb interface for left arm on: \t ---> \t" + str(device_name))
			print "Attempting to load the kernel module..."
			os.system("sudo modprobe usbserial vendor=0x067b product=0x2303")

			try:
				print ("Opening mara control on: \t \t ---> \t" + str(self.interface.name))
				self.interface = serial.Serial(self.device, 9600, xonxoff=True, timeout=0.1)
				self.is_initialized = True

			except Exception, e:
				print "Failed...exiting"
				self.is_initialized = False

	def resetSerialInterface(self):
		os.system("sudo modprobe usbserial vendor=0x067b product=0x2303")
		self.interface = serial.Serial(self.device_name, 9600,xonxoff=True, timeout=0.1)
		self.interface.write('\r')
		val = self.interface.read(1)
		while val:
			val = self.interface.read(1)
		self.is_initialized = True


	# looks for USB devices on /dev, returns a list of stings containing the device names
	def getUSBDeviceNames(self):
		devices = []
		for i in range(10):
			path = "/dev/ttyUSB" + str(i)
			try:
				if os.path.exists(path):
					devices.append(path)
			except:
				pass
		return devices


	# decodes hex response from serial query for joint values
	def decodeJointValues(self, joint_array, verbose=False):
		# degrees per encoder step
		joint_angles = {'j1':None, 'j2':None, 'j3':None, 'j4':None, 'j5':None, 'j6':None, 'g':None}

		try:
			for i in range(7):
				val = int(joint_array[i*4:(i*4)+4], 16)
				# put values between 0-360
				# joints get +-1800 from 0x0000
				# gripper: open = 0xbb80, closed = 0x0000
				if i != 6:
					encoder_ratio = 0.10
					key = 'j'+str(i+1)
					if val > 1800:
						val = 180 + (180 - ((65535 - val)* float(encoder_ratio)))
						joint_angles[key] =  val
					else:
						joint_angles[key] =  val * float(encoder_ratio)
				else:
					encoder_ratio = 0.001702
					key = 'g'
					if val > 0 and val < 54801:
						val = val + 7500
						joint_angles[key] =  val * float(encoder_ratio)
					else:
						val = val - 54801
						joint_angles[key] =  val * float(encoder_ratio)
			self.encoder_decode_error = False

		except:
			print "Failed to decode joint values..."
			self.encoder_decode_error = True

		if verbose:
			rospy.loginfo("Joint angles received " + str(joint_angles))

		return joint_angles

	# queries the arm for joint angles and returns dictionary of 'joint':angle pairs for each arm
	def getJointAngles(self):
		if self.interface:
			# send command to get left arm joint angles
			self.interface.write('p\r')
			self.joint_array = self.interface.read(29)

			if len(self.joint_array) != 0 and ">Stop" not in str(self.joint_array):
	
				# convert hex response to joint angles
				self.joint_angles = self.decodeJointValues(self.joint_array)

				# this modification to decoded values is neccesary for some reason
				if not self.encoder_decode_error:
					self.joint_angles['j3'] = self.joint_angles['j3'] - self.joint_angles['j2'] 		# offset linked encoders
					if self.joint_angles['j3'] < 0:
						self.joint_angles['j3'] += 360
				else:
					print "Error: Bad joint values. Re-initializing serial interface"
					self.resetSerialInterface()
			else:
				if ">Stop" in str(self.joint_array):
					print "Warning: joints near singularity..."
				else:
					print "Error: failed to receive joint state from left arm. Re-initializing serial interface"
					self.resetSerialInterface()

		else:
			print "Serial interface is not working...re-initializing"
			self.resetSerialInterface()

		return self.joint_angles

	# command a single joint at 'vel' degrees per sec using the 'jv' command
	def commandJointVelocity(self, joint, vel):
		command = None
		sign = '#'

		if 'g' not in joint:
			joint = ''.join([i for i in joint if i.isdigit()])

		if self.interface:
			command = 'jv ' + str(joint) + sign + str(vel) + '\r'
			self.interface.write('jv ' + str(joint) + sign + str(vel) + '\r')


	# command a multplie joints at 'vel' degrees per sec using the 'm' command
	def commandAllJointVelocities(self, vels, left_right='l'):
		command = None
		sign = '#'

		full_names = {'j1':self.arm_name+'_joint1', 'j2':self.arm_name+'_joint2', 'j3':self.arm_name+'_joint3', \
					  'j4':self.arm_name+'_joint4', 'j5':self.arm_name+'_joint5', 'j6':self.arm_name+'_joint6', 'g':self.arm_name+'_gripper'}

		cmd_str = ['0','0','0','0','0','0','0']

		for j,v in vels.iteritems():
			if j == 'j1' or j == full_names['j1']:
				idx = 0
			elif j == 'j2' or j == full_names['j2']:
				idx = 1
			elif j == 'j3' or j == full_names['j3']:
				idx = 2
			elif j == 'j4' or j == full_names['j4']:
				idx = 3
			elif j == 'j5' or j == full_names['j5']:
				idx = 4
			elif j == 'j6' or j == full_names['j6']:
				idx = 5
			elif j == 'g' or j == full_names['g']:
				idx = 6

			# round up to the nearest velocity command (A=10, B=20 ...)
			if v < 0:
				if abs(v) <= 10:
					cmd_str[idx] = 'A'
				elif abs(v) <= 20:
					cmd_str[idx] = 'B'
				else:
					cmd_str[idx] = 'C'
			elif v > 0:
				if abs(v) <= 10:
					cmd_str[idx] = 'a'
				elif abs(v) <= 20:
					cmd_str[idx] = 'b'
				else:
					cmd_str[idx] = 'c'
			else:
				cmd_str[idx] = '0'

		cmd_str = ''.join(cmd_str)
		if self.interface:
			self.interface.write('m' + cmd_str + '\r')


	# send carriage return to stop arm movement
	def stopMovement(self):
		if self.interface:
			self.interface.write('\r')
			print self.interface.read(5)
			print self.interface.read(29)


	# given the current angle of a joint and desired angle, returns the signed velocity required for
	# shortest movement
	def signVelocity(self, current_angle, desired_angle, vel, isGripper=False):
		# check which direction we should rotate the joint
		# set the sign
		if not isGripper:
			# shift current angle to zero and desired angle relative to current angle
			shifted_current = 0
			shifted_desired = desired_angle - current_angle

			if shifted_desired < 0:
				shifted_desired = shifted_desired + 360

			if shifted_desired > 180:
				new_vel = abs(vel) * -1
			else:
				new_vel = abs(vel) 
		else:
			if desired_angle < current_angle:
				new_vel = abs(vel) * -1
			else:
				new_vel = abs(vel)

		return new_vel

	# OLD (Dont use)
	# static velocity control (only using 10, 20, or 30 deg per sec builtin commands) using 'jv' command
	def commandJointAngle_STATIC(self, joint_name, angle, vel, variable_velocity=False):

		angles = self.getJointAngles()
		rate = 0.01
		# set encoder error tolerance for gripper and arm joints (larger error for gripper?)
		if 'g' in joint_name:
			tolerance = 1.0
			isGripper = True
		else:
			tolerance = 0.3
			isGripper = False

		# command the left arm
		if self.interface and angles:

			vel = self.signVelocity(angles[joint_name], angle, vel, isGripper)
			self.commandJointVelocity(joint_name, vel)
			angles = self.getJointAngles()

			lower = angle - tolerance
			upper = angle + tolerance

			# set stop condition for joint movement with wrap-around at 360 degrees
			print "Moving joint", joint_name, 'to angle (', lower, '-', upper, ') at rate', vel
			if lower < 0:
				stop_condition = inRange(angles[joint_name], 360 + lower, 360) or inRange(angles[joint_name], 0, upper)
			else:
				stop_condition = inRange(angles[joint_name], lower, upper)

			# iteratively command joint at velocity and check position
			while not stop_condition:
				vel = self.signVelocity(angles[joint_name], angle, vel, isGripper)
				self.commandJointVelocity(joint_name, vel)
				angles = self.getJointAngles()

				if lower < 0:
					stop_condition = inRange(angles[joint_name], 360 + lower, 360) or inRange(angles[joint_name], 0, upper)
				else:
					stop_condition = inRange(angles[joint_name], lower, upper)

				time.sleep(rate)

			#self.stopMovement()
			self.commandJointVelocity(joint_name, 0)

	# command joints to a specific postion at constant velocity (using 'm' command)
	def commandAllJointAngles(self, des_angles, vels):

		curr_angles = self.getJointAngles()
		self.updateJointPositions(curr_angles)
		rate = 0.05
		
		full_names = {'j1':self.arm_name+'_joint1', 'j2':self.arm_name+'_joint2', 'j3':self.arm_name+'_joint3', \
					  'j4':self.arm_name+'_joint4', 'j5':self.arm_name+'_joint5', 'j6':self.arm_name+'_joint6', 'g':self.arm_name+'_gripper'}

		# re-sign the velocities 
		for joint_name in curr_angles.keys():
			if self.interface and curr_angles:	
				if joint_name == 'g':
					isGripper = True		
				else:
					isGripper = False
				vel = self.signVelocity(curr_angles[joint_name], des_angles[full_names[joint_name]], vels[full_names[joint_name]], isGripper)
				vels[full_names[joint_name]] = vel
		
		stop_conditions = curr_angles.copy()
		for condi in stop_conditions.keys():
			stop_conditions[condi] = False

		for joint_name in curr_angles.keys():
			tolerance = 1.0
			lower = des_angles[full_names[joint_name]] - tolerance
			upper = des_angles[full_names[joint_name]] + tolerance
			# set stop condition for joint movement with wrap-around at 360 degrees
			if lower < 0:
				stop_conditions[joint_name] = inRange(curr_angles[joint_name], 360 + lower, 360) or inRange(curr_angles[joint_name], 0, upper)
			else:
				stop_conditions[joint_name] = inRange(curr_angles[joint_name], lower, upper)

		# hold until all the high duty cycles are done
		print all(value == 0 for value in vels.values())
		while not all(value == 0 for value in vels.values()):
			curr_angles = self.getJointAngles()
			self.updateJointPositions(curr_angles)
			self.commandAllJointVelocities(vels)

			print curr_angles
			for joint_name in curr_angles.keys():
				tolerance = 1.0
				lower = des_angles[full_names[joint_name]] - tolerance
				upper = des_angles[full_names[joint_name]] + tolerance
				# set stop condition for joint movement with wrap-around at 360 degrees
				if lower < 0:
					stop_conditions[joint_name] = inRange(curr_angles[joint_name], 360 + lower, 360) or inRange(curr_angles[joint_name], 0, upper)
				else:
					stop_conditions[joint_name] = inRange(curr_angles[joint_name], lower, upper)

				isChangedVel = False
				for joint_name,theta in curr_angles.iteritems():
					if stop_conditions[joint_name] and vels[full_names[joint_name]] != 0:
						isChangedVel = True
						vels[full_names[joint_name]] = 0

				if isChangedVel:
					print "stopping a joint", vels
					self.commandAllJointVelocities(vels)
			rospy.sleep(rate)

		print "Done!"
		self.pause_control_loop = False



	# getst the gripper in a good starting position after powering the arm
	def initializeGripper(self):
		self.commandJointVelocity(joint='g', vel=10)
		# open the gripper a bit
		angles = self.getJointAngles()

		self.commandJointAngle_STATIC('g', angle=30, vel=20)
		self.commandJointAngle_STATIC('g', angle=20, vel=20)

		print "Done!"

	# scale velocity down to < 30 deg/s
	def enforceMaxTrajectoryVelocity(self, trajectory):
		
		limitExceeded = True

		while(limitExceeded):
			limitExceeded = False
			# iterate over points and joint names
			for p1 in trajectory.points:
				for n in range(len(trajectory.joint_names)):
					# if the velocity threshold is exceeded, double the timestamps
					if p1.velocities[n] > self.max_trajectory_velocity or p1.velocities[n] < -self.max_trajectory_velocity:
						for p2 in trajectory.points:
							p2.time_from_start *= 2
							new_vels = []
							for i in range(len(p2.velocities)):
								new_vels.append(p2.velocities[i] / 2)
							p2.velocities = new_vels

						limitExceeded = True
					if limitExceeded:
						break
				if limitExceeded:
					break
			
		for p1 in trajectory.points:
			new_positions = []
			for n in range(len(trajectory.joint_names)):
				angle = p1.positions[n] * (180.0/math.pi)
				angle = angle + 180 			# weird offset required for some reason
				if angle < 0:
					angle = angle + 360
				elif angle > 360:
					angle = angle % 360
				new_positions.append(angle)
			p1.positions = new_positions

		new_trajectory = trajectory
		return new_trajectory






	#--------------------------------------------------------------------------------------------------------------------------------------------------#
	# Main Control Loop
	#--------------------------------------------------------------------------------------------------------------------------------------------------#
	# - obtains joint positions and publishes to:	 /mara/<arm>/joint_positions/<joint_name>
	# - accepts trajectories from:					 /move_group/goal  	(TODO: needs to change?)
	# - executes joint trajectories using PD control loop
	def startJointTrajectoryControlLoop(self):
		
		if self.interface:

			period = self.rate  			# should be 0.2 
			print "Entering control loop"

			while not rospy.is_shutdown():

				if not self.pause_control_loop:
					velocity_windows = {'j1':[], 'j2':[], 'j3':[], 'j4':[], 'j5':[], 'j6':[], 'g':[]}		# records instantaneous velocities of joints

					# while no trajectories are being executed, simply update the joint state
					angles = self.getJointAngles() 
					if not angles:
						print "Exiting control loop"
						break
					self.updateJointPositions(angles)

					# when there is a new trajectory, execute it
					# trajectory is a series of [pos, timestamp] for each joint
					if self.trajectory_queue != None and len(self.trajectory_queue) > 0:

						print "Executing joint trajectory...", self.arm_name
						curr_trajectory = self.trajectory_queue[0]

						t_start = time.time()
						t_start_ros = rospy.get_time()
						angles_prev = angles
						waypoint_prev = angles

						positions_err_dict = {'j1':[], 'j2':[], 'j3':[], 'j4':[], 'j5':[], 'j6':[], 'g':[]}
						velocities_err_dict = {'j1':[], 'j2':[], 'j3':[], 'j4':[], 'j5':[], 'j6':[], 'g':[]}


						# iterate through waypoints in the trajectory
						for waypoint, waypoint_num in zip(curr_trajectory.points, range(len(curr_trajectory.points))):

							# check if its the last waypoint (do extra correction if so)
							last_waypoint = False
							if waypoint_num == len(curr_trajectory.points)-1:
								last_waypoint = True

							t_subdiv = time.time() 			# subdivision of waypoint period at interval == PWM period
							t1 = time.time() 				# starting time of PWM period
							t3 = time.time()
							t_waypoint = time.time() 		# time waypoint was first seen
							t_interval = period 			# initial velocity calcualtion based on ideal interval
							initial_command = True 			# dont the period for initial command
							error_warning = False			# warns when joints are outside final error
							final_error_tolerance = 0.8

							waypoint_angles = angles 		# joint values when waypoint was started
							#print "WAYPOINT--------", waypoint_num,"of", len(curr_trajectory.points),"---------------------------------------------------"
							#print "check interval = ", t_interval
							#print 'check time = ', t_subdiv - t_start
							#print 'current waypoint duration', waypoint.time_from_start.to_sec()

							error_correct_attempts = 0 		# number of attempted corrections on final waypoint
							# if the waypoint goal time has not elapsed, or we reached the last waypoint with significant error
							while t_subdiv < t_start + waypoint.time_from_start.to_sec() or (last_waypoint and error_warning and error_correct_attempts < 20):

								# attempt to re-query the joint state
								while None in angles.viewvalues():
									print "Retrying to query joint states..."
									angles = self.getJointAngles() # get new joint values
									rospy.sleep(self.rate)

								# only try to correct fine eend effector errors a few times
								if (last_waypoint and error_warning):
									error_correct_attempts += 1

								#print "waypoint check...\t \t", (t_start + waypoint.time_from_start.to_sec()) - t_subdiv
								measured_vels = {'j1':0, 'j2':0, 'j3':0, 'j4':0, 'j5':0, 'j6':0, 'g':0}
								zero_vels = {'j1':0, 'j2':0, 'j3':0, 'j4':0, 'j5':0, 'j6':0, 'g':0} 	# zero velocity commands to joints
								cmd_vels = {'j1':0, 'j2':0, 'j3':0, 'j4':0, 'j5':0, 'j6':0, 'g':0} 		# high velocity commands to joints
								duty_ratios = {'j1':0.0, 'j2':0.0, 'j3':0.0, 'j4':0.0, 'j5':0.0, 'j6':0.0, 'g':0.0}

								# calculate goal pos and vel
								goal_angles, goal_vels = {}, {}
								for name, des in zip(curr_trajectory.joint_names, waypoint.positions):
									# calculate desired joint velocity accounting for wraparound
									if (des - angles[name]) < -180:
										goal_vels[name] = (des - angles[name] + 360) / ((t_start + waypoint.time_from_start.to_sec()) - t_waypoint)
									elif (des - angles[name]) > 180:
										goal_vels[name] = (des - angles[name] - 360) / ((t_start + waypoint.time_from_start.to_sec()) - t_waypoint)
									else:
										goal_vels[name] = (des - angles[name]) / ((t_start + waypoint.time_from_start.to_sec()) - t_waypoint)
									goal_angles[name] = des

								#print "Last waypoint:", last_waypoint, "Error warn:", error_warning
								#print "Current angles", angles
								#print "Previous angles", angles_prev
								#print "Goal angles", goal_angles
								#print "Goal vels", goal_vels
								#print "Starting pos", waypoint_angles
								#print "time", ((t_start + waypoint.time_from_start.to_sec()) - t_waypoint)

								error_warning = False 			# set true if large joint errors exist at last waypoint

								# for each joint calculate PD velocity commands
								for joint_name in curr_trajectory.joint_names:
									# check if were moving clockwise or counterclockwise
									vel = goal_vels[joint_name]
									
									# record a window of joint velocities
									window_size = 1

									# calculate velocity accounting for wraparound in the positive and negative direction
									if angles[joint_name] < 90 and angles_prev[joint_name] > 270:
										delta_theta = angles[joint_name] - angles_prev[joint_name] + 360
									elif angles[joint_name] > 270 and angles_prev[joint_name] < 90:
										delta_theta = angles[joint_name] - angles_prev[joint_name] - 360
									else:
										delta_theta = angles[joint_name] - angles_prev[joint_name]

									# calculate goal error for the last waypoint
									if last_waypoint:
										if goal_angles[joint_name] < 90 and angles[joint_name] > 270:
											err_theta = goal_angles[joint_name] - angles[joint_name] + 360
										elif goal_angles[joint_name] > 270 and angles[joint_name] < 90:
											err_theta = goal_angles[joint_name] - angles[joint_name] - 360
										else:
											err_theta = goal_angles[joint_name] - angles[joint_name]
									
									else:
										expected_angle = waypoint_angles[joint_name] + ((t_subdiv - t_waypoint) * goal_vels[joint_name])
										#print "expected", expected_angle, "current", angles[joint_name]
										if expected_angle < 90 and angles[joint_name] > 270:
											err_theta = expected_angle - angles[joint_name] + 360
										elif expected_angle > 270 and angles[joint_name] < 90:
											err_theta = expected_angle - angles[joint_name] - 360
										else:
											err_theta = expected_angle - angles[joint_name]

									# set a flag to show large error at last waypoint
									if abs(err_theta) > final_error_tolerance:
										error_warning = True

									# record the instantaneous velocity ina  sliding window
									velocity_windows[joint_name].append(delta_theta/t_interval)
									if len(velocity_windows) > window_size:
										velocity_windows[joint_name] = velocity_windows[joint_name][-window_size:]
									
									#measured_vels[joint_name] = (angles[joint_name] - angles_prev[joint_name])/period 					# instantaneous measured vel
									measured_vels[joint_name] = sum(velocity_windows[joint_name])/len(velocity_windows[joint_name]) 	# sliding window measured vel
									err_vel = goal_vels[joint_name] - measured_vels[joint_name]

									# add the errors to their respective dictionaries
									positions_err_dict[joint_name].append(err_theta)
									velocities_err_dict[joint_name].append(err_vel)

									# PD calculation (these values can probably be tuned a little better)
									kp = 0.01
									kd = 0.1
									pid_vel = vel * min((kp * abs(err_theta)), 1.0) + (kd * err_vel)

									# set duty cycles
									if abs(pid_vel) <= 10:
										duty_ratio = (abs(pid_vel)) / 10.0
									elif abs(pid_vel) <= 20:
										duty_ratio = (abs(pid_vel)) / 20.0
									else:
										duty_ratio = (abs(pid_vel)) / 30.0

									# set joint commands
									cmd_vels[joint_name] = pid_vel
									duty_ratios[joint_name] = min(duty_ratio, 1.0)
									#print "(actual, goal, duty):", joint_name, measured_vels[joint_name], goal_vels[joint_name], duty_ratios[joint_name]

								# wait the period out if this is not the first command for a waypoint
								if not initial_command:
									t6 = time.time()
									t_wait = period - (t6 - t1)
									rospy.sleep(max(t_wait, 0.0))
									t7 = time.time()
								else:
									initial_command = False

								#print "actual period:", t7 - t1
								# -------------------------------------------------------------------------------------------------------------
								# PWM 
								# Send the 'm' command in the style of a PWM signal to acheive the desired velocities across all waypoints
								# -------------------------------------------------------------------------------------------------------------
								t1 = time.time()
								self.commandAllJointVelocities(cmd_vels)
								t2 = time.time()

								# place the sensor polling in duty cylce
								angles_prev = angles 			# save old joint values
								angles =  self.getJointAngles() # get new joint values
								self.updateJointPositions(angles)

								t3_prev = t3
								t3 = time.time()
								t_interval = t3 - t3_prev		# time between joint querys (for velocity calc)

								# hold until all the high duty cycles are done
								while any(cmd_vels.values()):
									t4 = time.time()
									curr_duty = (t4 - t2) / period

									# pull down PWM after duty cycle elapses for each joint
									isChangedVel = False
									for j,r in duty_ratios.iteritems():
										if r <= curr_duty and cmd_vels[j] != 0:
											isChangedVel = True
											cmd_vels[j] = 0

									if isChangedVel:
										# low velocity command
										self.commandAllJointVelocities(cmd_vels)
										#print 'duty', curr_duty, t4 - t2

								t5 = time.time()
								t_subdiv = time.time()
								# END PWM ---------------------------------------------------------------------------------------------------

							t_end_waypoint = time.time()
							waypoint_prev = waypoint

						# TODO make this an actual queue 
						self.trajectory_queue = []

						avg_pos_errors = {'j1':0, 'j2':0, 'j3':0, 'j4':0, 'j5':0, 'j6':0, 'g':0}
						avg_vel_errors = {'j1':0, 'j2':0, 'j3':0, 'j4':0, 'j5':0, 'j6':0, 'g':0}
						for j in self.ordered_joints:
							for p_e, v_e in zip(positions_err_dict[j], velocities_err_dict[j]):
								avg_pos_errors[j] = avg_pos_errors[j] + abs(p_e)
								avg_vel_errors[j] = avg_vel_errors[j] + abs(v_e)
							if (len(positions_err_dict[j]) != 0):
								avg_pos_errors[j] = avg_pos_errors[j] / len(positions_err_dict[j])
							if (len(velocities_err_dict[j]) != 0):
								avg_vel_errors[j] = avg_vel_errors[j] / len(velocities_err_dict[j])

						#print "POSITION ERROR", avg_pos_errors, sum(avg_pos_errors.values())/len(avg_pos_errors)
						#print "VELOCITIES ERROR", avg_vel_errors, sum(avg_vel_errors.values())/len(avg_vel_errors)

						self.trajectory_status = self.result.result.SUCCESSFUL
						# plot errors
						#plt.plot(positions_err_dict['j1'], 'b')
						#plt.plot(positions_err_dict['j2'], 'r')
						#plt.plot(positions_err_dict['j3'], 'm')
						#plt.plot(positions_err_dict['j4'], 'g')
						#plt.plot(positions_err_dict['j5'], 'k')
						#plt.plot(positions_err_dict['j6'], 'y')
						#plt.show()

					# no trajectory in queue
					else:
						rospy.sleep(self.rate)
						
			# no serial interface initialized
			else:
				pass







lr = str(sys.argv[1])
num = int(sys.argv[2])

print "Starting trajectory server for", lr, 'with device number', num

success = rospy.init_node('joint_trajectory_controller_'+lr)
jts = JointTrajectoryServer(device_num=num, left_right=lr)

# initialize the grippers and have user watch to find the device num
if jts.is_initialized:
	jts.initializeGripper()

jts.startJointTrajectoryControlLoop()

rospy.spin()

jts.interface.close()





