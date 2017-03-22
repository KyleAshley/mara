#!/usr/bin/env python
import serial
import cv2
import sys, time, os
import rospy
from mara_utils import *
import datetime

import moveit_msgs.msg
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# - implement a feedback loop for desired robot states using standard ros API

# NOTES
# - On startup:
# 	- gripper auto closes, need to command positive velocity to open before sending more commands 
# Before running:
# - $ sudo chmod +777 /dev/ttyUSB0
# - $ sudo modprobe usbserial vendor=0x067b product=0x2303 

class mara_serial():
	def __init__(self, left_right='lr'):
		self.device = "/dev/ttyUSB0" 	# linux device name for serial interface
		self.interface = None 			# python serial interface

		self.joint_angles = None
		# joint names from ee -> base
		self.orderedJoints = ['g', 'j6', 'j5', 'j4', 'j3', 'j2', 'j1']

		# project directory
		self.project_path = "/home/carrt/Dropbox/catkin_ws/src/mara/"

		
		# open the serial interface to the arm
		try:
			self.interface = serial.Serial(self.device, 9600, timeout=0.1)
			print ("Opening mara control for left arm on: \t \t ---> \t" + str(self.interface.name))
		except Exception,e:
			print("Could not open usb interface for left arm on: \t ---> \t" + str(self.device))


	# decodes hex response from serial query for joint values
	def decodeJointValues(self, joint_array, verbose=False):
		# degrees per encoder step
		joint_angles = {'j1':None, 'j2':None, 'j3':None, 'j4':None, 'j5':None, 'j6':None, 'g':None}

		for i in range(7):
			val = int(joint_array[i*4:(i*4)+4], 16)
			print i, val
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

		if verbose:
			rospy.loginfo("Joint angles received " + str(joint_angles))

		return joint_angles

	# queries the arm for joint angles and returns dictionary of 'joint':angle pairs for each arm
	def getJointAngles(self):
		if self.interface:
			# send command to get left arm joint angles
			self.interface.write('p\r')
			self.joint_array = self.interface.read(29)
			if len(self.joint_array) != 0:
				# convert hex response to joint angles
				self.joint_angles = self.decodeJointValues(self.joint_array)
			else:
				print "Warning: failed to receive joint state from left arm"

		return self.joint_angles

	# static velocity control (only using 10, 20, or 30 deg per sec builtin commands)
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
				print angles

				if lower < 0:
					stop_condition = inRange(angles[joint_name], 360 + lower, 360) or inRange(angles[joint_name], 0, upper)
				else:
					stop_condition = inRange(angles[joint_name], lower, upper)

				time.sleep(rate)

			#self.stopMovement()
			self.commandJointVelocity(joint_name, 0)

				# given the current angle of a joint and desired angle, returns the signed velocity required for
	# shortest movement
	def signVelocity(self, current_angle, desired_angle, vel, gripper=False):
		# check which direction we should rotate the joint

		# set the sign
		if not gripper:
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

	# command a single joint at 'vel' degrees per sec
	def commandJointVelocity(self, joint, vel):
		command = None
		sign = '#'

		if 'g' not in joint:
			joint = ''.join([i for i in joint if i.isdigit()])

		if self.interface:
			command = 'jv ' + str(joint) + sign + str(vel) + '\r'
			self.interface.write('jv ' + str(joint) + sign + str(vel) + '\r')

	# getst the gripper in a good starting position after powering the arm
	def initializeGripper(self, left_right='l'):
		self.commandJointVelocity(joint='g', vel=10)
		# open the gripper a bit
		print "Initializing", left_right, "gripper"
		angles = self.getJointAngles()

		self.commandJointAngle_STATIC('g', angle=30, vel=20)
		self.commandJointAngle_STATIC('g', angle=20, vel=20)

		print "Done!"









							


rospy.init_node('joint_angle_getter')

mara_serial = mara_serial()
mara_serial.initializeGripper()

mara_serial.commandJointVelocity('j5', 10)
t1 = time.time()
t2 = time.time()
while t2 - t1 < 30:
	print "--------------------------------------------------------------------------"
	print "ANGLES", mara_serial.getJointAngles()
	print "--------------------------------------------------------------------------"
	t2 = time.time()




