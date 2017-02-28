#!/usr/bin/env python
import serial
import cv2
import sys, time, os
import rospy
from mara_utils import *
import datetime

# NOTES
# - On startup:
# 	- gripper auto closes, need to command positive velocity to open before sending more commands 
# sudo modprobe usbserial vendor=0x067b product=0x2303 

class mara_serial():
	def __init__(self, left_right='lr'):
		self.device_l = "/dev/ttyUSB0"
		self.device_r = "/dev/ttyUSB1"
		self.interface_l = None
		self.interface_r = None

		self.joint_array_l = None
		self.joint_array_r = None
		self.joint_angles_l = None
		self.joint_angles_r = None

		self.orderedJoints = ['g', 'j6', 'j5', 'j4', 'j3', 'j2', 'j1']

		self.project_path = "/home/carrt/Dropbox/catkin_ws/src/mara/"
		
		if 'l' in left_right:
			try:
				self.interface_l = serial.Serial(self.device_l, 9600, timeout=0.1)
				print ("Opening mara control for left arm on: \t \t ---> \t" + str(self.interface_l.name))
			except Exception,e:
				print("Could not open usb interface for left arm on: \t ---> \t" + str(self.device_l))

		if 'r' in left_right:
			try:
				self.interface_r = serial.Serial(self.device_r, 9600, timeout=0.1)
				print ("Opening mara control for right arm on: \t \t ---> \t" + str(self.interface_r.name))
			except Exception,e:
				print("Could not open usb interface for right arm on: \t ---> \t" + str(self.device_r))
		
	# decodes hex response from serial query for joint values
	def decodeJointValues(self, joint_array, verbose=False):
		# degrees per encoder step
		
		joint_angles = {'j1':None, 'j2':None, 'j3':None, 'j4':None, 'j5':None, 'j6':None, 'g':None}

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

		if verbose:
			rospy.loginfo("Joint angles received " + str(joint_angles))

		return joint_angles

	# queries the arm for joint angles and returns dictionary of 'joint':angle pairs for each arm
	def getJointAngles(self, left_right='l'):

		if 'l' in left_right and self.interface_l:
			# send command to get left arm joint angles
			self.interface_l.write('p\r')
			self.joint_array_l = self.interface_l.read(29)
			if len(self.joint_array_l) != 0:
				# convert hex response to joint angles
				self.joint_angles_l = self.decodeJointValues(self.joint_array_l)
			else:
				print "Warning: failed to receive joint state from left arm"

		if 'r' in left_right and self.interface_r:
			# send command to get right arm joint angles
			self.interface_r.write('p\r')
			self.joint_array_r = self.interface_r.read(29)
			
			if len(self.joint_array_r) != 0:
				# convert hex response to joint angles
				self.joint_angles_r = self.decodeJointValues(self.joint_array_r)
			else:
				print "Warning: failed to receive joint state from right arm"

		return self.joint_angles_l, self.joint_angles_r

	# command a single joint at 'vel' degrees per sec
	def commandJointVelocity(self, joint, vel, left_right='l'):
		command_l = None
		command_r = None
		sign = '#'

		if 'g' not in joint:
			joint = ''.join([i for i in joint if i.isdigit()])

		if 'l' in left_right and self.interface_l:
			command_l = 'jv ' + str(joint) + sign + str(vel) + '\r'
			self.interface_l.write('jv ' + str(joint) + sign + str(vel) + '\r')

		if 'r' in left_right and self.interface_r:
			command_r = 'jv ' + str(joint) + sign + str(vel) + '\r'
			self.interface_r.write('jv ' + str(joint) + sign + str(vel) + '\r')

	# send carriage return to stop arm movement
	def stopMovement(self):
		if 'l' in left_right and self.interface_l:
			self.interface_l.write('\r')
			print self.interface_l.read(5)
			print self.interface_l.read(29)

		if 'r' in left_right and self.interface_r:
			self.interface_r.write('\r')
			print self.interface_r.read(5)
			print self.interface_r.read(29)

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

	# attempts to command the joint to a desired angle at a certain velocity
	def commandJointAngle(self, joint_name, angle, vel, left_right, variable_velocity=False):

		l_angles, r_angles = self.getJointAngles(left_right=left_right)
		rate = 0.01
		# set encoder error tolerance for gripper and arm joints (larger error for gripper?)
		if 'g' in joint_name:
			tolerance = 1.0
			isGripper = True
		else:
			tolerance = 0.3
			isGripper = False

		# command the left arm
		if 'l' in left_right and self.interface_l and l_angles:

			vel = self.signVelocity(l_angles[joint_name], angle, vel, isGripper)
			self.commandJointVelocity(joint_name, vel, left_right=left_right)
			l_angles, r_angles = self.getJointAngles(left_right=left_right)

			lower = angle - tolerance
			upper = angle + tolerance

			# set stop condition for joint movement with wrap-around at 360 degrees
			print "Moving joint", joint_name, 'to angle (', lower, '-', upper, ') at rate', vel
			if lower < 0:
				stop_condition = inRange(l_angles[joint_name], 360 + lower, 360) or inRange(l_angles[joint_name], 0, upper)
			else:
				stop_condition = inRange(l_angles[joint_name], lower, upper)

			# iteratively command joint at velocity and check position
			while not stop_condition:
				vel = self.signVelocity(l_angles[joint_name], angle, vel, isGripper)
				self.commandJointVelocity(joint_name, vel, left_right=left_right)
				l_angles, r_angles = self.getJointAngles(left_right=left_right)
				print l_angles

				if lower < 0:
					stop_condition = inRange(l_angles[joint_name], 360 + lower, 360) or inRange(l_angles[joint_name], 0, upper)
				else:
					stop_condition = inRange(l_angles[joint_name], lower, upper)

				time.sleep(rate)

			#self.stopMovement()
			self.commandJointVelocity(joint_name, 0, left_right=left_right)
			print "DONE"

	def initializeGripper(self, left_right='l'):
		self.commandJointVelocity(joint='g', vel=10, left_right=left_right)
		# open the gripper a bit
		print "Initializing", left_right, "gripper"
		l_angles, r_angles = self.getJointAngles(left_right=left_right)

		self.commandJointAngle('g', angle=30, vel=20, left_right=left_right)
		self.commandJointAngle('g', angle=20, vel=20, left_right=left_right)

		print "Done!"
			


	def foldArm(self, left_right='l'):
		desired_angles = {'j1':45, 'j2':265, 'j3':90, 'j4':90, 'j5':110, 'j6':180, 'g':20}
		vel_command = {'j1':10, 'j2':10, 'j3':10, 'j4':10, 'j5':10, 'j6':10, 'g':10}
		
		# iterate through joints from gripper to base and reset joint positions
		for joint_name in self.orderedJoints:
			print "Folding joint", joint_name
			self.commandJointAngle(joint_name, desired_angles[joint_name], vel_command[joint_name], left_right=left_right)

	def unfoldArm(self, left_right='l'):
		desired_angles = {'g': 30, 'j4': 180, 'j5': 90, 'j6': 120, 'j1': 40, 'j2': 100, 'j3': 240}
		vel_command = {'j1':10, 'j2':10, 'j3':10, 'j4':10, 'j5':10, 'j6':10, 'g':10}
		
		# iterate through joints from gripper to base and reset joint positions
		for joint_name in self.orderedJoints:
			print "Folding joint", joint_name
			self.commandJointAngle(joint_name, desired_angles[joint_name], vel_command[joint_name], left_right=left_right)


	def zeroArm(self, left_right='l'):
		desired_angles = {'j1':0, 'j2':0, 'j3':0, 'j4':0, 'j5':0, 'j6':0, 'g':30}
		vel_command = {'j1':10, 'j2':10, 'j3':10, 'j4':10, 'j5':10, 'j6':10, 'g':10}
		
		# iterate through joints from gripper to base and reset joint positions
		for joint_name in self.orderedJoints:
			print "Folding joint", joint_name
			self.commandJointAngle(joint_name, desired_angles[joint_name], vel_command[joint_name], left_right=left_right)
		

	def testVelocity(self, vel):
		mara_serial.commandJointAngle('j1', 0, 10, left_right='l')
		l_angles, r_angles =  mara_serial.getJointAngles()

		fast_command = 10
		period = 0.20
		t_start = time.time()
		while l_angles['j1'] < 30 or l_angles['j1'] > 300:
			t1 = time.time()
			if vel <= 5:
				duty_ratio = vel / 10.0
				fast_command = 10
				slow_command = 0
			elif vel <= 10:
				duty_ratio = vel / 20.0
				fast_command = 20
				slow_command = 0
			elif vel <= 20:
				duty_ratio = vel / 20.0
				fast_command = 20
				slow_command = 10
			else:
				duty_ratio = vel / 30.0
				fast_command = 30
				slow_command = 20

			print "Duty ratio", duty_ratio, 1-duty_ratio
			print "Duty time", duty_ratio*period, "Off time", (1-duty_ratio)*period
			self.commandJointVelocity('j1', vel=fast_command, left_right=left_right)
			t3 = time.time()
			# place the sensor polling in the larger part of the PWM pulse
			if duty_ratio >= 0.5:
				# poll the encoders and sleep for duty cycle
				l_angles, r_angles =  mara_serial.getJointAngles()
				t4 = time.time()
				t_poll = (t4 - t3)
				time.sleep( max((period * (duty_ratio) - t_poll), 0.0))
			else:
				# sleep the remaining period
				time.sleep( max(((period) * (duty_ratio)), 0.0 ))

			self.commandJointVelocity('j1', vel=slow_command, left_right=left_right)
			t3 = time.time()
			if duty_ratio < 0.5:
				# poll the encoders and sleep for duty cycle
				l_angles, r_angles =  mara_serial.getJointAngles()
				t4 = time.time()
				t_poll = (t4 - t3)
				time.sleep( max((period * (1- duty_ratio) - t_poll), 0.0))
			else:
				# sleep the remaining period
				time.sleep( max(((period) * (1-duty_ratio)), 0.0 ))

			t2 = time.time()
			print "Iteration:", t2 - t1, "secs"
			print "Poll:", t_poll, "secs"
			print "Total:", t2 - t_start, "secs"

		mara_serial.commandJointVelocity('j1', 0, left_right='l')
		l_angles, r_angles =  mara_serial.getJointAngles()
		print l_angles

	def moveWithError(self, joint, degrees, vel):
	
		l_angles, r_angles =  mara_serial.getJointAngles()

		start_angles = l_angles
		fast_command = 10
		period = 0.20
		t_start = time.time()
		while l_angles[joint] < start_angles[joint] + degrees:
			t1 = time.time()
			if vel <= 5:
				duty_ratio = vel / 10.0
				fast_command = 10
				slow_command = 0
			elif vel <= 10:
				duty_ratio = vel / 20.0
				fast_command = 20
				slow_command = 0
			elif vel <= 20:
				duty_ratio = vel / 20.0
				fast_command = 20
				slow_command = 10
			else:
				duty_ratio = vel / 30.0
				fast_command = 30
				slow_command = 20

			#print "Duty ratio", duty_ratio, 1-duty_ratio
			#print "Duty time", duty_ratio*period, "Off time", (1-duty_ratio)*period
			self.commandJointVelocity(joint, vel=fast_command, left_right=left_right)
			t3 = time.time()
			# place the sensor polling in the larger part of the PWM pulse
			if duty_ratio >= 0.5:
				# poll the encoders and sleep for duty cycle
				l_angles, r_angles =  mara_serial.getJointAngles()
				t4 = time.time()
				t_poll = (t4 - t3)
				time.sleep( max((period * (duty_ratio) - t_poll), 0.0))
			else:
				# sleep the remaining period
				time.sleep( max(((period) * (duty_ratio)), 0.0 ))

			self.commandJointVelocity(joint, vel=slow_command, left_right=left_right)
			t3 = time.time()
			if duty_ratio < 0.5:
				# poll the encoders and sleep for duty cycle
				l_angles, r_angles =  mara_serial.getJointAngles()
				t4 = time.time()
				t_poll = (t4 - t3)
				time.sleep( max((period * (1- duty_ratio) - t_poll), 0.0))
			else:
				# sleep the remaining period
				time.sleep( max(((period) * (1-duty_ratio)), 0.0 ))

			t2 = time.time()
			#print "Iteration:", t2 - t1, "secs"
			#print "Poll:", t_poll, "secs"
			#print "Total:", t2 - t_start, "secs"

		mara_serial.commandJointAngle(joint, start_angles[joint], 10, left_right='l')
		mara_serial.commandJointVelocity(joint, 0, left_right='l')
		return abs(l_angles[joint] - (start_angles[joint] + degrees)),  (degrees / (t2 - t_start)) - vel

	# commands all joints through a series of movements to determine PID coefficients for accurate control
	def calibrateController(self):
		now = datetime.datetime.now()
		calib_file = open(self.project_path + 'arm_calib_'+now.strftime("%m-%d-%Y")+'.txt', 'w')

		errors_pos = {'j1':[], 'j2':[], 'j3':[], 'j4':[], 'j5':[], 'j6':[], 'g':[]}
		errors_vel = {'j1':[], 'j2':[], 'j3':[], 'j4':[], 'j5':[], 'j6':[], 'g':[]}

		for joint in self.orderedJoints:
			calib_file.write(str(joint)+'\n')
			for vel in range(5,30):
				error_pos, error_vel = self.moveWithError(joint, 20, vel)
				errors_pos[joint].append(error_pos)
				errors_vel[joint].append(error_vel)
				calib_file.write(str(error_pos) + ", " + str(error_vel)+'\n')
			calib_file.write('\n')

left_right = 'l'
mara_serial = mara_serial(left_right=left_right)
print mara_serial.getJointAngles()
mara_serial.initializeGripper(left_right=left_right)
mara_serial.unfoldArm(left_right=left_right)
mara_serial.calibrateController()
#print mara_serial.getJointAngles()

#mara_serial.commandJointAngle('g', 60, 20, left_right='l')
#mara_serial.testVelocity(30)
#mara_serial.foldArm(left_right=left_right)
#mara_serial.zeroArm(left_right=left_right)

# how fast to query joint angles via usb
joint_state_rate = 1.0

# joint velocities in degrees per second
#vel = {'j1':1, 'j2':0, 'j3':1, 'j4':0, 'j5':1, 'j6':0, 'g':1}
#mara_serial.commandJointVelocity



"""
while cv2.waitKey(30) != 27:
	t_start = time.time()
	
	# get joint angles
	l_angles, r_angles = mara_serial.getJointAngles(left_right='lr')
	print 'left_arm:', l_angles
	print 'right_arm:', r_angles

	# wait specified time to query joint states again
	t_end = time.time()
	while (t_end - t_start) < joint_state_rate:
		time.sleep(0.001)
		t_end = time.time()
"""


