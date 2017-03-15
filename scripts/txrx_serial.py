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
# Before running:
# - $ sudo chmod +777 /dev/ttyUSB0
# - $ sudo modprobe usbserial vendor=0x067b product=0x2303 

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

		self.l_vels = {'j1':0, 'j2':0, 'j3':0, 'j4':0, 'j5':0, 'j6':0, 'g':0}
		self.r_vels = {'j1':0, 'j2':0, 'j3':0, 'j4':0, 'j5':0, 'j6':0, 'g':0}

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

	# command a single joint at 'vel' degrees per sec
	def commandAllJointVelocities(self, vels, left_right='l'):
		command_l = None
		command_r = None
		sign = '#'

		cmd_str = ['0','0','0','0','0','0','0']

		for j,v in vels.iteritems():
			if j == 'j1':
				idx = 0
			elif j == 'j2':
				idx = 1
			elif j == 'j3':
				idx = 2
			elif j == 'j4':
				idx = 3
			elif j == 'j5':
				idx = 4
			elif j == 'j6':
				idx = 5
			elif j == 'g':
				idx = 6

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
		if 'l' in left_right and self.interface_l:
			self.interface_l.write('m' + cmd_str + '\r')

		if 'r' in left_right and self.interface_r:
			self.interface_r.write('m' + cmd_str + '\r')


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

	# static velocity control (only using 10, 20, or 30 deg per sec builtin commands)
	def commandJointAngle_STATIC(self, joint_name, angle, vel, left_right, variable_velocity=False):

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

	# PWM control for all joints, commands joints to a specific position (blocking)
	def commandAllJointAngles(self, angles, vels, left_right, variable_velocity=False):
		
		# command the left arm
		if 'l' in left_right and self.interface_l:

			l_angles, r_angles = self.getJointAngles(left_right=left_right)

			lowers = {'j1':0, 'j2':0, 'j3':0, 'j4':0, 'j5':0, 'j6':0, 'g':0} 		# upper range of stopping position
			uppers = {'j1':0, 'j2':0, 'j3':0, 'j4':0, 'j5':0, 'j6':0, 'g':0}		# lower range of stopping position

			measured_vels = {'j1':0, 'j2':0, 'j3':0, 'j4':0, 'j5':0, 'j6':0, 'g':0}
			zero_vels = {'j1':0, 'j2':0, 'j3':0, 'j4':0, 'j5':0, 'j6':0, 'g':0} 	# zero velocity commands to joints
			cmd_vels = {'j1':0, 'j2':0, 'j3':0, 'j4':0, 'j5':0, 'j6':0, 'g':0} 		# high velocity commands to joints
			stop_conditions = {'j1':False, 'j2':False, 'j3':False, 'j4':False, 'j5':False, 'j6':False, 'g':False}
			duty_ratios = {'j1':0.0, 'j2':0.0, 'j3':0.0, 'j4':0.0, 'j5':0.0, 'j6':0.0, 'g':0.0}

			# iterate through joints and set stopping range
			for joint_name in angles.keys():
				if 'g' in joint_name:
					tolerance = 5.0
					isGripper = True
				else:
					tolerance = 5.0
					isGripper = False

				# check if were moving clockwise or counterclockwise
				vel = self.signVelocity(l_angles[joint_name], angles[joint_name], vels[joint_name], isGripper)

				# set stopping tolerances
				lower = angles[joint_name] - tolerance
				upper = angles[joint_name] + tolerance

				# evaluate stopping conditions for all joints
				if lower < 0:
					stop_condition = inRange(l_angles[joint_name], 360 + lower, 360) or inRange(l_angles[joint_name], 0, upper)
				if upper > 360:
					stop_condition = inRange(l_angles[joint_name], lower, 360) or inRange(l_angles[joint_name], 0, upper % 360)
				else:
					stop_condition = inRange(l_angles[joint_name], lower, upper)

				# set duty cycles
				if abs(vel) <= 10:
					duty_ratio = (abs(vel)) / 10.0
				elif abs(vel) <= 20:
					duty_ratio = (abs(vel)) / 20.0
				else:
					duty_ratio = (abs(vel)) / 30.0

				# set joint commands
				# TODO: here is where pid comes in?
				cmd_vels[joint_name] = vel
				duty_ratios[joint_name] = duty_ratio
				stop_conditions[joint_name] = stop_condition


			period = 0.2
			while not all(stop_conditions.values()):


				print "STOP CONDIS", stop_conditions
				print "CMD VELS", cmd_vels
				print "DUTY RATIOS", duty_ratios

				t1 = time.time()
				l_angles_prev = l_angles 	# save old joint values

				# send duty cycle
				self.commandAllJointVelocities(cmd_vels, left_right=left_right)
				t2 = time.time()
				# place the sensor polling in duty pulse
				l_angles, r_angles =  self.getJointAngles()
				t3 = time.time()

				while any(cmd_vels.values()):
					t4 = time.time()
					curr_duty = (t4 - t2) / period

					# pull down PWM after duty cycle elapses
					isChangedVel = False
					for j,r in duty_ratios.iteritems():
						if r < curr_duty and cmd_vels[j] != 0:
							isChangedVel = True
							cmd_vels[j] = 0

					if isChangedVel:
						# low velocity command
						self.commandAllJointVelocities(cmd_vels, left_right=left_right)
						time.sleep(0.0008)

				t5 = time.time()
				# iterate through joints and set stopping range
				for joint_name in angles.keys():
					if 'g' in joint_name:
						tolerance = 5.0
						isGripper = True
					else:
						tolerance = 5.0
						isGripper = False

					measured_vel = (l_angles[joint_name] - l_angles_prev[joint_name]) / period
					measured_vels[joint_name] = measured_vel

					# check velocity command if we're moving clockwise or counterclockwise
					vel = self.signVelocity(l_angles[joint_name], angles[joint_name], vels[joint_name], isGripper)

					# set stopping tolerances
					lower = angles[joint_name] - tolerance
					upper = angles[joint_name] + tolerance

					# evaluate stopping conditions for all joints
					if lower < 0:
						stop_condition = inRange(l_angles[joint_name], 360 + lower, 360) or inRange(l_angles[joint_name], 0, upper)
					if upper > 360:
						stop_condition = inRange(l_angles[joint_name], lower, 360) or inRange(l_angles[joint_name], 0, upper % 360)
					else:
						stop_condition = inRange(l_angles[joint_name], lower, upper)

					# set duty cycles
					if abs(vel) <= 10:
						duty_ratio = (abs(vel)) / 10.0
					elif abs(vel) <= 20:
						duty_ratio = (abs(vel)) / 20.0
					else:
						duty_ratio = (abs(vel)) / 30.0

					# set joint commands
					# TODO: here is where pid comes in?
					kd = 0.1
					delta_v = vel - measured_vel
					pid_vel = vel + kd*delta_v
					print "NEW", vel, measured_vel, pid_vel

					cmd_vels[joint_name] = pid_vel
					duty_ratios[joint_name] = duty_ratio
					stop_conditions[joint_name] = stop_condition

				

				print "VELOCITIES", measured_vels
				print "ANGLES", l_angles
				t6 = time.time()
				t_wait = period - (t6 - t1)
				time.sleep(max(t_wait, 0.0))
				t7 = time.time()





	def initializeGripper(self, left_right='l'):
		self.commandJointVelocity(joint='g', vel=10, left_right=left_right)
		# open the gripper a bit
		print "Initializing", left_right, "gripper"
		l_angles, r_angles = self.getJointAngles(left_right=left_right)

		self.commandJointAngle_STATIC('g', angle=30, vel=20, left_right=left_right)
		self.commandJointAngle_STATIC('g', angle=20, vel=20, left_right=left_right)

		print "Done!"
			


	def foldArm(self, left_right='l'):
		desired_angles = {'j1':45, 'j2':265, 'j3':90, 'j4':90, 'j5':110, 'j6':180, 'g':20}
		vel_command = {'j1':10, 'j2':10, 'j3':10, 'j4':10, 'j5':10, 'j6':10, 'g':10}
		
		# iterate through joints from gripper to base and reset joint positions
		for joint_name in self.orderedJoints:
			print "Folding joint", joint_name
			self.commandJointAngle_STATIC(joint_name, desired_angles[joint_name], vel_command[joint_name], left_right=left_right)

	def unfoldArm(self, left_right='l'):
		desired_angles = {'g': 40, 'j4': 180, 'j5': 90, 'j6': 120, 'j1': 40, 'j2': 120, 'j3': 240}
		vel_command = {'j1':10, 'j2':10, 'j3':10, 'j4':10, 'j5':10, 'j6':10, 'g':10}
		
		# iterate through joints from gripper to base and reset joint positions
		for joint_name in self.orderedJoints:
			print "Folding joint", joint_name
			self.commandJointAngle_STATIC(joint_name, desired_angles[joint_name], vel_command[joint_name], left_right=left_right)


	def zeroArm(self, left_right='l'):
		desired_angles = {'j1':0, 'j2':0, 'j3':0, 'j4':0, 'j5':0, 'j6':0, 'g':30}
		vel_command = {'j1':10, 'j2':10, 'j3':10, 'j4':10, 'j5':10, 'j6':10, 'g':10}
		
		# iterate through joints from gripper to base and reset joint positions
		for joint_name in self.orderedJoints:
			print "Folding joint", joint_name
			self.commandJointAngle_STATIC(joint_name, desired_angles[joint_name], vel_command[joint_name], left_right=left_right)
		

	def testVelocity(self, joint, angle, left_right='l'):
		mara_serial.commandJointAngle(joint, angle, 10, left_right='l')
		


	# commands all joints through a series of movements to determine PID coefficients for accurate control
	def calibrateController(self):
		now = datetime.datetime.now()
		calib_file = open(self.project_path + 'arm_calib_'+now.strftime("%m-%d-%Y")+'.txt', 'wa')

		errors_pose = {'j1':[], 'j2':[], 'j3':[], 'j4':[], 'j5':[], 'j6':[], 'g':[]}
		errors_vel = {'j1':[], 'j2':[], 'j3':[], 'j4':[], 'j5':[], 'j6':[], 'g':[]}

		for joint in self.orderedJoints:
			calib_file.write(str(joint)+'\n')
			for vel in range(1,30):
				error_pose_pos, error_vel_pos = self.moveWithError(joint, 20, vel)
				time.sleep(0.5)
				error_pose_neg, error_vel_neg = self.moveWithError(joint, -20, -vel)
				#errors_pose[joint].append(error_pose_pos)
				#errors_vel[joint].append(error_vel)
				calib_file.write(str(error_pose_pos) + ", " + str(error_pose_neg) + ", " + str(error_vel_pos) + ", " + str(error_vel_neg)+'\n')
				calib_file.flush()
				time.sleep(0.5)
			calib_file.write('\n')


	def loadCalibration(self, fname):
		pass

left_right = 'l'
mara_serial = mara_serial(left_right=left_right)
print mara_serial.getJointAngles()
mara_serial.initializeGripper(left_right=left_right)
#mara_serial.unfoldArm(left_right=left_right)
#mara_serial.calibrateController()
#mara_serial.testVelocity( 'j1', 30, left_right=left_right)
#print mara_serial.getJointAngles()

#desired_angles = {'g': 40, 'j4': 180, 'j5': 90, 'j6': 120, 'j1': 40, 'j2': 140, 'j3': 270}
#vel_commands = {'j1':0, 'j2':0, 'j3':5, 'j4':0, 'j5':0, 'j6':0, 'g':0}
#mara_serial.commandAllJointAngles(desired_angles, vel_commands, left_right, variable_velocity=False)

desired_angles = {'g': 40, 'j4': 0, 'j5': 0, 'j6': 0, 'j1': 0, 'j2': 0, 'j3': 0}
vel_commands = {'j1':5, 'j2':5, 'j3':5, 'j4':5, 'j5':5, 'j6':5, 'g':5}
mara_serial.commandAllJointAngles(desired_angles, vel_commands, left_right, variable_velocity=False)

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


