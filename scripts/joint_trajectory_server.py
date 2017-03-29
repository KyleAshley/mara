#!/usr/bin/env python
import serial
import cv2
import sys, time, os
from subprocess import call
import rospy
from mara_utils import *
import datetime
import math 
import stat

import moveit_msgs.msg
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from moveit_msgs.msg import MoveGroupActionResult, ExecuteTrajectoryActionResult

import matplotlib.pyplot as plt
import numpy as np

import threading


# NOTES
# - On startup:
# 	- gripper auto closes, need to command positive velocity to open before sending more commands 
# Before running:
# - $ sudo chmod +777 /dev/ttyUSB0
# - $ sudo modprobe usbserial vendor=0x067b product=0x2303 

class mara_serial():
	def __init__(self, device_num=None, left_right='l'):

		self.isInitialized = False
		self.interface = None 			# python serial interface

		device_list = self.getUSBDeviceNames()
		if device_num == None:
			self.device_name = device_list[0] 					# linux device name for serial interface
		else:
			self.device_name = device_list[device_num] 					# linux device name for serial interface

		call(["sudo", "chmod", "+777", self.device_name]) 	# give permissions to access device file

		# open the serial interface to the arm
		try:
			self.interface = serial.Serial(self.device_name, 9600, timeout=0.1)
			print self.interface.name
			self.isInitialized = True
		except:
			print("Could not open usb interface for left arm on: \t ---> \t" + str(self.device_name))
			print "Attempting to load the kernel module..."
			os.system("sudo modprobe usbserial vendor=0x067b product=0x2303")

			try:
				print ("Opening mara control on: \t \t ---> \t" + str(self.interface.name))
				self.interface = serial.Serial(self.device, 9600, timeout=0.1)
				self.isInitialized = True

			except Exception, e:
				print "Failed...exiting"
				self.isInitialized = False

		# need a better method here
		if 'r' in left_right:
			self.arm_name = "right"
		else:
			self.arm_name = "left"

		print "Initializing", self.arm_name, "arm..."

		self.rate = 0.2 				# pulse width period for sending joint commands through USB  (Dont change it) (0.3 is safe, 0.2 is extreme)

		# serial communication results
		self.joint_array = None 	# most recent hex array from encoders
		self.joint_angles = None 	# most recent decoded joint angles
		self.trajectory_queue = [] 	# trajectory queue contains incoming arm trajectory goals
		self.trajectory_queue_len = 1

		# joint names from ee -> base
		self.orderedJoints = ['g', 'j6', 'j5', 'j4', 'j3', 'j2', 'j1']

		# project directory
		self.project_path = "/home/carrt/Dropbox/catkin_ws/src/mara/"

		# ros subs and pubs
		self.trajectory_sub = rospy.Subscriber("/mara/joint_trajectory_controller/trajectory", JointTrajectory, self.updateJointTrajectory)
		self.rviz_trajectory_sub = rospy.Subscriber('/move_group/result', MoveGroupActionResult, self.updateMoveitTrajectory, queue_size=4)

		self.joint_state_pub = rospy.Publisher("/mara/joint_positions", JointState, queue_size=2)
		self.rviz_joint_state_pub = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=1)

		self.trajectory_result_pub = rospy.Publisher('/execute_trajectory/result', ExecuteTrajectoryActionResult, queue_size=1)

		
			
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
	def getJointAngles(self):
		if self.interface:
			# send command to get left arm joint angles
			self.interface.write('p\r')
			self.joint_array = self.interface.read(29)

			if len(self.joint_array) != 0 and ">Stop" not in str(self.joint_array):
				# convert hex response to joint angles
				self.joint_angles = self.decodeJointValues(self.joint_array)
				self.joint_angles['j3'] = self.joint_angles['j3'] - self.joint_angles['j2'] 		# offset linked encoders
				if self.joint_angles['j3'] < 0:
					self.joint_angles['j3'] += 360

			else:
				if ">Stop" in str(self.joint_array):
					print "Warning: joints near singularity..."
				else:
					print "Warning: failed to receive joint state from left arm"

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


	# command a single joint at 'vel' degrees per sec using the 'm' command
	def commandAllJointVelocities(self, vels, left_right='l'):
		command = None
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
		
		print trajectory.points

		limitExceeded = True

		while(limitExceeded):
			limitExceeded = False
			# iterate over points and joint names
			for p1 in trajectory.points:
				for n in range(len(trajectory.joint_names)):
					# if the velocity threshold is exceeded, double the timestamps
					if p1.velocities[n] > 0.3 or p1.velocities[n] < -0.3:
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
			print p1.positions

		new_trajectory = trajectory
		return new_trajectory




	# update the trajectory queue with new goals from ROS sub
	def updateJointTrajectory(self, msg):
		print 'Received new joint trajectory'
		#self.trajectory_queue.append(msg)
		#self.trajectory_queue = self.trajectory_queue[-self.trajectory_queue_len:]

	# receives trajectories from the moveit topic and adds them to the execution queue
	def updateMoveitTrajectory(self, msg):
		if self.arm_name in msg.result.planned_trajectory.joint_trajectory.joint_names[0]:
			print 'Received new RVIZ joint trajectory for', self.arm_name 
			trajectory = msg.result.planned_trajectory.joint_trajectory
			short_names = {'joint1':'j1', 'joint2':'j2', 'joint3':'j3', 'joint4':'j4', 'joint5':'j5', 'joint6':'j6', 'gripper':'g'}
			new_names = []
			for n in trajectory.joint_names:
				print n
				name = n.strip('left_')
				name = name.strip('right_')
				new_names.append(short_names[name])
			trajectory.joint_names = new_names

			trajectory = self.enforceMaxTrajectoryVelocity(trajectory)

			self.trajectory_queue.append(trajectory)
			#self.trajectory_queue = self.trajectory_queue[-self.trajectory_queue_len:]


	# publish the robot joint states
	def updateJointPositions(self, angles):
		msg = JointState()
		msg.header = Header()
		msg.header.stamp = rospy.Time.now()

		if self.arm_name == "left":
			full_names = {'j1':'left_joint1', 'j2':'left_joint2', 'j3':'left_joint3', 'j4':'left_joint4', 'j5':'left_joint5', 'j6':'left_joint6', 'g':'left_gripper'}
		elif self.arm_name == "right":
			full_names = {'j1':'right_joint1', 'j2':'right_joint2', 'j3':'right_joint3', 'j4':'right_joint4', 'j5':'right_joint5', 'j6':'right_joint6', 'g':'right_gripper'}
		else:
			full_names = {'j1':'joint1', 'j2':'joint2', 'j3':'joint3', 'j4':'joint4', 'j5':'joint5', 'j6':'joint6', 'g':'gripper'}

		names = []
		positions = []
		for name, theta in angles.iteritems():
			names.append(full_names[name])
			positions.append((theta - 180.0) * (math.pi/180.0))

		msg.name = names
		msg.position = positions

		self.joint_state_pub.publish(msg)


	# main control loop
	# - obtains joint positions and publishes to:	 /mara/<arm>/joint_positions/<joint_name>
	# - accepts trajectories from:					 /move_group/goal  	(TODO: needs to change?)
	# - executes joint trajectories using PD control loop
	# - TODO: add joint limits
	def startJointTrajectoryControlLoop(self):
		
		if self.interface:

			period = self.rate  			# should be 0.2 
			print "Entering control loop"

			while not rospy.is_shutdown():

				velocity_windows = {'j1':[], 'j2':[], 'j3':[], 'j4':[], 'j5':[], 'j6':[], 'g':[]}		# records instantaneous velocities of joints

				# while no trajectories are being executed, simply update the joint state
				angles = self.getJointAngles() 
				self.updateJointPositions(angles)

				# when there is a new trajectory, execute it
				# trajectory is a series of [pos, timestamp] for each joint
				if len(self.trajectory_queue) > 0:

					print "Executing joint trajectory...", self.arm_name
					curr_trajectory = self.trajectory_queue[0]

					t_start = time.time()
					angles_prev = angles
					waypoint_prev = angles

					positions_err_dict = {'j1':[], 'j2':[], 'j3':[], 'j4':[], 'j5':[], 'j6':[], 'g':[]}
					velocities_err_dict = {'j1':[], 'j2':[], 'j3':[], 'j4':[], 'j5':[], 'j6':[], 'g':[]}


					# iterate through waypoints in the trajectory
					for waypoint, waypoint_num in zip(curr_trajectory.points, range(len(curr_trajectory.points))):

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
						final_error_tolerance = 3

						waypoint_angles = angles 		# joint values when waypoint was started
						#print "WAYPOINT--------", waypoint_num,"of", len(curr_trajectory.points),"---------------------------------------------------"
						#print "check interval = ", t_interval
						#print 'check time = ', t_subdiv - t_start
						#print 'current waypoint duration', waypoint.time_from_start.to_sec()

						error_correct_attempts = 0
						# if the waypoint goal time has not elapsed, or we reached the last waypoint with significant error
						while t_subdiv < t_start + waypoint.time_from_start.to_sec() or (last_waypoint and error_warning and error_correct_attempts < 20):

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

							error_warning = False 			# set true if large joint errors exist

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

								if abs(err_theta) > final_error_tolerance:
									error_warning = True

								# record the instantaneous velocity ina  sliding window
								velocity_windows[joint_name].append(delta_theta/t_interval)
								if len(velocity_windows) > window_size:
									velocity_windows[joint_name] = velocity_windows[joint_name][-window_size:]
								
								#measured_vels[joint_name] = (angles[joint_name] - angles_prev[joint_name])/period 					# instantaneous measured vel
								measured_vels[joint_name] = sum(velocity_windows[joint_name])/len(velocity_windows[joint_name]) 	# sliding window measured vel
								err_vel = goal_vels[joint_name] - measured_vels[joint_name]

								positions_err_dict[joint_name].append(err_theta)
								velocities_err_dict[joint_name].append(err_vel)
								#print joint_name, 'angle', angles[joint_name], 'delta', delta_theta, 'goal', goal_vels[joint_name], 'actual', measured_vels[joint_name]

								kp = 0.05
								kd = 0.1
								#pid_vel = vel
								#print min((kp * abs(err_theta)), 1.0)
								pid_vel = vel * min((kp * abs(err_theta)), 1.0) + (kd * err_vel)
								#pid_vel = vel + (kd * err_vel) + (kp * err_theta)

								#print 'cmd', vel, 'err vel', err_vel, 'err pos', err_theta, 'pid', pid_vel 

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
								time.sleep(max(t_wait, 0.0))
								t7 = time.time()
							else:
								initial_command = False

							#print "actual period:", t7 - t1
							# PWM -----------------------------------------------
							# send duty cycle
							t1 = time.time()
							self.commandAllJointVelocities(cmd_vels)
							t2 = time.time()

							# place the sensor polling in duty pulse
							angles_prev = angles 			# save old joint values
							angles =  self.getJointAngles() # get new joint values

							t3_prev = t3
							t3 = time.time()
							t_interval = t3 - t3_prev		# time between joint querys (for velocity calc)

							# hold until all the high duty cycles are done
							while any(cmd_vels.values()):
								t4 = time.time()
								curr_duty = (t4 - t2) / period

								# pull down PWM after duty cycle elapses
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
							# END PWM -----------------------------------------------

						t_end_waypoint = time.time()
						waypoint_prev = waypoint



					# remove the trajectory from the queue
					if len(self.trajectory_queue) > 1:
						self.trajectory_queue = self.trajectory_queue.remove(self.trajectory_queue[0])
					else:
						self.trajectory_queue = []


					avg_pos_errors = {'j1':0, 'j2':0, 'j3':0, 'j4':0, 'j5':0, 'j6':0, 'g':0}
					avg_vel_errors = {'j1':0, 'j2':0, 'j3':0, 'j4':0, 'j5':0, 'j6':0, 'g':0}
					for j in self.orderedJoints:
						for p_e, v_e in zip(positions_err_dict[j], velocities_err_dict[j]):
							avg_pos_errors[j] = avg_pos_errors[j] + abs(p_e)
							avg_vel_errors[j] = avg_vel_errors[j] + abs(v_e)
						if (len(positions_err_dict[j]) != 0):
							avg_pos_errors[j] = avg_pos_errors[j] / len(positions_err_dict[j])
						if (len(velocities_err_dict[j]) != 0):
							avg_vel_errors[j] = avg_vel_errors[j] / len(velocities_err_dict[j])

					print "POSITION ERROR", avg_pos_errors, sum(avg_pos_errors.values())/len(avg_pos_errors)
					print "VELOCITIES ERROR", avg_vel_errors, sum(avg_vel_errors.values())/len(avg_vel_errors)

					msg = ExecuteTrajectoryActionResult()
					print msg
					msg.header = Header()
					msg.header.stamp = rospy.Time.now()
					msg.result.error_code.val = 1
					self.trajectory_result_pub.publish(msg)

					# plot errors
					plt.plot(positions_err_dict['j1'], 'b')
					plt.plot(positions_err_dict['j2'], 'r')
					plt.plot(positions_err_dict['j3'], 'm')
					plt.plot(positions_err_dict['j4'], 'g')
					plt.plot(positions_err_dict['j5'], 'k')
					plt.plot(positions_err_dict['j6'], 'y')
					plt.show()

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
mara_serial = mara_serial(device_num=num, left_right=lr)

# initialize the grippers and have user watch to find the device num
if mara_serial.isInitialized:
	mara_serial.initializeGripper()

mara_serial.startJointTrajectoryControlLoop()

rospy.spin()





