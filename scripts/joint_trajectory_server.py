#!/usr/bin/env python
import serial
import cv2
import sys, time, os
import rospy
from mara_utils import *
import datetime
import math 

import moveit_msgs.msg
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from moveit_msgs.msg import MoveGroupActionResult

import matplotlib.pyplot as plt
import numpy as np
# - implement a feedback loop for desired robot states using standard ros API

# NOTES
# - On startup:
# 	- gripper auto closes, need to command positive velocity to open before sending more commands 
# Before running:
# - $ sudo chmod +777 /dev/ttyUSB0
# - $ sudo modprobe usbserial vendor=0x067b product=0x2303 

class mara_serial():
	def __init__(self, left_right='lr'):
		self.device = "/dev/ttyUSB1" 	# linux device name for serial interface
		self.interface = None 			# python serial interface

		self.rate = 0.3

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
		self.trajectory_sub = rospy.Subscriber('/mara/left_arm/joint_trajectory_controller/trajectory', JointTrajectory, self.updateJointTrajectory)
		self.rviz_trajectory_sub = rospy.Subscriber('/move_group/result', MoveGroupActionResult, self.updateMoveitTrajectory)

		self.joint_state_pub = rospy.Publisher('/mara/left_arm/joint_positions', JointState)

	
		
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


		# sets virtual zero position for joints to a logical place
		#joint_angles['j3'] -= 111.297469362
		#if joint_angles['j3'] < 0:
		#	joint_angles['j3'] += 360


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
				self.joint_angles['j3'] = self.joint_angles['j3'] - self.joint_angles['j2'] 		# offset linked encoders

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
	def initializeGripper(self, left_right='l'):
		self.commandJointVelocity(joint='g', vel=10)
		# open the gripper a bit
		print "Initializing", left_right, "gripper"
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
				new_positions.append(angle + 180)
			p1.positions = new_positions
			print p1.positions

		new_trajectory = trajectory
		return new_trajectory




	# update the trajectory queue with new goals from ROS sub
	def updateJointTrajectory(self, msg):
		print 'Received new joint trajectory'
		self.trajectory_queue.append(msg)
		self.trajectory_queue = self.trajectory_queue[-self.trajectory_queue_len:]

	def updateMoveitTrajectory(self, msg):
		print 'Received new RVIZ joint trajectory'
		trajectory = msg.result.planned_trajectory.joint_trajectory
		short_names = {'joint1':'j1', 'joint2':'j2', 'joint3':'j3', 'joint4':'j4', 'joint5':'j5', 'joint6':'j6', 'gripper':'g'}
		new_names = []
		for n in trajectory.joint_names:
			new_names.append(short_names[n])
		trajectory.joint_names = new_names

		trajectory = self.enforceMaxTrajectoryVelocity(trajectory)

		self.trajectory_queue.append(trajectory)
		self.trajectory_queue = self.trajectory_queue[-self.trajectory_queue_len:]


	# publish the robot joint states
	def updateJointPositions(self, angles):
		msg = JointState()
		msg.header = Header()
		msg.header.stamp = rospy.Time.now()

		full_names = {'j1':'joint1', 'j2':'joint2', 'j3':'joint3', 'j4':'joint4', 'j5':'joint5', 'j6':'joint6', 'g':'gripper'}
		names = []
		positions = []
		for name, theta in angles.iteritems():
			names.append(full_names[name])
			#positions.append(0)
			positions.append((theta - 180.0) * (math.pi/180.0))

		msg.name = names
		msg.position = positions

		self.joint_state_pub.publish(msg)


	# main control loop
	# - obtain joint positions and publish to /mara/<arm>/joint_positions/<joint_name>
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

					print "Executing joint trajectory..."
					curr_trajectory = self.trajectory_queue[0]
					t_start = time.time()
					angles_prev = angles


					positions_err_dict = {'j1':[], 'j2':[], 'j3':[], 'j4':[], 'j5':[], 'j6':[], 'g':[]}
					velocities_err_dict = {'j1':[], 'j2':[], 'j3':[], 'j4':[], 'j5':[], 'j6':[], 'g':[]}

					# iterate through waypoints in the trajectory
					for waypoint in curr_trajectory.points:
						t_subdiv = time.time() 			# subdivision of waypoint period at interval == PWM period
						t1 = time.time() 				# starting time of PWM period
						t3 = time.time()
						t_waypoint = time.time() 		# time waypoint was first seen
						t_interval = period 			# initial velocity calcualtion based on ideal interval
						initial_command = True 			# dont the period for initial command

						waypoint_angles = angles 		# joint values when waypoint was started
						print "WAYPOINT-----------------------------------------------------------"
						print "check interval = ", t_interval
						print 'check time = ', t_subdiv - t_start
						print 'current waypoint duration', waypoint.time_from_start.to_sec()

						# if the waypoint goal time has not elapsed
						while t_subdiv < t_start + waypoint.time_from_start.to_sec():
							print "waypoint check...\t \t", (t_start + waypoint.time_from_start.to_sec()) - t_subdiv
							measured_vels = {'j1':0, 'j2':0, 'j3':0, 'j4':0, 'j5':0, 'j6':0, 'g':0}
							zero_vels = {'j1':0, 'j2':0, 'j3':0, 'j4':0, 'j5':0, 'j6':0, 'g':0} 	# zero velocity commands to joints
							cmd_vels = {'j1':0, 'j2':0, 'j3':0, 'j4':0, 'j5':0, 'j6':0, 'g':0} 		# high velocity commands to joints
							duty_ratios = {'j1':0.0, 'j2':0.0, 'j3':0.0, 'j4':0.0, 'j5':0.0, 'j6':0.0, 'g':0.0}

							# calculate goal pos and vel
							goal_angles, goal_vels = {}, {}
							for name, des in zip(curr_trajectory.joint_names, waypoint.positions):

								# calculate desired velocity accounting for wraparound
								if (des - waypoint_angles[name]) < -180:
									goal_vels[name] = (des - waypoint_angles[name] + 360) / ((t_start + waypoint.time_from_start.to_sec()) - t_waypoint)
								elif (des - waypoint_angles[name]) > 180:
									goal_vels[name] = (des - waypoint_angles[name] - 360) / ((t_start + waypoint.time_from_start.to_sec()) - t_waypoint)
								else:
									goal_vels[name] = (des - waypoint_angles[name]) / ((t_start + waypoint.time_from_start.to_sec()) - t_waypoint)
								goal_angles[name] = des

							print "Current angles", angles
							print "Goal angles", goal_angles
							print "Goal vels", goal_vels
							print "Starting pos", waypoint_angles
							print "time", ((t_start + waypoint.time_from_start.to_sec()) - t_waypoint)

							# for each joint calculate PD velocity commands
							for joint_name in curr_trajectory.joint_names:
								# check if were moving clockwise or counterclockwise
								vel = goal_vels[joint_name]
								
								# record a window of joint velocities
								window_size = 1

								# calculate velocity accounting for wraparound in the positive and negative direction
								if angles[joint_name] < 180 and angles_prev[joint_name] > 180 and goal_vels[joint_name] > 0:
									delta_theta = angles[joint_name] - angles_prev[joint_name] + 360
								elif angles[joint_name] > 180 and angles_prev[joint_name] < 180 and goal_vels[joint_name] < 0:
									delta_theta = angles[joint_name] - angles_prev[joint_name] - 360
								else:
									delta_theta = angles[joint_name] - angles_prev[joint_name]

								# record the instantaneous velocity ina  sliding window
								velocity_windows[joint_name].append(delta_theta/t_interval)
								if len(velocity_windows) > window_size:
									velocity_windows[joint_name] = velocity_windows[joint_name][-window_size:]
								
								err_theta = goal_angles[joint_name] - angles[joint_name]
								#measured_vels[joint_name] = (angles[joint_name] - angles_prev[joint_name])/period 					# instantaneous measured vel
								measured_vels[joint_name] = sum(velocity_windows[joint_name])/len(velocity_windows[joint_name]) 	# sliding window measured vel
								err_vel = goal_vels[joint_name] - measured_vels[joint_name]

								positions_err_dict[joint_name].append(angles[joint_name])
								velocities_err_dict[joint_name].append(measured_vels[joint_name])
								#print joint_name, 'angle', angles[joint_name], 'delta', delta_theta, 'goal', goal_vels[joint_name], 'actual', measured_vels[joint_name]

								kp = 0.2
								kd = 0.2
								pid_vel = vel
								#pid_vel = vel * min((kp * abs(err_theta)), 1.0) + (kd * err_vel)

								print 'cmd', vel, 'err vel', err_vel, 'err pos', err_theta, 'pid', pid_vel 

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

						for name, des in zip(curr_trajectory.joint_names, waypoint.positions):
							delta = angles[name] - des
							if delta < -180:
								delta += 360
							elif delta > 180:
								delta -= 360
							print name, "Waypoint position error", delta
						
						for name in curr_trajectory.joint_names:
							delta = (angles[name] - waypoint_angles[name])
							if delta < -180:
								delta += 360
							elif delta > 180:
								delta -= 360
							print "Observed waypoint velocity",  delta / (t_end_waypoint - t_waypoint)


					# remove the trajectory from the queue
					self.trajectory_queue = self.trajectory_queue[1:]
					plt.plot(positions_err_dict['j1'], 'b')
					plt.plot(positions_err_dict['j2'], 'r')
					plt.plot(positions_err_dict['j3'], 'm')
					plt.plot(positions_err_dict['j4'], 'g')
					plt.plot(positions_err_dict['j5'], 'k')
					plt.plot(positions_err_dict['j6'], 'y')
					plt.show()

				# TODO: make this sleep the ROS rate
				else:

					rospy.sleep(self.rate)




			else:
				pass










							




			



rospy.init_node('joint_trajectory_controller')

mara_serial = mara_serial()
mara_serial.initializeGripper()
#mara_serial.unfoldArm(left_right=left_right)
#mara_serial.calibrateController()
#mara_serial.testVelocity( 'j1', 30, left_right=left_right)
#print mara_serial.getJointAngles()

#desired_angles = {'g': 40, 'j4': 180, 'j5': 90, 'j6': 120, 'j1': 40, 'j2': 140, 'j3': 270}
#vel_commands = {'j1':0, 'j2':0, 'j3':5, 'j4':0, 'j5':0, 'j6':0, 'g':0}
#mara_serial.commandAllJointAngles(desired_angles, vel_commands, left_right, )

mara_serial.startJointTrajectoryControlLoop()

#mara_serial.commandJointAngle('g', 60, 20, left_right='l')
#mara_serial.testVelocity(30)
#mara_serial.foldArm(left_right=left_right)
#mara_serial.zeroArm(left_right=left_right)

# how fast to query joint angles via usb
rospy.spin()
# joint velocities in degrees per second
#vel = {'j1':1, 'j2':0, 'j3':1, 'j4':0, 'j5':1, 'j6':0, 'g':1}
#mara_serial.commandJointVelocity




