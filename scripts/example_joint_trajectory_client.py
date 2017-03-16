#!/usr/bin/env python
import serial
import cv2
import sys, time, os
import rospy
from mara_utils import *
import datetime

import moveit_msgs.msg
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


positions = None
names = None
flag = False

def updateJointState(msg):
	global positions, current_pos, flag, names
	pos = {}
	positions = msg.position
	names = msg.name
 	flag = True

rospy.init_node('joint_trajectory_client')
joint_trajectory_pub = rospy.Publisher('/mara/left_arm/joint_trajectory_controller/trajectory', JointTrajectory, queue_size=1)
joint_pos_sub = rospy.Subscriber('/mara/left_arm/joint_positions', JointState, updateJointState)




while not flag:
	rospy.sleep(1.0)
	print "waiting for current positions..."





msg = JointTrajectory()
msg.header = Header()
msg.joint_names = names


pl = []

p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4], positions[5], positions[6]]
p.time_from_start = rospy.Duration(1)
pl.append(p)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4]-10, positions[5], positions[6]-20]
p.time_from_start = rospy.Duration(2)
pl.append(p)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4]-20, positions[5], positions[6]-40]
p.time_from_start = rospy.Duration(3)
pl.append(p)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4]-20, positions[5], positions[6]-60]
p.time_from_start = rospy.Duration(4)
pl.append(p)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4]-20, positions[5], positions[6]-40]
p.time_from_start = rospy.Duration(5)
pl.append(p)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4]-20, positions[5], positions[6]-20]
p.time_from_start = rospy.Duration(6)
pl.append(p)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4]-20, positions[5], positions[6]]
p.time_from_start = rospy.Duration(7)
pl.append(p)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4]-20, positions[5], positions[6]+10]
p.time_from_start = rospy.Duration(8)
pl.append(p)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4]-20, positions[5], positions[6]+20]
p.time_from_start = rospy.Duration(9)
pl.append(p)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4]-20, positions[5], positions[6]+30]
p.time_from_start = rospy.Duration(10)
pl.append(p)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4]-10, positions[5], positions[6]+20]
p.time_from_start = rospy.Duration(11)
pl.append(p)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4], positions[5], positions[6]+10]
p.time_from_start = rospy.Duration(12)
pl.append(p)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4], positions[5], positions[6]]
p.time_from_start = rospy.Duration(13)
pl.append(p)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4], positions[5], positions[6]]
p.time_from_start = rospy.Duration(14)
pl.append(p)


for p in pl:
	for i in range(len(p.positions)):
		p.positions[i] = p.positions[i]%360





msg.points = pl

print msg.points
print msg.joint_names

#joint_trajectory_pub.publish(msg)

joint_trajectory_pub.publish(msg)