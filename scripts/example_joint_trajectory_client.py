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

import moveit_commander
import moveit_msgs.msg

"""
# BELOW: waits for joint position feedback on ROS topic, executes a static trajectory from that position
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
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4]-10, positions[5], positions[6]]
p.time_from_start = rospy.Duration(2)
pl.append(p)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4]-20, positions[5], positions[6]]
p.time_from_start = rospy.Duration(3)
pl.append(p)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4]-20, positions[5], positions[6]]
p.time_from_start = rospy.Duration(6)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4]-30, positions[5], positions[6]]
p.time_from_start = rospy.Duration(7)
pl.append(p)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4]-40, positions[5], positions[6]]
p.time_from_start = rospy.Duration(8)
pl.append(p)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4]-40, positions[5], positions[6]]
p.time_from_start = rospy.Duration(11)
pl.append(p)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4]-50, positions[5], positions[6]]
p.time_from_start = rospy.Duration(12)
pl.append(p)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4]-60, positions[5], positions[6]]
p.time_from_start = rospy.Duration(13)
pl.append(p)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4]-60, positions[5], positions[6]]
p.time_from_start = rospy.Duration(15)
pl.append(p)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4]-70, positions[5], positions[6]]
p.time_from_start = rospy.Duration(16)
pl.append(p)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4]-80, positions[5], positions[6]]
p.time_from_start = rospy.Duration(17)
pl.append(p)
p = JointTrajectoryPoint()
p.positions = [positions[0], positions[1], positions[2], positions[3], positions[4]-80, positions[5], positions[6]]
p.time_from_start = rospy.Duration(20)
 

for p in pl:
	for i in range(len(p.positions)):
		p.positions[i] = p.positions[i]%360

msg.points = pl

print msg.points
print msg.joint_names

#joint_trajectory_pub.publish(msg)

joint_trajectory_pub.publish(msg)
"""



robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("left_arm")
display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)
# wait for rviz to launch
rospy.sleep(10)


# create the path
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1.0
pose_target.position.x = -0.3
pose_target.position.y = -0.4
pose_target.position.z = 0.6
group.set_pose_target(pose_target)

plan = group.plan()
joint_trajectory_pub.publish(plan)
print plan

