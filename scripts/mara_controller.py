#!/usr/bin/env python

import rospy
from camera_publisher import camera_publisher
import mara_moveit
import threading
import copy
import tf
from geometry_msgs.msg import PoseStamped

import os, sys, time

import mara_positions
from mara_utils import *
import OPEAssist
from std_msgs.msg import Int32, Header
from actionlib_msgs.msg import GoalStatusArray, GoalStatus, GoalID
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
    FollowJointTrajectoryActionFeedback,
    FollowJointTrajectoryActionResult
)
# Steps to launch:
#
# 1: run the trajectory servers for each arm:
# 	$ rosrun mara joint_trajectory_server.py r 0
# 	$ rosrun mara joint_trajectory_server.py l 1
#
# 2: launch moveit:
#	$ roslaunch mara_moveit_config demo_mara.launch
#
# 3: run the controller
#	$ rosrun mara mara_controller.py   


# To kill all MARA processes: $ sudo kill -9 `ps -ef | grep mara | awk '{ print $2 }'`



OPENNI_CMD = "roslaunch openni_launch openni.launch"
OPE_DIR = "/home/carrt/Dropbox/catkin_ws/src/mara/OPE-Release"
KILL_XNSENSOR_CMD = "killall killXnSensorServer"

class mara_controller():
	def __init__(self):
		self.moveit = mara_moveit.MaraMoveIt()
		self.tf_listen = tf.TransformListener()

		self.rightGripperAction_pub = rospy.Publisher("/mara/limb/right/gripper_action/position", Int32, queue_size=1)
		self.leftGripperAction_pub = rospy.Publisher("/mara/limb/left/gripper_action/position", Int32, queue_size=1)

		self.leftGripperStatus_sub = rospy.Subscriber('/mara/limb/left/gripper_action/result', GoalStatus, self.updateLeftGripperStatus, queue_size=2)
		self.rightGripperStatus_sub = rospy.Subscriber('/mara/limb/right/gripper_action/result', GoalStatus, self.updateRightGripperStatus, queue_size=2)

		self.leftArmStatus_sub = rospy.Subscriber('/mara/limb/left/follow_joint_trajectory/result', FollowJointTrajectoryActionResult, self.updateLeftArmStatus, queue_size=2)
		self.rightArmStatus_sub = rospy.Subscriber('/mara/limb/right/follow_joint_trajectory/result', FollowJointTrajectoryActionResult, self.updateRightArmStatus, queue_size=2)

		self.right_gripper_result = -1
		self.left_gripper_result = -1

		self.right_arm_result = -1
		self.left_arm_result = -1

	def updateLeftGripperStatus(self, msg):
		print "Received terminal result for GRIPPER", msg.status
		self.left_gripper_result = int(msg.status)
	def updateRightGripperStatus(self, msg):
		print "Received terminal result for GRIPPER", msg.status
		self.right_gripper_result = int(msg.status)

	def updateLeftArmStatus(self, msg):
		print "Received terminal result for ARM", msg.result.error_code
		self.left_arm_result = int(msg.result.error_code)
	def updateRightArmStatus(self, msg):
		print "Received terminal result for ARM", msg.result.error_code
		self.right_arm_result = int(msg.result.error_code)


	def waitOnArmSuccess(self, lr):
		msg = FollowJointTrajectoryActionResult()
		if lr == 'left' or lr == 'l':
			while(self.left_arm_result != msg.result.SUCCESSFUL):
				rospy.sleep(0.1)
			self.left_arm_result = -1
		if lr == "right" or lr == 'r':
			while(self.right_arm_result != msg.result.SUCCESSFUL):
				rospy.sleep(0.1)
			self.right_arm_result = -1

	def waitOnGripperSuccess(self, lr):
		msg = GoalStatus()
		if lr == 'left' or lr == 'l':
			while(self.left_gripper_result != msg.SUCCEEDED):
				rospy.sleep(0.1)
			self.left_gripper_result = -1
		if lr == "right" or lr == 'r':
			while(self.right_gripper_result != msg.SUCCEEDED):
				rospy.sleep(0.1)
			self.right_gripper_result = -1


	def commandGripperPosition(self, lr, val):
		if lr == 'left':
			print "commanding left gripper"
			self.leftGripperAction_pub.publish(int(val))
			print "waiting for gripper..."
			self.waitOnGripperSuccess('left')
		elif lr == 'right':
			print "commanding right gripper"
			self.rightGripperAction_pub.publish(int(val))
			print "waiting for gripper..."
			self.waitOnGripperSuccess('right')
			
	#------------------------------------------------------------------------------------------#
	# Arm motion
	#
	# Known issues:
	# 	- planning a path plan actually executes it as well... (should be a moveit issue)
	# 	  lines relating to execution have been commented out
	#------------------------------------------------------------------------------------------#
	def goToWaiting(self, arm):
		if arm == 0 or arm == "left":
			rospy.loginfo("Going to waiting position (left)")
			waitingPlan = self.moveit.createPathPlan(mara_positions.leftWaitingPos, \
														0, gripperorientation=mara_positions.leftWaitingRot)
			"""
			if self.moveit.leftGroup.execute(waitingPlan):
				#self.leftLimbWaitingPos = self.leftLimb.joint_angles()
				return True
			else:
				print "Could not move to waiting position!"
				return False
			"""
			print "Waiting on execution"
			self.waitOnArmSuccess('left')

		elif arm == 1 or arm == "right":
			rospy.loginfo("Going to waiting position (right)")
			waitingPlan = self.moveit.createPathPlan(mara_positions.rightWaitingPos, \
													1, gripperorientation=mara_positions.rightWaitingRot)
			"""
			if self.moveit.rightGroup.execute(waitingPlan):
				#self.rightLimbWaitingPos = self.rightLimb.joint_angles()
				return True
			else:
				print "Could not move to waiting position!"
				return False
			"""
			print "Waiting on execution"
			self.waitOnArmSuccess('right')

	# creates a path plan to the specified (x,y,z) coordinates
	# TODO: Fix the issue where PLANs actually EXECUTE (execution code is commented)
	def moveArmToPosition(self, arm, pos, orient):
		if arm == 0 or arm == "left":
			rospy.loginfo("Moving Left arm")
			plan = self.moveit.createPathPlan(pos, 0, gripperorientation=orient)
			"""
			if self.moveit.leftGroup.execute(plan):
				#self.leftLimbWaitingPos = self.leftLimb.joint_angles()
				return True
			else:
				print "Could not move to waiting position!"
				return False
			"""
			print "Waiting on execution"
			self.waitOnArmSuccess('left')

		elif arm == 1 or arm == "right":
			rospy.loginfo("Moving right arm")
			plan = self.moveit.createPathPlan(pos, 1, gripperorientation=orient)
			"""
			if self.moveit.rightGroup.execute(plan):
				#self.rightLimbWaitingPos = self.rightLimb.joint_angles()
				return True
			else:
				print "Could not move to waiting position!"
				return False
			"""
			print "Waiting on execution"
			self.waitOnArmSuccess('right')

	# transforms a position (x,y,z) and orientation (r,p,y) to a new frame that is in the tf tree
	def transformPoseToFrame(self, pos, rot, frame):
		msg = PoseStamped()
		msg.header = Header()
		msg.header.stamp = rospy.Time.now() 
		msg.header.frame_id = 'camera_link'

		quaternion = tf.transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
		msg.pose.position.x = pos[0]
		msg.pose.position.y = pos[1]
		msg.pose.position.z = pos[2]
		msg.pose.orientation.x = quaternion[0]
		msg.pose.orientation.y = quaternion[1]
		msg.pose.orientation.z = quaternion[2]
		msg.pose.orientation.w = quaternion[3]

		new_pose = self.tf_listen.transformPose(frame, msg)
		return new_pose.pose.position, new_pose.pose.orientation

	# removes all collision models from the planning environment
	def removeOPECollisionModels(self):
		for i in range(100):
			self.moveit.scene.remove_world_object("OBJECT" + str(i))
		self.moveit.scene.remove_world_object("TABLE")

	# runs OPE and saves obejct/table position then adds them to the planning environment
	def getObjectPositions(self):
		# remove old point cloud files
		rospy.loginfo("Removing old point clouds and collision models")
		removePCDs(OPE_DIR)
		self.removeOPECollisionModels()

		# run OPE
		rospy.loginfo("Running OPE")
		OPEAssist.runOPE()
		OPEAssist.loadOPEResults()

		objectLoc = None
		if OPEAssist.objCount > 0:

			rospy.loginfo("Adding Table Collision Model")
			pos, q = self.transformPoseToFrame(OPEAssist.tablePos, [0,0,0], 'base_link')
			OPEAssist.tablePos = [pos.x, pos.y, pos.z]

			#ADD Table Collision Model
			self.moveit.addObject("TABLE",
								OPEAssist.tablePos,
								OPEAssist.tableSize)

			rospy.loginfo("Adding Object Collision Models")
			# ADD Object Collision Models
			for x in OPEAssist.objList:
				pos, q = self.transformPoseToFrame(x['objPos'], x['objRot'], 'base_link')
				x['objPos'] = [pos.x, pos.y, pos.z]
				x['objRot'] = [q.x, q.y, q.z, q.w]

				self.moveit.addObject("OBJECT" + str(x['objNumber']),
									x['objPos'],
									x['objSize'],
									x['objRot'])
		else:
			rospy.loginfo("No Objects Detected")
			return 


	# Object Grasping
	# - Assume objects are in view of robot on a table
	# - Runs OPE and grasps  object with color of parameter 'color'
	def command_grasp_object(self, color=None):
		rospy.loginfo("Grasp Object Starting")

		rospy.loginfo("Moving right gripper")
		self.commandGripperPosition(lr='right', val=80)

		self.objectNum = 1
		objectPos = OPEAssist.objList[self.objectNum]['objPos']
		preObjectPos = copy.deepcopy(OPEAssist.objList[self.objectNum]['objPos'])
		objectPos[1] -= 0.1 			# make the object position just in front of the centroid
		preObjectPos[1] -= 0.28			# amke the pre-object position just in front of the object


		rospy.loginfo("Moving to pre object")
		self.moveArmToPosition( "right", preObjectPos, mara_positions.rightWaitingRot )

		self.moveit.scene.remove_world_object("OBJECT" + str(self.objectNum))
		rospy.loginfo("Moving to object")
		self.moveArmToPosition( "right", objectPos, mara_positions.rightWaitingRot )

		rospy.loginfo("Moving gripper")
		self.commandGripperPosition(lr='right', val=40)

		rospy.loginfo("Moving to pre object")
		self.moveArmToPosition("right", preObjectPos, mara_positions.rightWaitingRot )

		rospy.loginfo("Moving to waiting")
		self.goToWaiting("right")






success = rospy.init_node('mara_controller')
mara = mara_controller()
# reset arms
rospy.loginfo("Moving right arm to waiting position")
mara.goToWaiting("right")
mara.getObjectPositions()
mara.command_grasp_object()

print "Done!"
rospy.spin()
print "Exiting"