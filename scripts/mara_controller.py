#!/usr/bin/env python

import rospy
from camera_publisher import camera_publisher
import mara_moveit
import threading

import os, sys

import mara_positions
from mara_utils import *
import OPEAssist

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



OPENNI_CMD = "roslaunch openni_launch openni.launch"
OPE_DIR = "/home/carrt/Dropbox/catkin_ws/src/mara/OPE-MARA"
KILL_XNSENSOR_CMD = "killall killXnSensorServer"

class mara_controller():
	def __init__(self):
		self.moveit = mara_moveit.MaraMoveIt()


	def goToWaiting(self, arm):
		if arm == 0 or arm == "left":
			rospy.loginfo("Going to waiting position (left)")
			waitingPlan = self.moveit.createPathPlan(mara_positions.leftWaitingPos, 0, gripperorientation=mara_positions.leftWaitingRot)
			if self.moveit.leftGroup.execute(waitingPlan):
				#self.leftLimbWaitingPos = self.leftLimb.joint_angles()
				return True
			else:
				print "Could not move to waiting position!"
				return False

		elif arm == 1 or arm == "right":
			rospy.loginfo("Going to waiting position (right)")
			waitingPlan = self.moveit.createPathPlan(mara_positions.rightWaitingPos, 1, gripperorientation=mara_positions.rightWaitingRot)
			if self.moveit.rightGroup.execute(waitingPlan):
				#self.rightLimbWaitingPos = self.rightLimb.joint_angles()
				return True
			else:
				print "Could not move to waiting position!"
				return False

	# Object Grasping
	# - Assume objects are in view of robot on a table
	# - Runs OPE and grasps  object with color of parameter 'color'
	def command_grasp_object(self, color=None):
		rospy.loginfo("Retreive Grasp Starting")

		# remove any remaining collision models of OPE objects
		for i in range(50):
			self.moveit.scene.remove_world_object("OBJECT" + str(i))
		self.moveit.scene.remove_world_object("TABLE")

		# reset arms
		self.goToWaiting(0)
		#rospy.sleep(1)

		#self.goToWaiting(1)
		#rospy.sleep(1)

		#Remove Old PCDs
		#removePCDs(OPE_DIR)
		#rospy.sleep(1)

		#if self.gotoWaiting():
		#*********************************************************************************#
		# Object Pose Estimation Selection (WORKING)
		# - Estimate Object Poses 
		# - Save OPE Results to txt
		# - Average Hues and save to color file

		'''
		# Run OPE
		ret = -1
		while ret != 0:
			print "Attempting to run OPE"
			ret = OPEAssist.runOPE()
			rospy.sleep(1)

		OPEAssist.loadOPEResults()
		#rospy.sleep(1)

		# Get OPE Object Colors
		#colors_process = subprocess.Popen(COLORS_CMD, shell=True, preexec_fn=os.setsid)
		#rospy.sleep(1)

		# Colored Object Selection
		#desired_color = color
		desired_color = "red"
		objectNum = 0
		arm = 0

		"""
		# Color recognition
		colorFile = open(OPE_DIR + "ObjectColors.txt")
		content = colorFile.readlines()
		print content

		i = 0
		for colors in content:
			objColor = colors.strip("'").rstrip()

			print "checking: " + objColor
			if objColor == desired_color:
				objectNum = i
				break
			i = i + 1
		"""

		print "Object Number: " , objectNum

		# Show OPE Results, Grab the Object
		objectLoc = None
		if OPEAssist.objCount > 0:

			rospy.loginfo("Adding Table Collision Model")
			#ADD Table Collision Model
			self.moveit.addObject("TABLE",
									OPEAssist.tablePos,
									OPEAssist.tableSize)

			# ADD Object Collision Models
			for x in OPEAssist.objList:
				self.moveit.addObject("OBJECT" + str(x['objNumber']),
										x['objPos'],
										x['objSize'])

			#rospy.sleep(1)
			#OPEAssist.showOPEResults()

			objectLoc = OPEAssist.objList[objectNum]['objPos']
			print "OBJECT ", objectNum


			print "Removing collisions models"
            for i in range(OPEAssist.objCount):
                self.moveit.scene.remove_world_object("OBJECT" + str(i))
            self.moveit.scene.remove_world_object("TABLE")
            

		#*********************************************************************************#
		else:
			rospy.loginfo("No Objects Detected")
			return 

	'''





success = rospy.init_node('mara_controller')
mara = mara_controller()
mara.command_grasp_object()

print "Done!"
rospy.spin()
print "Exiting"