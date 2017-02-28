#!/usr/bin/env python

import rospy
import cv2
from matplotlib import pyplot as plt

import sys, time
from std_msgs.msg import Int32MultiArray, Int32, Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class stereo():
	def __init__(self):
		self.img_right = None
		self.img_left = None	

		self.stereo = cv2.StereoSGBM_create(minDisparity=1, numDisparities=128, blockSize=15, mode=True)
		self.bridge = CvBridge()

		self.img_left_sub = rospy.Subscriber('camera_front_1/image_rgb', Image, self.imageLeftCb)
		self.img_right_sub = rospy.Subscriber('camera_front_2/image_rgb', Image, self.imageRightCb)

	def imageLeftCb(self, msg):
		l = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		self.img_left = cv2.cvtColor(l, cv2.COLOR_BGR2GRAY)
		#cv2.imshow('left', self.img_left)


		#cv2.waitKey(20)

	def imageRightCb(self, msg):
		r = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		self.img_right = cv2.cvtColor(r, cv2.COLOR_BGR2GRAY)
		#cv2.imshow('right', self.img_right)
		#cv2.waitKey(20)

	def disarityCalculate(self):
		while not rospy.is_shutdown():
			if self.img_left is not None and self.img_right is not None:
				disp = self.stereo.compute(self.img_left, self.img_right)
				disp_norm = disp.copy()
				disp_norm *= 2
				#cv2.normalize(disp, disp_norm, 0, 255, cv2.NORM_MINMAX)
				#disp_norm = cv2.equalizeHist(disp_norm)
				cv2.imshow('disparity', disp_norm)
				cv2.waitKey(30)
			


if __name__ == "__main__":


	rospy.init_node('mara_stereo', anonymous=True)		
	stereo = stereo()
	stereo.disarityCalculate()
	rospy.spin()

		