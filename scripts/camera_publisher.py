#!/usr/bin/env python

import rospy
import cv2

import sys, time
from std_msgs.msg import Int32MultiArray, Int32, Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

MAX_RATE = 60 	# maximum camera refresh rate

class camera_publisher():
	def __init__(self, name='', w=1920, h=1080, rate=30, device=0 ):

		self.w = w 					# camera resolution (w)
		self.h = h					# camera resolution (h)
		self.rate = rate 			# refresh rate

		self.device = device 		# device ID
		self.enable = True

		self.name = name

		# setup the ROS publisher for images
		rate = rospy.Rate(self.rate)
		
		self.stream = cv2.VideoCapture(self.device)
		self.bridge = CvBridge()
		self.image = None 			# most recent 

		if not self.stream.isOpened():
			rospy.loginfo("No camera found for device ID: "+ str(self.device))

		self.rgb_pub = rospy.Publisher(self.name+'/image_rgb', Image, queue_size=10)

		self.enable_sub = rospy.Subscriber(self.name+'/enable', Bool, self.enableCb)
		self.rate_sub = rospy.Subscriber(self.name+'/refresh_rate', Int32, self.rateCb)
		self.id_sub = rospy.Subscriber(self.name+'/device_id', Int32, self.deviceCb)
		self.resolution_sub = rospy.Subscriber(self.name+'/img_res', String, self.resolutionCb)

		while not rospy.is_shutdown():
			if self.enable:
				if self.stream.isOpened():
					success, self.image = self.stream.read()
					if success:
						img_msg = self.bridge.cv2_to_imgmsg(self.image, "bgr8")
						self.rgb_pub.publish(img_msg)
						cv2.imshow(self.name, self.image)
						cv2.waitKey(10)



	# start publishing
	def enableCb(self, msg):
		enable = msg.data

		if self.enable != enable:
			self.enable = enable

			if self.enable:
				rospy.loginfo('Camera is on with device ID: ' + str(self.device))
			else:
				rospy.loginfo('Camera is off with device ID: ' + str(self.device))

	def resolutionCb(self, msg):
		data = msg.data

		if 'x' in data:
			data = data.split('x')
		elif ' ' in data:
			data = data.split(' ')

		if len(data) == 2:
			if data[0] < 0:
				pass
			else:
				self.w = int(data[0])

			if data[1] < 0:
				pass
			else:
				self.h = int(data[1])

			if self.w != self.stream.get(cv2.CAP_PROP_FRAME_WIDTH):
				rospy.loginfo("Setting resolution width to: "+str(self.w))
				self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
			if self.h != self.stream.get(cv2.CAP_PROP_FRAME_HEIGHT):
				rospy.loginfo("Setting resolution height to: "+str(self.h))
				self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, self.h)
		else:
			rospy.loginfo("Invalid resolution format: Use the form <width>x<height> or <width> <height>")
		

	def rateCb(self, msg):
		data = msg.data
		if data < 0 or data > MAX_RATE:
			self.rate = 0
		else:
			self.rate = data

			if self.rate != self.stream.get(cv2.CAP_PROP_FPS):
				rospy.loginfo("Setting rate to: "+str(self.rate))
				self.stream.set(cv2.CAP_PROP_FPS, self.rate)

	def deviceCb(self, msg):
		data = msg.data
		if self.device != data:
			if self.enable:
				self.enable = False
			self.stream.open(data)
			self.device = data

			if self.stream.isOpened():
				rospy.loginfo('Camera is on with device ID: ' + str(self.device))
			else:
				rospy.loginfo('Camera is off with device ID: ' + str(self.device))


if __name__ == "__main__":

	name = 'camera'
	device = 0
	if len(sys.argv) >= 2:
		name = str(sys.argv[1])
	if len(sys.argv) >= 3:
		device = int(sys.argv[2])	

	rospy.init_node('mara_dashboard', anonymous=True)		
	camera_front_1 = camera_publisher(name=name, device=device)
	rospy.spin()

		





