#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from mara.cfg import mara_paramsConfig
from std_msgs.msg import Int32MultiArray, Int32, Bool, String

class dashboard():

	def __init__(self):

		# nested dictionary of camera parameters adjustable through dynreconf gui
		self.cam_params = ['enable', 'refresh_rate', 'device_id', 'img_res']
		params1 = {'enable':False, 'refresh_rate':30, 'device_id':0, 'img_res':"1280x720"}
		params2 = {'enable':False, 'refresh_rate':30, 'device_id':1, 'img_res':"1280x720"}
		self.cameras = {'camera_front_1':params1, 'camera_front_2':params2}


		# ROS messaging
		self.camera_front_1_enable_pub = rospy.Publisher('camera_front_1/enable', Bool, queue_size=1)
		self.camera_front_2_enable_pub = rospy.Publisher('camera_front_2/enable', Bool, queue_size=1)

		self.camera_front_1_refresh_rate_pub = rospy.Publisher('camera_front_1/refresh_rate', Int32, queue_size=1)
		self.camera_front_2_refresh_rate_pub = rospy.Publisher('camera_front_2/refresh_rate', Int32, queue_size=1)

		self.camera_front_1_device_id_pub = rospy.Publisher('camera_front_1/device_id', Int32, queue_size=1)
		self.camera_front_2_device_id_pub = rospy.Publisher('camera_front_2/device_id', Int32, queue_size=1)

		self.camera_front_1_img_res_pub = rospy.Publisher('camera_front_1/img_res', String, queue_size=1)
		self.camera_front_2_img_res_pub = rospy.Publisher('camera_front_2/img_res', String, queue_size=1)

		publishers1 = {'enable':self.camera_front_1_enable_pub, \
					   'refresh_rate':self.camera_front_1_refresh_rate_pub, \
					   'device_id':self.camera_front_1_device_id_pub, \
					   'img_res':self.camera_front_1_img_res_pub, \
					   }
		publishers2 = {'enable':self.camera_front_2_enable_pub, \
					   'refresh_rate':self.camera_front_2_refresh_rate_pub, \
					   'device_id':self.camera_front_2_device_id_pub, \
					   'img_res':self.camera_front_2_img_res_pub, \
					   }
		self.camera_publishers = {'camera_front_1':publishers1, 'camera_front_2':publishers2}

		# setup the callback for the reconfigure server
		self.server = DynamicReconfigureServer(mara_paramsConfig, self.reconfigure)



	def reconfigure(self, config, level):
		# grab new values from GUI

		# check configurable params for each camera
		for p in self.cam_params:
			for c in self.cameras:
				if config[c+'_'+p] != self.cameras[c][p]:
					self.cameras[c][p] = config[c+'_'+p]		# set the new value
					self.camera_publishers[c][p].publish(config[c+'_'+p])

		return config



if __name__ == "__main__":
	rospy.init_node('mara_dashboard', anonymous=True)
	db = dashboard()
	rospy.spin()


