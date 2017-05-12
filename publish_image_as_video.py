#! /usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Copyright (c) 2015 PAL Robotics SL.
Released under the BSD License.
Created on 7/14/15
@author: Sammy Pfeiffer
test_video_resource.py contains
a testing code to see if opencv can open a video stream
useful to debug if video_stream does not work
"""

import cv2
import time
from sensor_msgs.msg import Image
import yaml
from sensor_msgs.msg import CameraInfo
import rospy
from cv_bridge import CvBridge, CvBridgeError
import argparse
from std_msgs.msg import Bool

# initialize values

enable_camera = False
grab_frame = False

rospy.init_node('publish_image_as_video')

bridge = CvBridge()

image_pub = rospy.Publisher("/hd_cam/image_raw",Image, queue_size=10)

info_pub = rospy.Publisher("/hd_cam/camera_info", CameraInfo, queue_size=10)

rate = rospy.Rate(10)

def check_enable_camera():
	global enable_camera
	value = rospy.get_param("/enable_camera", False)
	if value and not enable_camera:
		rospy.set_param("/grab_frame", True)
	enable_camera = value

def check_grab_frame():
	global grab_frame
	grab_frame = rospy.get_param("/grab_frame", False)

def yaml_to_CameraInfo(yaml_fname):

	# Load data from file
	with open(yaml_fname, "r") as file_handle:
		calib_data = yaml.load(file_handle)
	# Parse
	camera_info_msg = CameraInfo()
	camera_info_msg.header.frame_id = calib_data["camera_name"]
	camera_info_msg.width = calib_data["image_width"]
	camera_info_msg.height = calib_data["image_height"]
	camera_info_msg.K = calib_data["camera_matrix"]["data"]
	camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
	camera_info_msg.R = calib_data["rectification_matrix"]["data"]
	camera_info_msg.P = calib_data["projection_matrix"]["data"]
	camera_info_msg.distortion_model = calib_data["distortion_model"]
	return camera_info_msg

if __name__ == '__main__':

	# Filename is the absolute path to the camera_info yaml

	filename = "/home/native/.ros/camera_info/hd_cam.yaml"

	# Parse yaml file
	camera_info_msg = yaml_to_CameraInfo(filename)

	cap = cv2.VideoCapture(0)
	
	if not cap.isOpened():
		print "Error opening resource: " + str(resource)
		print "Maybe opencv VideoCapture can't open it"
		exit(0)

	rval, frame = cap.read()
	
	while not rospy.is_shutdown():
		
		check_enable_camera()
		check_grab_frame()

		if grab_frame:

			# Discard the first 10 frames
			for i in range(0,10):
				rval, frame = cap.read()
			
			rval, frame = cap.read()
			rospy.set_param("/grab_frame", False)

		if enable_camera:

			#cv2.imshow("Stream: " + resource_name, frame)
			#rval, frame = cap.read()
			#image_msg = Image()

			try:
				image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
				image_msg.header.frame_id = "hd_cam"
				camera_info_msg.header.stamp = rospy.Time.now()
				image_msg.header.stamp = camera_info_msg.header.stamp
				#image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
				image_pub.publish(image_msg)
				info_pub.publish(camera_info_msg)
				rate.sleep()
			except CvBridgeError as e:
				print e

		#key = cv2.waitKey(20)
		# print "key pressed: " + str(key)
		# exit on ESC, you may want to uncomment the print to know which key is ESC for you
		#if key == 27 or key == 1048603:
		#	break
#cv2.destroyWindow("preview")