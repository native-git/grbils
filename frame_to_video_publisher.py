#! /usr/bin/env python2

import time
from sensor_msgs.msg import Image
import yaml
from sensor_msgs.msg import CameraInfo
import rospy

# initialize values
update_requested = False

rospy.init_node('frame_to_video_publisher')

image_pub = rospy.Publisher("/hd_cam/image_raw",Image, queue_size=10)

info_pub = rospy.Publisher("/hd_cam/camera_info", CameraInfo, queue_size=10)

rate = rospy.Rate(10)

def enable_camera():
	rospy.set_param("/enable_camera", True)

def disable_camera():
	rospy.set_param("/enable_camera", False)

def enable_grab_frame():
	rospy.set_param("/grab_frame", True)

def retrieve_image():

	global image_msg

	got_one = False

	while not got_one:
		try:
			image_msg = rospy.wait_for_message("/hd_cam/frames", Image)
			got_one = True
		except:
			(rospy.exceptions.ROSException,rospy.exceptions.ROSInterruptException)
			continue
		rate.sleep()

def check_for_update_request():
	global update_requested
	update_requested = rospy.get_param("/request_update", False)

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

	enable_camera()
	retrieve_image()
	disable_camera()

	while not rospy.is_shutdown():

		check_for_update_request()

		if update_requested:
			enable_camera()
			retrieve_image()
			disable_camera()
			rospy.set_param("/request_update", False)

		camera_info_msg.header.stamp = rospy.Time.now()
		image_msg.header.stamp = camera_info_msg.header.stamp
		image_pub.publish(image_msg)
		info_pub.publish(camera_info_msg)
		rate.sleep()