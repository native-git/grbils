#! /usr/bin/env python

import message_filters
from sensor_msgs.msg import Image, CameraInfo
import rospy

rospy.init_node("camera_synchronizer")

pub_cam = rospy.Publisher("/hd_cam/sync/image_raw",Image, queue_size=10)
pub_info = rospy.Publisher("/hd_cam/sync/camera_info", CameraInfo, queue_size=10)

rate = rospy.Rate(10)

def callback(image, camera_info):
# Solve all of perception here...
	print "--"
	print camera_info
	print "--"
	image.header.stamp = camera_info.header.stamp
	image.header.frame_id = "hd_cam/sync"
	camera_info.header.frame_id = "hd_cam/sync"
	pub_cam.publish(image)
	pub_info.publish(camera_info)
	rate.sleep()

image_sub = message_filters.Subscriber('/hd_cam/image_raw', Image)
info_sub = message_filters.Subscriber('/hd_cam/camera_info', CameraInfo)

while not rospy.is_shutdown():
	ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
	ts.registerCallback(callback)
	rospy.spin()
