#!/usr/bin/env python

import rospy
import math
import tf
import geometry_msgs.msg


def sendTF(inv_trans, inv_rot, fromName, toName):
	br = tf.TransformBroadcaster()

	## traslation, rotation, time of publish, to-frame, from-frame
	br.sendTransform( (inv_trans[0], inv_trans[1], inv_trans[2]),
		(inv_rot[0], inv_rot[1], inv_rot[2], inv_rot[3]), 
		rospy.Time.now(),
		toName,
		fromName)



def Main():

	while not rospy.is_shutdown():
		"""
		try:
			## lookup - from-frame, to-frame, time of transform
			(trans,rot) = listener.lookupTransform('/usb_cam2/ceiling_grid', '/usb_cam2', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		## translation, rotation, from-frame, to-frame
		sendTF(trans, rot, '/ceiling_grid', '/usb_cam2_new')
		"""
		try:
			(trans,rot) = listener.lookupTransform('hd_cam/GrBILS_grid', '/hd_cam', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		
		sendTF(trans, rot, '/GrBILS_grid', '/hd_cam_new')
		
if __name__ == '__main__':
	rospy.init_node('tf_inverter')
	listener = tf.TransformListener()
	rate = rospy.Rate(10.0)
	Main()
