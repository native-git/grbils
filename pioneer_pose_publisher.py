#!/usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

def Main():

	while not rospy.is_shutdown():

		try:
			(trans,rot) = listener.lookupTransform('/map', '/hd_cam_new', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		pose = PoseWithCovarianceStamped()
		pose.header.frame_id = 'map'
		pose.header.stamp = rospy.Time(0)
		
		pose.pose.pose.position.x = trans[0]
		pose.pose.pose.position.y = trans[1]
		pose.pose.pose.position.z = trans[2]
		
		pose.pose.pose.orientation.x = rot[0]
		pose.pose.pose.orientation.y = rot[1]
		pose.pose.pose.orientation.z = rot[2]
		pose.pose.pose.orientation.w = rot[3]

		pose.pose.covariance[0] = 0.01 #x_covariance
		pose.pose.covariance[7] = 0.01 #y_covariance
		pose.pose.covariance[35] = 0.0872665 #z_ang covariance (I think)

		publisher.publish(pose)
		rate.sleep()
		
if __name__ == '__main__':
	rospy.init_node('pioneer_pose_publisher')
	publisher = rospy.Publisher('pioneer/pose', PoseWithCovarianceStamped, queue_size=1)
	listener = tf.TransformListener()
	rate = rospy.Rate(10.0)
	Main()