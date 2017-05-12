#! /usr/bin/env python

import rospy
import tf
import geometry_msgs.msg

rospy.init_node('accuracy_testing')
rate = rospy.Rate(10.0)
listener = tf.TransformListener()

def check_position():

	i = 0
	x_old = 0
	y_old = 0
	yaw_old = 0

	while i <= 21:

		try:
			(trans,rot) = listener.lookupTransform('/odom_guess', '/pioneer', rospy.Time(0))
			rate.sleep()
			i += 1
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		euler = tf.transformations.euler_from_quaternion(rot)

		x = trans[0]
		y = trans[1]

		yaw = euler[2]

		x_avg = (x + ((i-1)*x_old))/i
		y_avg = (y + ((i-1)*y_old))/i
		yaw_avg = (yaw + ((i-1)*yaw_old))/i

		x_old = x_avg
		y_old = y_avg
		yaw_old = yaw_avg
		
	quaternion = tf.transformations.quaternion_from_euler(0.0,0.0,yaw_avg)

	trans = (x_avg,y_avg,0.0)

	print "Trans:"
	print "\tx: " + str(trans[0])
	print "\ty: " + str(trans[1])
	print "Rot: "
	print "\tyaw: " + str(yaw_avg) 

check_position()