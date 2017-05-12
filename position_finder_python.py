#!/usr/bin/env python

import rospy
import math
import tf
from time import sleep

def Main():

	while not rospy.is_shutdown():

		try:
			(trans,rot) = listener.lookupTransform('/map', '/holonomic_drive', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		# trans contains the x, y, and z components of the position of the camera with respect to map (units are in meters)
		
		x = trans[0]
		y = trans[1]
		z = trans[2]

		# rot is a quaternion containing the rotational components of the translation between the map and the camera
		# Since euler angles are somewhat easier to work with, we will convert to those:

		euler = tf.transformations.euler_from_quaternion(rot)
		
		# Lastly, since the default units are radians, we will convert to degrees since it is more intuitive
		roll = math.degrees(euler[0])
		pitch = math.degrees(euler[1])
		yaw = math.degrees(euler[2])

		print "TRANSLATIONAL COMPONENTS"
		print "X: " + str(x)
		print "Y: " + str(y)
		print "Z: " + str(z)
		print ""
		print "ROTATIONAL COMPONENTS"
		print "ROLL: " + str(roll)
		print "PITCH: " + str(pitch)
		print "YAW: " + str(yaw)
		print "------------------------------"
		sleep(0.5)

if __name__ == '__main__':
	rospy.init_node('position_sender')
	listener = tf.TransformListener()
	rate = rospy.Rate(10.0)
	Main()
