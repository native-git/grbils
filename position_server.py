#!/usr/bin/env python

import socket
import rospy
import math
import tf

s = socket.socket()
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
host = '192.168.207.201'
port = 10001
s.bind((host,port))
s.listen(5)

def Main():

	while not rospy.is_shutdown():

		c, addr = s.accept()

		try:
			(trans,rot) = listener.lookupTransform('/map', '/hd_cam_new', rospy.Time(0))
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

		msg = str(x)+","+str(y)+","+str(z)+":"+str(roll)+","+str(pitch)+","+str(yaw)+"\n"

		c.send(msg)
		c.close()


if __name__ == '__main__':
	rospy.init_node('position_server')
	listener = tf.TransformListener()
	rate = rospy.Rate(10.0)
	Main()