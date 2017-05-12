#!/usr/bin/env python

import socket

# Connect to the server
#s.connect((server,port))

while True:

	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	server = '192.168.207.201'
	port = 10000
	s.connect((server,port))

	try:
		# Receive the auto-replied position information
		msg = s.recv(10000)
		msg = msg.strip('\n')

		# Process the message to get the data we want
		trans = msg.split(':')[0].split(",")
		rot = msg.split(':')[1].split(",")

		x = trans[0]
		y = trans[1]
		z = trans[2]

		roll = rot[0]
		pitch = rot[1]
		yaw = rot[2]

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
		s.close()

	except IndexError:
		continue

s.close()