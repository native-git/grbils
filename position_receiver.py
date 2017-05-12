#!/usr/bin/env python

import serial

ser = serial.Serial()
ser.port = '/dev/ttyUSB1'
ser.baudrate = 38400
ser.open()

while True:

	try:
		# Read in a a line from the RF module, and strip the new line character at the end
		msg = ser.readline()
		msg = msg.strip('\n')
		
		# Do some string formatting to parse the message properly and seperate it into the proper components
		trans = msg.split(':')[0].split(",")
		rot = msg.split(':')[1].split(",")
		
		# Convert the string values to floats so that you can do things with them
		x = float(trans[0])
		y = float(trans[1])
		z = float(trans[2])

		roll = float(rot[0])
		pitch = float(rot[1])
		yaw = float(rot[2])

		# Print it nicely
		
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
	except IndexError:
		continue

ser.close()

