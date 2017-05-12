#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
from numpy import interp,sign
from geometry_msgs.msg import Quaternion
import time
import csv

# Initial delay

time.sleep(3)

def get_angle():
	get_one = True

	while get_one:
		try:
			(trans,rot) = listener.lookupTransform('/map', '/holonomic_drive', rospy.Time(0))
			get_one = False
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

	euler = tf.transformations.euler_from_quaternion(rot)
	yaw = math.degrees(euler[2])
	return yaw

def stop():

	cmd = Quaternion()
	cmd.x = 0
	cmd.y = 0
	cmd.z = 0
	cmd.w = 0
	publisher.publish(cmd)


if __name__ == '__main__':

	
	with open('velocity_test.csv', 'w') as csvfile:
		fieldnames = ['Speed', 'angular_velocity']
		writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

		writer.writeheader()

		commands = [None]*4

		# ROS Specific Setup Stuff:
		#-----------------------
		rospy.init_node('angular_velocity_test')
		listener = tf.TransformListener()
		publisher = rospy.Publisher("/holonomic_drive/cmd_vel", Quaternion, queue_size=1)
		rate = rospy.Rate(10)
		#-----------------------
		wheels = [None] * 4
		Vmax = 255

		for speed in range(Vmax, -15, -15):
			
			count = 0
			
			while count <= 3:

				delta_t = 0.25

				cmd = Quaternion()
				yaw_1 = get_angle()
				cmd.x = speed
				cmd.y = speed
				cmd.z = speed
				cmd.w = speed
				publisher.publish(cmd)
				time.sleep(delta_t)
				stop()
				time.sleep(0.5)
				yaw_2 = get_angle()
				
				if sign(yaw_1) == -1 and sign(yaw_2) == 1:
					d_theta = 360 - (yaw_2 - yaw_1)
				else:
					d_theta = abs(yaw_2 - yaw_1)

				angular_velocity = d_theta/delta_t 
				writer.writerow({'Speed': str(speed), 'angular_velocity': str(angular_velocity)})
				print "Speed: " + str(speed) + " Angular Velocity: " + str(angular_velocity)
				count += 1