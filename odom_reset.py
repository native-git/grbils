#!/usr/bin/env python

import rospy
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import os
import time

t = tf.Transformer(True, rospy.Duration(20.0))

def kill_odom_transform():
	os.system("rosnode kill /map_2_odom >/dev/null &")

def launch_static(x,y,qz,qw):
	command = "roslaunch grbils map_2_odom.launch "
	command += "x:=" +str(x) + " y:=" +str(y) + " qx:=0.0 qy:=0.0 qz:=" + str(qz) + " qw:=" + str(qw) +" >/dev/null &"
	print command
	os.system(command)

def set_transform(from_a,to_b,x,y,qz,qw):

	m = geometry_msgs.msg.TransformStamped()
	m.header.frame_id = from_a
	m.child_frame_id = to_b
	m.transform.translation.x = x
	m.transform.translation.y = y
	m.transform.rotation.x = 0.0
	m.transform.rotation.y = 0.0
	m.transform.rotation.z = qz
	m.transform.rotation.w = qw
	t.setTransform(m)

def check_for_request():

	got_one = False

	while not got_one:
		try:
			update_msg = rospy.wait_for_message("update_odom", Empty)
			got_one = True
		except:
			(rospy.exceptions.ROSException,rospy.exceptions.ROSInterruptException)
			continue
		rate.sleep()

def check_stationary():

	stationary = False
	
	cmd_msg = rospy.wait_for_message("cmd_vel", Twist)
	
	if cmd_msg.linear.x == 0 and cmd_msg.angular.z == 0:
		stationary = True
	
	return stationary

def check_camera():

	i = 0
	x_old = 0
	y_old = 0
	yaw_old = 0

	while i <= 21:

		try:
			(trans,rot) = listener.lookupTransform('/map', '/pioneer', rospy.Time(0))
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

	set_transform("map_guess","camera_guess",x_avg,y_avg,quaternion[2],quaternion[3])
	
def Main():

	running = False

	while not rospy.is_shutdown():
		
		if running:
			check_for_request()
			print "Update requested"

		"""

		static = check_stationary()
		
		if static:
		"""
		try:
			(trans,rot) = listener.lookupTransform('/base_link', '/odom', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		set_transform("camera_guess","temp_odom",trans[0],trans[1],rot[2],rot[3])

		print "Not moving, estimating position"
		check_camera()
		if running:
			print "Killing previous transform"
			kill_odom_transform()
		(lin,ang) = t.lookupTransform("map_guess","temp_odom",rospy.Time(0))
		print "Publishing tf map -> odom"
		launch_static(lin[0],lin[1],ang[2],ang[3])
		running = True
		time.sleep(3)
		"""
		else:
			print "The robot is still moving, please try again later"
		"""
if __name__ == '__main__':
	rospy.init_node('odom_reset')
	listener = tf.TransformListener()
	rate = rospy.Rate(15.0)
	Main()
