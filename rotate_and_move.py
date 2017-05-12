#! /usr/bin/env python

import rospy
import tf
import math
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,TransformStamped

rospy.init_node("rotate_and_move")
rate = rospy.Rate(10.0)

listener = tf.TransformListener()
t = tf.Transformer(True, rospy.Duration(20.0))
pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

def get_transform(from_a,to_b):

	(trans,rot) = t.lookupTransform(from_a,to_b,rospy.Time(0))
	return (trans,rot)

def set_transform(from_a,to_b,(trans,rot)):

	m = TransformStamped()
	m.header.frame_id = from_a
	m.child_frame_id = to_b
	m.transform.translation.x = trans[0]
	m.transform.translation.y = trans[1]
	m.transform.rotation.x = 0.0
	m.transform.rotation.y = 0.0
	m.transform.rotation.z = rot[2]
	m.transform.rotation.w = rot[3]
	t.setTransform(m)

def lookup_odom():

	got_one = False

	while not got_one:
		
		try:
			(trans,rot) = listener.lookupTransform("/odom", "/base_link", rospy.Time(0))
			got_one = True
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		return (trans, rot)

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
	trans = (x_avg,y_avg,0.0)
	set_transform("map","pioneer",(trans,quaternion))

def odometryCb(msg):
	global cur_pos_x
	global cur_vel_x
	#print msg.pose.pose
	cur_pos_x = msg.pose.pose.position.x
	cur_vel_x = msg.twist.twist.linear.x
	sub_once.unregister()

def pub_cmd_vel(v_x,v_theta):

	msg = Twist()
	msg.linear.x = v_x
	msg.angular.z = v_theta
	pub.publish(msg)

def move_to(x):
	
	dt = 0.10
	transacc = 0.3 # 0.3 meters/sec^2
	transvelmax = 0.75 # 0.75 meters/sec
	threshold = 0.005 # set a 5mm tolerance for accepting the final position
	kp = 1.15

	i = 0

	set_transform("odom","initial_pose",lookup_odom())
	trans = (x,0.0,0.0)
	rot = (0.0,0.0,0.0,1.0)
	set_transform("initial_pose","target_x",(trans,rot))

	there = False
	slow_flag = False

	while not there:

		if i > 0:
			delay = 1 + 0.5*i
			time.sleep(delay)

		set_transform("odom","base_link",lookup_odom())
		(t,r) = get_transform("base_link","target_x")
		dist = t[0]
		if abs(dist) <= threshold:
			pub_cmd_vel(0,0)
			print "Target Achieved"
			there = True
			break
		
		x_vel = math.sqrt(dist*transacc/3)

		if x_vel >= transvelmax:
			x_vel = transvelmax
		if x_vel <= 0.001:
			x_vel = 0.001
		if slow_flag:
			x_vel_actual *= 0.5
		else:
			x_vel_actual = x_vel
		d_stop = (x_vel*dt)+((x_vel**2)/(2*transacc))
		x_vel = x_vel_actual
		print "x_vel: " + str(x_vel)
		print "x_dist: " + str(dist)
		print "d_stop: " + str(d_stop)
		#d_stop *= kp
		print "d_stop*kp: " + str(d_stop)

		rate.sleep()

		trying = True

		print i

		while trying:

			set_transform("odom","base_link",lookup_odom())
			(t,r) = get_transform("base_link","target_x")
			dist = t[0]

			if abs(dist) <= d_stop:
				pub_cmd_vel(0,0)
				i += 1
				trying = False
			if (dist - d_stop)/x_vel <= 2*dt:
				slow_flag = True
				pub_cmd_vel(0,0)
				i += 1
				trying = False
			else:
				pub_cmd_vel(x_vel,0)
				rate.sleep()

		rate.sleep()

	time.sleep(3)
	set_transform("odom","base_link",lookup_odom())
	(t,r) = get_transform("base_link","target_x")
	dist = t[0]
	print "Final dist: " + str(dist)

def rotate_to(theta):

	dt = 0.10
	rotacc = 100 # deg/sec^2 = 1.74533 rad/sec^2
	rotvelmax = 100 # deg/sec^2 = 1.74533 rad/sec^2
	kp = 1.5
	threshold = 0.5

	set_transform("odom","initial_pose",lookup_odom())
	quat = tf.transformations.quaternion_from_euler(0.0,0.0,math.radians(theta))
	trans = (0.0,0.0,0.0)
	set_transform("initial_pose","target_theta",(trans,quat))

	there = False

	ang_vel = 0.0
	i = 0

	while not there:

		if i > 0:
			delay = 1 + 0.5*i
			time.sleep(delay)

		set_transform("odom","base_link",lookup_odom())
		(t,r) = get_transform("base_link","target_theta")
		yaw_rad = r[2]
		rot = tf.transformations.euler_from_quaternion(r)
		yaw_deg = math.degrees(rot[2])
		if abs(yaw_deg) <= threshold:
			pub_cmd_vel(0,0)
			print "Target Achieved"
			there = True
		sign = (yaw_deg/abs(yaw_deg))
		ang_vel_deg = math.sqrt(abs(yaw_deg)*rotacc/3)
		if ang_vel_deg >= rotvelmax:
			ang_vel_deg = rotvelmax
		if ang_vel_deg <= 0.5:
			ang_vel_deg = 0.5
		ang_vel_deg *= sign
		ang_vel_deg = ang_vel_deg/kp
		#ang_vel_deg *= kp
		d_stop = (abs(ang_vel_deg)*dt)+((ang_vel_deg**2)/(2*rotacc))

		print "Ang_vel_rad: " + str(math.radians(ang_vel_deg))
		print "Ang_dist_deg: " + str(yaw_deg)
		print "d_stop: " + str(d_stop)

		rate.sleep()

		trying = True

		print i

		while trying:

			set_transform("odom","base_link",lookup_odom())
			(t,r) = get_transform("base_link","target_theta")
			yaw_rad = r[2]
			rot = tf.transformations.euler_from_quaternion(r)
			yaw_deg = math.degrees(rot[2])

			if abs(yaw_deg) <= d_stop:
				pub_cmd_vel(0,0)
				i += 1
				trying = False
			else:
				pub_cmd_vel(0,math.radians(ang_vel_deg))
				rate.sleep()

		rate.sleep()

	time.sleep(3)
	set_transform("odom","base_link",lookup_odom())
	(t,r) = get_transform("base_link","target_theta")
	yaw_rad = r[2]
	rot = tf.transformations.euler_from_quaternion(r)
	yaw_deg = math.degrees(rot[2])
	print "Final angle: " + str(yaw_deg)
