#! /usr/bin/env python

import random
import rospy
import tf
import os
import move_base_msgs.msg
import actionlib
import geometry_msgs.msg
import time
import csv

rospy.init_node("return_to_zero_test")
rate = rospy.Rate(10.0)

listener = tf.TransformListener()
t = tf.Transformer(True, rospy.Duration(20.0))

def kill_odom_transform():
	os.system("rosnode kill /map_2_odom >/dev/null &")

def launch_static(x,y,qz,qw):
	command = "roslaunch grbils map_2_odom.launch "
	command += "x:=" +str(x) + " y:=" +str(y) + " qx:=0.0 qy:=0.0 qz:=" + str(qz) + " qw:=" + str(qw) +" >/dev/null &"
	print command
	os.system(command)

def get_transform(from_a,to_b):

	(trans,rot) = t.lookupTransform(from_a,to_b,rospy.Time(0))
	return (trans,rot)

def set_transform(from_a,to_b,trans,rot):

	m = geometry_msgs.msg.TransformStamped()
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
		rate.sleep()
		return trans, rot

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

	set_transform("map","pioneer",trans,quaternion)

def initialize():
	check_camera()
	(trans,rot) = get_transform("map","pioneer")
	set_transform("map","origin",trans,rot)
	launch_static(trans[0],trans[1],rot[2],rot[3])

def move_base_client(trans,rot):

	client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

	client.wait_for_server()

	goal = move_base_msgs.msg.MoveBaseGoal()

	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()

	goal.target_pose.pose.position.x = trans[0]
	goal.target_pose.pose.position.y = trans[1]

	goal.target_pose.pose.orientation.z = rot[2]
	goal.target_pose.pose.orientation.w = rot[3]

	print "Sending goal..."
	client.send_goal(goal)

	client.wait_for_result()

	result = client.get_result()

with open('test_2_odom_only_return_to_zero.csv', 'ab') as csvfile:

	fieldnames = ['point_number','point_type','x','y','yaw']
	writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
	writer.writeheader()
	kill_odom_transform()
	initialize()
	(ti,ri) = get_transform("map","origin")
	ei = tf.transformations.euler_from_quaternion(ri)
	writer.writerow({'point_number': 0, 'point_type': 'initial_point', 'x': ti[0], 'y': ti[1], 'yaw': ei[2]})
	print 'point_number: 0 point_type: initial_point x:',ti[0],'y:',ti[1],'yaw:',ei[2],""
	
	for i in range(1,20):

		raw_input("Press enter to continue...")
		x = random.uniform(-2,2)
		y = random.uniform(-2,2)
		yaw = random.uniform(-180,180)

		rand_pos = [x,y,yaw]

		trans = [x,y,0.0]
		rot = tf.transformations.quaternion_from_euler(0.0,0.0,yaw)

		set_transform("origin","target",trans,rot)
		(t0,r0) = get_transform("map","target")
		(t1,r1) = get_transform("origin","target")
		e1 = tf.transformations.euler_from_quaternion(r1)
		writer.writerow({'point_number': i, 'point_type': 'rand_target', 'x': t1[0], 'y': t1[1], 'yaw': e1[2]})
		print 'point_number:',i,'point_type: rand_target x:',t1[0],'y:',t1[1],'yaw:',e1[2],""
		print "Moving to target..."
		move_base_client(t0,r0)
		time.sleep(3)
		check_camera()
		(t2,r2) = get_transform("origin","pioneer")
		e2 = tf.transformations.euler_from_quaternion(r2)
		writer.writerow({'point_number': i, 'point_type': 'camera_at_rand', 'x': t2[0], 'y': t2[1], 'yaw': e2[2]})
		print 'point_number:',i,'point_type: camera_at_rand x:',t2[0],'y:',t2[1],'yaw:',e2[2],""
		(t3,r3) = lookup_odom()
		e3 = tf.transformations.euler_from_quaternion(r3)
		writer.writerow({'point_number': i, 'point_type': 'odom_at_rand', 'x': t3[0], 'y': t3[1], 'yaw': e3[2]})
		print 'point_number:',i,'point_type: odom_at_rand x:',t3[0],'y:',t3[1],'yaw:',e3[2],""
		(t4,r4) = get_transform("map","origin")
		print "Returning to origin"
		move_base_client(t4,r4)
		time.sleep(3)
		check_camera()
		(t5,r5) = get_transform("origin","pioneer")
		e5 = tf.transformations.euler_from_quaternion(r5)
		writer.writerow({'point_number': i, 'point_type': 'camera_at_origin', 'x': t5[0], 'y': t5[1], 'yaw': e5[2]})
		print 'point_number:',i,'point_type: camera_at_origin x:',t5[0],'y:',t5[1],'yaw:',e5[2],""
		(t6,r6) = lookup_odom()
		e6 = tf.transformations.euler_from_quaternion(r6)
		writer.writerow({'point_number': i, 'point_type': 'odom_at_origin', 'x': t6[0], 'y': t6[1], 'yaw': e6[2]})
		print 'point_number:',i,'point_type: odom_at_origin x:',t6[0],'y:',t6[1],'yaw:',e6[2],""