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
from std_srvs.srv import Empty
import math

rospy.init_node("generic_accuracy_test")
rate = rospy.Rate(10.0)

listener = tf.TransformListener()
t = tf.Transformer(True, rospy.Duration(20.0))

def clear_costmaps():
	rospy.wait_for_service('move_base/clear_costmaps')
	try:
		clear = rospy.ServiceProxy('move_base/clear_costmaps',Empty)
		response = clear()
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

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

def set_transform(from_a,to_b,(trans,rot)):

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

def initialize():
	check_camera()
	(trans,rot) = get_transform("map","pioneer")
	set_transform("map","origin",(trans,rot))
	launch_static(trans[0],trans[1],rot[2],rot[3])

def move_base_client((trans,rot)):

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

def log_and_print(point_number,point_type,(trans,rot)):
	euler = tf.transformations.euler_from_quaternion(rot)
	yaw = math.degrees(euler[2])
	writer.writerow({'point_number': point_number, 'point_type': point_type, 'x': trans[0], 'y': trans[1], 'yaw': yaw})
	print 'point_number:',point_number,'point_type:',point_type,'x:',trans[0],'y:',trans[1],'yaw:',yaw,""

with open('test_5_odom_only_manual_reposition_return_to_zero.csv', 'ab') as csvfile:

	fieldnames = ['point_number','point_type','x','y','yaw']
	writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
	writer.writeheader()
	kill_odom_transform()
	initialize()
	log_and_print(0,'initial_point',get_transform("map","origin"))

	for i in range(1,21):

		raw_input("Press enter to continue...")
		x = random.uniform(-2,2)
		y = random.uniform(-2,2)
		yaw_deg = random.uniform(-180,180)
		yaw = math.radians(yaw_deg)

		trans = [x,y,0.0]
		rot = tf.transformations.quaternion_from_euler(0.0,0.0,yaw)

		set_transform("origin","target",(trans,rot))	

		# Get position of target relative to origin (point 0)	
		log_and_print(i,'rand_target',get_transform("origin",'target'))
		
		# Clear the costmaps
		clear_costmaps()

		# Move to the random target
		print "Moving to target..."
		move_base_client(get_transform("map","target"))
		
		# Wait for camera to settle down
		time.sleep(3)
		
		# Add the camera data to the tf tree
		check_camera()

		# Record camera at random point
		log_and_print(i,"camera_at_rand",get_transform("origin","pioneer"))

		# Record odometry at random point
		log_and_print(i,"odom_at_rand",lookup_odom())

		# Clear Costmaps again
		clear_costmaps()

		# Return to origin (point 0)
		move_base_client(get_transform("map","origin"))

		# Sleep again
		time.sleep(3)
		
		# Record camera's estimate at origin		
		check_camera()
		log_and_print(i,"camera_at_origin",get_transform("origin","pioneer"))
		
		# Log odometry information relative to origin
		log_and_print(i,"odom_at_origin",lookup_odom())