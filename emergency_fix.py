#! /usr/bin/env python

import rospy
import tf
import geometry_msgs.msg
import actionlib
import move_base_msgs.msg
import os
import sys

# Target Values with respect to odom_guess
#x = 0.0
#y = 0.0
x = 3.138
y = 1.573
yaw = 1.984

#print sys.argv[1]
#print sys.argv[2]

targ_trans = [x,y,0.0]
quat = tf.transformations.quaternion_from_euler(0.0,0.0,yaw)

rospy.init_node('pioneer_goal_client')
rate = rospy.Rate(10.0)
listener = tf.TransformListener()
t = tf.Transformer(True, rospy.Duration(20.0))

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

set_transform("odom_guess","target",targ_trans,quat)

def get_transform(from_a,to_b):

	got_one = False

	while not got_one:
		
		try:
			(trans,rot) = listener.lookupTransform(from_a, to_b, rospy.Time(0))
			got_one = True
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		rate.sleep()
		return trans, rot

def add_to_tree(from_a,to_b):

	(trans,rot) = get_transform(from_a,to_b)
	set_transform(from_a,to_b,trans,rot)

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

def move_base_client(trans,rot):

	client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

	client.wait_for_server()

	goal = move_base_msgs.msg.MoveBaseGoal()

	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()

	goal.target_pose.pose.position.x = -2.233
	goal.target_pose.pose.position.y = 10.581

	goal.target_pose.pose.orientation.z = 0.703
	goal.target_pose.pose.orientation.w = 0.711

	print "Sending goal..."
	client.send_goal(goal)

	client.wait_for_result()

	result = client.get_result()

	print result

move_base_client(targ_trans,quat)