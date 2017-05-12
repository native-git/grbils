#! /usr/bin/env python

import rospy
import tf
import actionlib
import move_base_msgs.msg
import os

rospy.init_node('pioneer_goal_client')
listener = tf.TransformListener()
os.system("roslaunch grbils target_launcher.launch x:=0 y:=0 &")

def move_base_client(x,y,qx,qy,qz,qw):

	client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

	client.wait_for_server()

	goal = move_base_msgs.msg.MoveBaseGoal()

	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()

	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y = y

	goal.target_pose.pose.orientation.x = qx
	goal.target_pose.pose.orientation.y = qy
	goal.target_pose.pose.orientation.z = qz
	goal.target_pose.pose.orientation.w = qw

	print "Sending goal..."
	client.send_goal(goal)

	client.wait_for_result()

	result = client.get_result()

	print result

there = False

while not there:

	try:
		(trans,quat) = listener.lookupTransform('/pioneer', '/target', rospy.Time(0))
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		continue

	there = True

x = float(trans[0])
y = float(trans[1])

command = "roslaunch grbils base_link_launcher.launch target:=base_link_target "
command += "x:=" +str(x) + " y:=" +str(y) + " qx:=" +str(quat[0]) + " qy:="+str(quat[1])+" qz:="+str(quat[2]) +" qw:="+str(quat[3])+" &"
os.system(command)

bool_2 = True

while bool_2:

	try:
		(t1,r1) = listener.lookupTransform('/map', '/base_link_target', rospy.Time(0))
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		continue

	bool_2 = False

os.system("rosnode kill /target_launcher &")
os.system("rosnode kill /base_link_launcher &")

a = t1[0]
b = t1[1]

d = r1[0]
e = r1[1]
f = r1[2]
g = r1[3]

move_base_client(a,b,d,e,f,g)
#move_base_client(t1[0],t1[1],r1[0],r1[1].r1[2].r1[3])