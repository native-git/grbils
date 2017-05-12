#! /usr/bin/env python

import rospy
import tf
import os

rospy.init_node('odom_tf_manager')
listener = tf.TransformListener()

def lookup_tf(from_a,to_b):
	
	trans = None

	while trans is None:
		
		try:
			(trans,rot) = listener.lookupTransform(from_a, to_b, rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

	return trans,rot

def publish_tf(from_a,to_b,trans,rot):

	x = trans[0]
	y = trans[1]

	command = "roslaunch grbils odom_tf_manager_launcher.launch"
	command += " from_a:=" + str(from_a) + " to_b:=" + str(to_b)
	command += " x:=" +str(x) + " y:=" +str(y) + " qx:=0 qy:=0 qz:="+str(rot[2]) +" qw:="+str(rot[3])+" &"
	os.system(command)

def kill_tf_publisher(node_name):

	command = "rosnode kill /"
	command += str(node_name) + " &"

start_trans = [-7.327,-1.152,0.0]
start_rot = [0.0,0.0,-0.69,0.723]

publish_tf("odom","map",start_trans,start_rot)
t_i,r_i = lookup_tf("map","pioneer")
kill_tf_publisher("odom_tf_manager_launcher")
publish_tf("map","odom",t_i,r_i)

print t_i
print r_i

#t1,r1 = lookup_tf("/base_link","/odom")
#t2,r2 = lookup_tf("/base_link","/odom")