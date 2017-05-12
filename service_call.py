#! /usr/bin/env python

import rospy
from roscpp_tutorials.srv import TwoInts

def rotate_client(theta):
	rospy.wait_for_service('/rosaria/rotate')
	try:
		rotate = rospy.ServiceProxy('/rosaria/rotate', TwoInts)
		rotate(theta,0)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def move_client(x):
	rospy.wait_for_service('/rosaria/move')
	try:
		rotate = rospy.ServiceProxy('/rosaria/move', TwoInts)
		rotate(x,0)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

