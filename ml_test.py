#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
from numpy import interp,sign
from geometry_msgs.msg import Quaternion
import time
import csv
import os

recording = True

#commands = [-150,-100,-50,0,50,100,150]
commands = [-100,-50,0,50,100]
def stop():

	cmd = Quaternion()
	cmd.x = 0
	cmd.y = 0
	cmd.z = 0
	cmd.w = 0
	publisher.publish(cmd)

def go(x,y,z,w):

	cmd = Quaternion()
	cmd.x = x
	cmd.y = y
	cmd.z = z
	cmd.w = w
	publisher.publish(cmd)

def publish_position(x,y,qx,qy,qz,qw):
	
	command = "roslaunch grbils static_tf.launch target:=target "
	command += "x:=" +str(x) + " y:=" +str(y) + " qx:=" +str(qx) + " qy:="+str(qy)+" qz:="+str(qz) +" qw:="+str(qw)+" &"
	os.system(command)

def get_position(from_a,to_b):
	get_one = True

	while get_one:
		try:
			(trans,rot) = listener.lookupTransform(from_a, to_b, rospy.Time(0))
			get_one = False
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

	x = trans[0]
	y = trans[1]
	euler = tf.transformations.euler_from_quaternion(rot)
	yaw = math.degrees(euler[2])
	return x,y,yaw

def record_results(w1,w2,w3,w4,dx,dy,dtheta):

	writer.writerow({'w1': x, 'w2': y, 'w3': z, 'w4':w, 'dx': dx, 'dy': dy, 'dtheta': dtheta})

def check_command(a,b,c,d):
	check = [sign(a),sign(b),sign(c),sign(d)]
	if sign(a)==sign(c) and sign(b)==sign(d) and sign(a)!=sign(b):
		return False
	zero_count = 0
	neg_count = 0
	pos_count = 0
	for i in check:
		if i == 0:
			zero_count += 1
		if i == -1:
			neg_count += 1
		if i == 1:
			pos_count += 1
	counts = [zero_count,neg_count,pos_count]
	for j in counts:
		if j == 1 or j == 3:
			return False
	return True
	
if __name__ == '__main__':

	
	with open('ml_dataset.csv', 'ab') as csvfile:
		
		fieldnames = ['w1','w2','w3','w4','dx','dy','dtheta']
		writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

		writer.writeheader()

		# ROS Specific Setup Stuff:
		#-----------------------
		rospy.init_node('ml_test')
		listener = tf.TransformListener()
		publisher = rospy.Publisher("/holonomic_drive/cmd_vel", Quaternion, queue_size=1)
		rate = rospy.Rate(10)

		while not rospy.is_shutdown():

			while recording:

				for x in commands:
					for y in commands:
						for z in commands:
							for w in commands:

								if check_command(x,y,z,w):
									
									get_one = True

									while get_one:
										print "trying"
										try:
											(trans,rot) = listener.lookupTransform('/map', '/holonomic_drive', rospy.Time(0))
											get_one = False
										except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
											continue

									publish_position(trans[0],trans[1],rot[0],rot[1],rot[2],rot[3])

									go(x,y,z,w)
									time.sleep(1.5)
									stop()
									time.sleep(0.5)

									dx,dy,dtheta = get_position('/target','/holonomic_drive')
									
									os.system("rosnode kill /target_publisher &")
									
									record_results(x,y,z,w,dx,dy,dtheta)
									#record_results(x,y,z,w,'GOOD','GOOD','GOOD')

								else:
									record_results(x,y,z,w,'BAD','BAD','BAD')

				recording = False

			exit()