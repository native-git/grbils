#!/usr/bin/env python

import socket
import rospy
import math
import tf
import geometry_msgs.msg
import os

s = socket.socket()
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
host = '192.168.207.201'
port = 10002
s.bind((host,port))
s.listen(5)

os.system("roslaunch grbils static_tf.launch target:=startup x:=5 y:=5 &")

def Main():

	while not rospy.is_shutdown():

		c, addr = s.accept()

		try:
			msg = c.recv(10000)
			msg = msg.strip('\n')

			# Process the message to get the data we want
			trans = msg.split(':')[0].split(",")
			rot = msg.split(':')[1].split(",")

			x = float(trans[0])
			y = float(trans[1])
			z = float(trans[2])

			roll = math.radians(float(rot[0]))
			pitch = math.radians(float(rot[1]))
			yaw = math.radians(float(rot[2]))

			quat = tf.transformations.quaternion_from_euler(roll,pitch,yaw)

			#print "Received values x: " + str(x) + " y: " + str(y) + " theta: " + str(roll)

			os.system("rosnode kill /target_publisher &")
			command = "roslaunch grbils static_tf.launch target:=target "
			command += "x:=" +str(x) + " y:=" +str(y) + " qx:=" +str(quat[0]) + " qy:="+str(quat[1])+" qz:="+str(quat[2]) +" qw:="+str(quat[3])+" &"
			os.system(command)

			"""
			t = geometry_msgs.msg.TransformStamped()
			t.header.frame_id = "/map"
			t.header.stamp = rospy.Time.now()
			t.child_frame_id = "/target"
			t.transform.translation.x = x
			t.transform.translation.y = y
			t.transform.translation.z = 0.0

			t.transform.rotation.x = quat[0]
			t.transform.rotation.y = quat[1]
			t.transform.rotation.z = quat[2]
			t.transform.rotation.w = quat[3]

			tfm = tf.msg.tfMessage([t])
			publisher.publish(tfm)
			"""
			c.send("OK")
			c.close()

			#rate.sleep()

		except IndexError:
			c.send("BAD")
			continue

if __name__ == '__main__':
	rospy.init_node('target_server')
	#publisher = rospy.Publisher("/tf_static", tf.msg.tfMessage, queue_size=0, latch=1)
	#rate = rospy.Rate(10.0)
	Main()