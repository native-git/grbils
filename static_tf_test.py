#!/usr/bin/env python

import rospy
import tf
import time
import tf2_ros
import geometry_msgs.msg

dir(tf2_ros)
print dir(geometry_msgs.msg)

y = 1.0

if __name__ == '__main__':

	while True:
		rospy.init_node('static_tf_test')
		broadcaster = tf2_ros.TransformBroadcaster()
		static_transformStamped = geometry_msgs.msg.TransformStamped()
	  
		static_transformStamped.header.stamp = rospy.Time.now()
		static_transformStamped.header.frame_id = "map"
		static_transformStamped.child_frame_id = "target"
	  
		static_transformStamped.transform.translation.x = 15.5
		static_transformStamped.transform.translation.y = y
		static_transformStamped.transform.translation.z = 0
	  
		static_transformStamped.transform.rotation.x = 0.0
		static_transformStamped.transform.rotation.y = 0.0
		static_transformStamped.transform.rotation.z = 0.0
		static_transformStamped.transform.rotation.w = 1.0
	 
		broadcaster.sendTransform(static_transformStamped)
		time.sleep(3)
		print y
		y += 0.5
		if y >= 20:
			y = 1.0