#!/usr/bin/env python
import rospy
import math
import sys
import random

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

if __name__ == '__main__':
	'''
	Sample code to publish a pcl2 with python
	'''
	rospy.init_node('pcl2_pub_example')
	pcl_pub = rospy.Publisher("/my_pcl_topic", PointCloud2)
	rospy.loginfo("Initializing sample pcl2 publisher node...")
	rate = rospy.Rate(5)
	#give time to roscore to make the connections

	while not rospy.is_shutdown():
		rate.sleep()
		cloud_points = [None] * 10
		#cloud_points = [[1.0, 1.0, 0.0],[1.0, 2.0, 0.0]]
		for i in range(0,len(cloud_points)):
			x = random.uniform(-2,2)
			y = random.uniform(-2,2)
			z = random.uniform(1,1.5)
			cloud_points[i] = [x,y,z]
		#header
		header = std_msgs.msg.Header()
		header.stamp = rospy.Time.now()
		header.frame_id = 'pioneer'
		#create pcl from points
		scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, cloud_points)
		#publish    
		rospy.loginfo("happily publishing sample pointcloud.. !")
		pcl_pub.publish(scaled_polygon_pcl)