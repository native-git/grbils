#!/usr/bin/env python

import argparse

url_for_location = "http://192.168.207.201:5000/location"
url_for_target = "http://192.168.207.201:5000/go_to?"

parser = argparse.ArgumentParser()

parser.add_argument("x", type=float,
					help="input the x-coordinate of the target")
parser.add_argument("y", type=float,
					help="input the y-coordinate of the target")
parser.add_argument("theta", type=float,
					help="input the theta of the target")

parser.add_argument("-l", "--location", action="store_true", help="set to find location (set either location or target - location flag has precedence over target)")
parser.add_argument("-t", "--target", action="store_true", help="set to specify the target (set either location or target - location flag has precedence over target)")

args = parser.parse_args()

if args.location:
	print url_for_location
elif args.target:
	url_for_target = url_for_target + "x="+str(args.x) + "&y="+str(args.y) + "&r="+str(args.theta)
	print url_for_target
else:
	print "Wrong usage"
	print "use the following command to see usage:"
	print "./rest_wrapper -h"