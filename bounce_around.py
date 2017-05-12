#! /usr/bin/env python

import os
import time

i = 0

while i <= 10:
	os.system("./pioneer_goal_client.py 3.0 0.0")
	time.sleep(3)
	os.system("./pioneer_goal_client.py 0.0 0.0")
	time.sleep(3)
	print "done"
	i += 1
