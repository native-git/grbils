#!/usr/bin/env python

fr = 30 # frame rate in [frames/sec]
tau = 1/fr # time constant
ang_res = 180/255 # degrees per speed

def f(x):
	y = 0.7212283251*x - 0.6771438007
	return y

for i in range(0,256):
	ang_vel = f(i)
	print "Cmd: " + str(i) + " Velocity: " + str(ang_vel)