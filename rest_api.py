#!/usr/bin/env python

from flask import Flask, request
import socket

app = Flask(__name__)

global server, location_port, target_port, host_port

server = '192.168.207.201'
host_port = 5000
location_port = 10001
target_port = 10002

@app.route('/')
def api_root():
	msg = "/location<br/>/go_to<ensp/>?x='x_pos'&y='y_pos'&r='theta'"
	return msg

@app.route('/location')
def api_location():
	
	try:
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.connect((server,location_port))

		# Receive the auto-replied position information
		msg = s.recv(10000)
		msg = msg.strip('\n')

		# Process the message to get the data we want
		trans = msg.split(':')[0].split(",")
		rot = msg.split(':')[1].split(",")

		x = trans[0]
		y = trans[1]
		z = trans[2]

		roll = rot[0]
		pitch = rot[1]
		yaw = rot[2]
		s.close()
		return 'X: ' + str(x) + " Y: " + str(y) + " THETA: " + str(yaw)

	except IndexError:
		return "Error Connecting...please try again"

@app.route('/go_to')
def go_to():
	
	received = False

	x = request.args.get('x', type=float)
	y = request.args.get('y', type=float)
	yaw = request.args.get('r', type=float)

	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((server,target_port))

	# Receive the auto-replied position information
	msg = str(x)+","+str(y)+","+str(0.0)+":"+str(0.0)+","+str(0.0)+","+str(yaw)+"\n"

	while not received:

		s.send(msg)
		response = s.recv(20)

		try:
			response = response.strip('\n')
			if response == 'BAD':
				break
			elif response == 'OK':
				s.close()
				received = True

		except IndexError:
			return "Please try again, bad connection to target_server"
			continue

	s.close()
	return 'Setting New Target-<br/>X: ' + str(x) + " Y: " + str(y) + " THETA: " + str(yaw)

if __name__ == '__main__':
	app.run(server,host_port)
