#!/usr/bin/env python3

import rospy
import numpy as np

from time import sleep

from mavros_msgs.msg import State, OverrideRCIn

class CameraNode():
	'''
		Camera Node
		Drives the camera to the vSB heading angle

	'''
	def __init__(self):


		# --- Pubs
		self.pub_rc_override = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

		# --- Init done
		rospy.loginfo('[CAMERA] Camera Node Start')

	def loop(self): 
		rospy.loginfo('[CAMERA] Camera node loop') 
		rc_msg = OverrideRCIn()
		for i in range(0,16):
			chan = [1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500]

			chan[i] = 1700
			testst


			rc_msg.channels = chan
			self.pub_rc_override.publish(rc_msg)
			print(i)

			sleep(1)

		rc_msg.channels = [1500,1500,1500,1500,1600,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500]
		self.pub_rc_override.publish(rc_msg)

if __name__ == '__main__':
	rospy.init_node('camera')

	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		camera = CameraNode()
		camera.loop()
	except rospy.ROSInterruptException: pass
