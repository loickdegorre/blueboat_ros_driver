#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Point

from time import sleep

from mavros_msgs.msg import State, OverrideRCIn

from lib.bboat_lib import *
from bboat_pkg.msg import *
from lib.command_lib import *

import datetime, time
import os



class LoggerNode(): 
	def __init__(self):

		self.rate = rospy.Rate(100)

		self.log_flag = rospy.get_param('/bboat_logger_node/log')
		#if self.log_flag:
			# INSERT LOGGING FILE CREATION + OPENING HERE
			

		# --- Subs
		# INSERT SUBSCRIBERS HERE

		# --- Init done
		rospy.loginfo('[LOGGER] Logger Node Start')

	def loop (self): 
		i=0
		while not rospy.is_shutdown():
			# rospy.loginfo('[LOGGER] Logger Loop')

			self.rate.sleep()
		# --- close files on shutdown



			
if __name__ == '__main__':
	rospy.init_node('logger')

	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		logger = LoggerNode()
		if logger.log_flag:
			rospy.logwarn('[LOGGER] Logging')
			logger.loop()
		else:
			rospy.logwarn('[LOGGER] No logs')
	except rospy.ROSInterruptException: pass