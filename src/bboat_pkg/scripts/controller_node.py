#!/usr/bin/env python3

import rospy

import numpy as np
import math
from pyproj import Proj, transform
from pyproj import Transformer

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Point, Twist

from bboat_pkg.msg import cmd_msg, mode_msg

from lib.bboat_lib import *

from lib.command_lib import *

import time


class ControllerNode(): 
	'''
		Controller Node
		Subscribers
			- Robot pose + heading - x, y, psi
			- Virtual Sailboat position and speed 
		Publishers
			- Command - frwd speed, turning speed
		Service Clients
			- Mode, AUTO vs MANUAL
	'''
	def __init__(self): 

		# --- Constants
		self.dT = 0.02
		self.rate = rospy.Rate(1/self.dT)

		self.pose_robot = np.zeros((3,1))
		self.vel_robot_RB = np.zeros((3,1))

		self.time = 0

		# --- Subs

		self.mode_msg = None
		self.sub_mode = rospy.Subscriber('/modepub', mode_msg, self.Mode_callback)
		rospy.wait_for_message('/modepub', mode_msg,  timeout=None)

		rospy.logwarn("HERE")

		self.sub_pose_robot = rospy.Subscriber('/pose_robot_R0', PoseStamped, self.Pose_Robot_callback)
		rospy.wait_for_message('/pose_robot_R0', PoseStamped, timeout=None)

		self.sub_vel_robot = rospy.Subscriber('/vel_robot_RB', Twist, self.Vel_Robot_callback)

		# --- Pubs

		self.pub_cmd = rospy.Publisher('/command', cmd_msg, queue_size=10)

		# --- Init done
		rospy.loginfo('[CONTROLLER] Controller node Start')

	def loop(self): 

		while not rospy.is_shutdown():

			mode = self.mode_msg

			if mode.mode == "AUTO":	

				# if mode.mission == "VSB" :
					# INSERT CONTROLLER FOR THE VSB MISSION HERE

				self.time += self.dT

				u1 = 0
				u2 = 0
				# ---
				# Build and publish command message
				if abs(u1) > MAX_SPEED_FWRD:
					u1 = np.sign(u1)*MAX_SPEED_FWRD
				if abs(u2) > MAX_SPEED_TURN:
					u2 = np.sign(u2)*MAX_SPEED_TURN	


				self.last_u1, self.last_u2 = u1, u2

				cmd = cmd_msg()
				cmd.u1 = Float64(u1)
				cmd.u2 = Float64(u2)

				self.pub_cmd.publish(cmd)

			self.rate.sleep()

	def Pose_Robot_callback(self, msg):
		'''
			Parse robot pose message - x, y, psi in local lambert frame R0
		'''
		self.pose_robot = np.array([[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z]]) # pose.position.z = psi


	def Vel_Robot_callback(self, msg): 
		self.vel_robot_RB = np.array([[msg.linear.x], [msg.linear.y], [msg.angular.z]])

	def Mode_callback(self, msg):
		self.mode_msg = msg


if __name__ == '__main__':
    rospy.init_node('controller')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        controller = ControllerNode()
        controller.loop()
    except rospy.ROSInterruptException: pass