#!/usr/bin/env python3

import rospy

import numpy as np
import math
from pyproj import Proj, transform
from pyproj import Transformer

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Point

from bboat_pkg.msg import cmd_msg
from bboat_pkg.srv import mode_serv, mode_servResponse, current_target_serv

from lib.bboat_lib import *

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

		# ---
		self.u_SB_thresh = 0.08

		self.rate = rospy.Rate(50)

		self.pose_robot = np.zeros((3,1))

		self.pose_vsb = np.zeros((3,1))

		self.speed_vsb = np.zeros((3,1))

		# --- Subs
		self.sub_pose_robot = rospy.Subscriber('/pose_robot_R0', PoseStamped, self.Pose_Robot_callback)
		rospy.wait_for_message('/pose_robot_R0', PoseStamped, timeout=None)

		self.sub_vsb_pose = rospy.Subscriber('/vSBPosition', PoseStamped, self.Pose_vSB_callback)

		self.sub_vsb_speed = rospy.Subscriber('/vSBSpeed', PoseStamped, self.Speed_vSB_callback)

		# --- Pubs
		self.pub_cmd = rospy.Publisher('/command', cmd_msg, queue_size=10)

		# --- Services
		rospy.wait_for_service('/mode')
		connected = False
		while not connected:
			try:
				self.client_mode = rospy.ServiceProxy('/mode', mode_serv)
				connected = True
			except rospy.ServiceException as exc:
				rospy.logwarn(f'[CONTROLLER] Mode service cannot be reached - {str(exc)}')
				connected = False

		rospy.wait_for_service('/current_target')
		connected = False
		while not connected:
			try:
				self.client_target = rospy.ServiceProxy('/current_target', current_target_serv)
				connected = True
			except resoyServiceException as exc:
				rospy.logwarn(f'[CONTROLLER] Current Target service cannot be reached - {str(exc)}')
				connected = False


		# --- Init done
		rospy.loginfo('[CONTROLLER] Controller node Start')

	def loop(self): 
		while not rospy.is_shutdown():
			mode = self.client_mode(True)

			if mode.mode == "AUTO":	
				x_rob, y_rob, psi_rob = self.pose_robot.flatten()
				x_SB, y_SB, psi_SB = self.pose_vsb.flatten()
				dx_SB, dy_SB, r_SB = self.speed_vsb.flatten() 
				u1 = 0.0
				u2 = 0.0
				if mode.mission == "VSB" :
					# ---
					# u1 - Forward speed
					kp = 1
					# Error in robot frame
					err_x_in_Rrob = (x_SB-x_rob)*math.cos(psi_rob) + (y_SB-y_rob)*math.sin(psi_rob)
					err_y_in_Rrob = -(x_SB-x_rob)*math.sin(psi_rob) + (y_SB-y_rob)*math.cos(psi_rob)

					vSB_speed_in_Rrob = dx_SB*math.cos(psi_rob) + dy_SB*math.sin(psi_rob)
					u1 = (vSB_speed_in_Rrob + kp*err_x_in_Rrob)
					
					# ---
					# u2 - Turning speed
					Kp = 0.5
					u_SB =  dx_SB*math.cos(psi_SB) + dy_SB*math.sin(psi_SB)
					v_SB = -dx_SB*math.sin(psi_SB) + dy_SB*math.cos(psi_SB)
					if u_SB > self.u_SB_thresh: 
						psi_des = psi_SB + math.atan2(v_SB, u_SB)
					else: 
						psi_des = psi_SB

					u2 = r_SB + Kp*sawtooth(psi_des - psi_rob)
				elif mode.mission == "CAP":
					u1 = 0.0

					psi_des = math.pi/2
					Kp = 0.5
					u2 = Kp*sawtooth(psi_des - psi_rob)

				elif mode.mission == "PTN":
					pt = self.client_target(True)
					x_tgt, y_tgt = pt.target.x, pt.target.y

					# err_x_in_Rrob = (x_tgt-x_rob)*math.cos(psi_rob) + (y_tgt-y_rob)*math.sin(psi_rob)
					# err_y_in_Rrob = -(x_tgt-x_rob)*math.sin(psi_rob) + (y_tgt-y_rob)*math.cos(psi_rob)

					err_x_R0 = x_tgt - x_rob
					err_y_R0 = x_tgt - y_rob


					u1 = 0.0

					psi_des = math.atan2(err_y_R0, err_x_R0)
					print(psi_des*180/math.pi)

					Kp = 0.5
					# print(f'err {sawtooth(psi_des - psi_rob)}')
					u2 = Kp*sawtooth(psi_des - psi_rob)

				# ---
				# Build and publish command message
				cmd = cmd_msg()
				cmd.u1 = Float64(u1)
				cmd.u2 = Float64(u2)

				self.pub_cmd.publish(cmd)

			self.rate.sleep()

	def Pose_Robot_callback(self, msg):
		'''
			Parse robot pose message - x, y, psi in local lambert frame R0
		'''
		self.pose_robot[0] = msg.pose.position.x
		self.pose_robot[1] = msg.pose.position.y
		self.pose_robot[2] = msg.pose.position.z # psi

	def Pose_vSB_callback(self, msg):
		'''
			Parse Virtuasl Sailboat pose message - x, y, psi considered in local lambert frame R0
		'''
		self.pose_vsb[0] = msg.pose.position.x
		self.pose_vsb[1] = msg.pose.position.y
		self.pose_vsb[2] = msg.pose.position.z


	def Speed_vSB_callback(self, msg):
		'''
			Parse Virtuasl Sailboat speed message - dx, dy, dpsi considered in local lambert frame R0
		'''
		self.speed_vsb[0] = msg.pose.position.x
		self.speed_vsb[1] = msg.pose.position.y
		self.speed_vsb[2] = msg.pose.position.z

if __name__ == '__main__':
    rospy.init_node('controller')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        controller = ControllerNode()
        controller.loop()
    except rospy.ROSInterruptException: pass