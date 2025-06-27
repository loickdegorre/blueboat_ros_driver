#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64

from time import sleep
import time

from mavros_msgs.msg import State, OverrideRCIn

from bboat_lib import *
from command_lib import *
from bboat_pkg.msg import *
from bboat_pkg.srv import path_description_serv


import datetime, time
import os

from virtual_sb_node import WIND_ANGLE, WIND_SPEED

class PlotterNode(): 
	def __init__(self):

		# rospy.wait_for_service('/get_spline_points')
		# connected = False
		# while not connected:
		# 	try:
		# 		self.path_points_client = rospy.ServiceProxy('/get_spline_points', path_description_serv)
		# 		connected = True
		# 	except rospy.ServiceException as exc:
		# 		rospy.logwarn(f'[CONTROLLER] Path spline service cannot be reached - {str(exc)}')
		# 		connected = False

		# self.path_points = reconstruct_spline_matrix(self.path_points_client())

		self.rate = rospy.Rate(15)

		self.flag_plotting = rospy.get_param('/bboat_plotter_node/plot')

		self.mode_simu = rospy.get_param('/bboat_plotter_node/mission_simu')

		self.flag_mission_traj = rospy.get_param('/bboat_plotter_node/mission_traj')

		# --- Subs
		self.sub_pose_robot = rospy.Subscriber('/pose_robot_R0', PoseStamped, self.Pose_Robot_callback)
		self.sub_target = rospy.Subscriber('/control_target', Point, self.Control_Target_callback)
		# rospy.wait_for_message('/pose_robot_R0', PoseStamped, timeout=None)
		self.x_rob_store = []
		self.y_rob_store = []
		self.psi_rob_store = []
		self.pose_rob = np.zeros((3,1))



		# rospy.wait_for_service('/current_target')
		# self.client_target = rospy.ServiceProxy('/current_target', current_target_serv)

		# rospy.wait_for_service('/control_target')
		# self.control_target = rospy.ServiceProxy('/control_target', current_target_serv)
		self.control_target = Point()

		self.xmin, self.xmax = -10, 10
		self.ymin, self.ymax = -10, 10


		if self.flag_mission_traj: 
			rospy.wait_for_service('/get_traj')
			self.client_traj = rospy.ServiceProxy('/get_traj', traj_serv)
			self.traj = self.client_traj()
			# print('PLOTTER - GOT TRAJ')
			# print(self.traj.trajx)
			# self.xmin, self.xmax = min(self.traj.trajx), max(self.traj.trajx)
			# self.ymin, self.ymax = min(self.traj.trajy), max(self.traj.trajy)

		else: 
			self.sub_vsb_pose = rospy.Subscriber('/vSBPosition', PoseStamped, self.Pose_vSB_callback)
			self.x_vsb_store = []
			self.y_vsb_store = []
			self.psi_vsb_store = []
			self.pose_vsb = np.zeros((3,1))
			self.vel_vsb_Rvsb = np.zeros((3,1))
			self.u_vsb_store = []
			self.v_vsb_store = []
			self.r_vsb_store = []
			self.sub_vsb_speed = rospy.Subscriber('/vSBSpeed', PoseStamped, self.Speed_vSB_callback)

			
		# 	self.a = np.zeros((2,1))
		# 	self.b = np.zeros((2,1))
		# 	self.sub_a = rospy.Subscriber('/a', Point, self.a_callback)
		# 	self.sub_b = rospy.Subscriber('/b', Point, self.b_callback)
		# 	rospy.wait_for_message('/a', Point, timeout=None)



		self.obstacle_file = rospy.get_param('/bboat_plotter_node/obstacle_filepath')
		self.obstacles = self.Parse_Obstacle_File()



			
		self.vel_robot_RB = np.zeros((3,1))
		self.u_rob_store = []
		self.v_rob_store = []
		self.r_rob_store = []
		self.sub_vel_robot = rospy.Subscriber('/vel_robot_RB', Twist, self.Vel_Robot_callback)


		self.u1, self.u2 = 0, 0
		self.u1_store = []
		self.u2_store = []
		self.sub_cmd = rospy.Subscriber('/command', cmd_msg, self.Command_callback)


		# --- Matplotlib plotting

		# if self.flag_plotting:
		# 	self.fig = figure(1)
		# 	self.plot_wind = self.fig.add_subplot(111, aspect='equal')
		# 	if not self.mode_simu: 
		# 		self.fig2 = figure(2)
		# 		self.plot_wind2 = self.fig2.add_subplot(131, aspect='equal')


		# --- Init done
		rospy.loginfo('[PLOTTER] Plotter Node Iniitialized')

	def loop (self): 
		i=0
		while not rospy.is_shutdown():
			# rospy.loginfo('[PLOTTER] Plotter Loop')
			start_time = time.perf_counter()

			self.Plot_1()

			#self.Plot_2()

			# self.Plot_3()

			self.u1_store.append(self.u1)
			self.u2_store.append(self.u2)
			self.x_rob_store.append(self.pose_rob[0,0])
			self.y_rob_store.append(self.pose_rob[1,0])
			self.psi_rob_store.append(self.pose_rob[2,0])
			self.u_rob_store.append(self.vel_robot_RB[0,0])
			self.v_rob_store.append(self.vel_robot_RB[1,0])
			self.r_rob_store.append(self.vel_robot_RB[2,0])
			# if not self.flag_mission_traj:
			# 	self.u_vsb_store.append(self.vel_vsb_Rvsb[0,0])
			# 	self.v_vsb_store.append(self.vel_vsb_Rvsb[1,0])
			# 	self.r_vsb_store.append(self.vel_vsb_Rvsb[2,0])	
			# 	self.x_vsb_store.append(self.pose_vsb[0,0])
			# 	self.y_vsb_store.append(self.pose_vsb[1,0])
			# 	self.psi_vsb_store.append(self.pose_vsb[2,0])


			self.rate.sleep()
			# end_time = time.perf_counter()
			# print(f"{int(1/(end_time - start_time))} ms")


	def Pose_Robot_callback(self, msg):
		'''
			Parse robot pose message - x, y, psi in local lambert frame R0
		'''
		self.pose_rob = np.array([[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z]])

	
	def Pose_vSB_callback(self, msg):
		'''
			Parse Virtuasl Sailboat pose message - x, y, psi considered in local lambert frame R0
		'''
		self.pose_vsb = np.array([[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z]])

		

	def Command_callback(self, msg): 
		'''
		Parse command msg
		Turn forward and turning speed to 1100 - 2000 values to override
		'''
		self.u1, self.u2 = msg.u1.data, msg.u2.data

			

	def Speed_vSB_callback(self, msg):
		'''
			Parse Virtuasl Sailboat speed message - dx, dy, dpsi considered in local lambert frame R0
		'''
		self.vel_vsb_Rvsb = np.array([[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z]])

	def Control_Target_callback(self, msg):

		self.control_target = msg


	def Vel_Robot_callback(self, msg): 
		self.vel_robot_RB = np.array([[msg.linear.x], [msg.linear.y], [-msg.angular.z]])

	def a_callback(self, msg):
		self.a[0,0] = msg.x
		self.a[1,0] = msg.y

	def b_callback(self, msg):
		self.b[0,0] = msg.x
		self.b[1,0] = msg.y


	def Parse_Obstacle_File(self): 
		file = open(self.obstacle_file)
		lines = file.readlines()
		obstacles = []
		for line in lines: 
			tab = line.split(",")
			x = float(tab[0])
			y = float(tab[1])
			R = float(tab[2])
			F = float(tab[3])
			obstacles.append([x, y, R, F])
		file.close()
		print(obstacles)
		return obstacles


	def Plot_1(self): 
		# Plot en y abscisse, x ordonnée pour avoir North vers le haut
		figure(1)
		cla()
		# self.plot_wind.grid()
		# self.plot_wind.invert_xaxis()
		xlabel('y_0 : East')
		ylabel('x_0 : North')
		


		if self.flag_mission_traj: 
			x_rob, y_rob, psi_rob = self.pose_rob.flatten()
			pt = self.control_target
			# print(pt) 
			plot( pt.y, pt.x, 'ok')
			plot(self.traj.trajy, self.traj.trajx, '--k')
			plot(y_rob, x_rob, 'ob')
			plot([y_rob, y_rob+5*sin(psi_rob)], [x_rob, x_rob+5*cos(psi_rob)], 'b')


		else: 

			self.xmin, self.xmax = min(self.a[0,0], self.b[0,0]), max(self.a[0,0], self.b[0,0]) #potentiellement inverser ?
			self.ymin, self.ymax = min(self.a[1,0], self.b[1,0]), max(self.a[1,0], self.b[1,0])

			plot([self.a[1,0], self.b[1,0]], [self.a[0,0], self.b[0,0]], 'k')
			plot(self.a[1,0], self.a[0,0], '*g')
			plot(self.b[1,0], self.b[0,0],  '*r')

			plot(self.y_vsb_store, self.x_vsb_store, '--r')
			plot(self.pose_vsb[1,0], self.pose_vsb[0,0], 'or')
			plot([self.pose_vsb[1,0], self.pose_vsb[1,0]+5*sin(self.pose_vsb[2,0])], [self.pose_vsb[0,0], self.pose_vsb[0,0]+5*cos(self.pose_vsb[2,0])], 'r')
			pt = self.control_target
			plot( pt.y, pt.x, 'ok')

			x_rob, y_rob, psi_rob = self.pose_rob.flatten()
			plot(y_rob, x_rob, 'ob')
			plot([y_rob, y_rob+5*sin(psi_rob)], [x_rob, x_rob+5*cos(psi_rob)], 'b')
			plot(self.y_rob_store, self.x_rob_store, '--b')

			wind = 10*WIND_SPEED * np.array([[cos(WIND_ANGLE)], [sin(WIND_ANGLE)]])
			quiver(self.b[1,0]+3, self.b[0,0]+3, wind[1,0], wind[0,0])

			for i in range(0, len(self.obstacles), 1):
				obs = self.obstacles[i]
				plot(obs[1], obs[0], 'om', MarkerSize=obs[2])
				# add_patch(plt.Circle(obs[0:2], obs[2], color='k', fill=False))



		# x_limit = [self.ymin - 50, self.ymax + 50]
		# y_limit = [self.xmin - 50, self.xmax + 50]

		# xlim(x_limit)
		# ylim(y_limit)



		pause(.0001)
		show(block=False)

	def Plot_2(self): 
		figure(2)
		subplot(3,1,1)
		cla()
		grid()
		ylabel('u')
		if len(self.u_rob_store) > 1000: 
			if not self.flag_mission_traj: 
				plot(self.u_vsb_store[len(self.u_vsb_store)-1000:len(self.u_vsb_store)-1], '-r', label='VSB')
			plot(self.u_rob_store[len(self.u_vsb_store)-1000:len(self.u_vsb_store)-1], '-b', label='USV')
			plot(self.u1_store[len(self.u_vsb_store)-1000:len(self.u_vsb_store)-1], '-g', label='cmd')
		else: 
			if not self.flag_mission_traj:
				plot(self.u_vsb_store, '-r', label='VSB')
			plot(self.u_rob_store, '-b', label='USV')
			plot(self.u1_store, '-g', label='cmd')
		legend()

		subplot(3,1,2)
		cla()
		grid()
		if len(self.u_vsb_store) > 1000: 
			plot(self.v_vsb_store[len(self.v_vsb_store)-1000:len(self.v_vsb_store)-1], '-r')
			plot(self.v_rob_store[len(self.v_vsb_store)-1000:len(self.v_vsb_store)-1], '-b')
		else: 
			plot(self.v_vsb_store, '-r')
			plot(self.v_rob_store, '-b')
		ylabel('v')


		subplot(3,1,3)
		cla()
		grid()
		if len(self.u_vsb_store) > 1000: 
			plot(self.r_vsb_store[len(self.r_vsb_store)-1000:len(self.r_vsb_store)-1], '-r')
			plot(self.r_rob_store[len(self.r_vsb_store)-1000:len(self.r_vsb_store)-1], '-b')
			plot(self.u2_store[len(self.r_vsb_store)-1000:len(self.r_vsb_store)-1], '-g')
		else: 
			plot(self.r_vsb_store, '-r')
			plot(self.r_rob_store, '-b')
			plot(self.u2_store, '-g')
		ylabel('r')

		pause(0.001)
		show(block=False)

	def Plot_3(self): 
		# Plot en y abscisse, x ordonnée pour avoir North vers le haut
		figure(1)
		cla()
		# self.plot_wind.grid()
		# self.plot_wind.invert_xaxis()
		xlabel('y_0 : East')
		ylabel('x_0 : North')


		x_rob, y_rob, psi_rob = self.pose_rob.flatten()
		plot(y_rob, x_rob, 'ob')
		plot([y_rob, y_rob+5*sin(psi_rob)], [x_rob, x_rob+5*cos(psi_rob)], 'b')
		plot(self.y_rob_store, self.x_rob_store, '--b')


		for i in range(0, len(self.obstacles), 1):
			obs = self.obstacles[i]
			plot(obs[1], obs[0], 'om', MarkerSize=obs[2])
			# add_patch(plt.Circle(obs[0:2], obs[2], color='k', fill=False))



		x_limit = [- 5000, 5000]
		y_limit = [- 5000, 5000]

		xlim(x_limit)
		ylim(y_limit)


		pause(.0001)
		show(block=False)


if __name__ == '__main__':
	rospy.init_node('plotter')

	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		plotter = PlotterNode()
		if plotter.flag_plotting:
			plotter.loop()
	except rospy.ROSInterruptException: pass