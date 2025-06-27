#!/usr/bin/env python3

import rospy

import numpy as np
import math
from pyproj import Proj, transform
from pyproj import Transformer

from std_msgs.msg import Float64, Float32
from geometry_msgs.msg import PoseStamped, Point, Twist

from bboat_pkg.msg import cmd_msg, mode_msg
from bboat_pkg.srv import mode_serv, mode_servResponse, current_target_serv, gain_serv, gain_servResponse, path_description_serv, traj_serv

from bboat_lib import *
from command_lib import *

import matplotlib.pyplot as plt

import time

from dockLib import *

epsx, epsy = 1,0
## H - Model_based_guidance - ADRC_guidance - LOS_TT - State_Extent
command_type = "LOS_TT" #Select command law 

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

		self.flag_mission_traj = rospy.get_param('/bboat_controller_node/mission_traj')

		# ---
		self.u_SB_thresh = 0.08
		self.dT = 0.02
		self.rate = rospy.Rate(1/self.dT)

		self.pose_robot = np.zeros((3,1))

		self.pose_vsb = np.zeros((3,1))

		self.speed_vsb = np.zeros((3,1))

		self.state_error_integral = np.zeros((3,1))

		self.i = 0

		self.V = np.zeros((2,1)) # ADRC virtual input V = [vx, vy]
		self.Zx = np.zeros((2,1)) # Observer state [x, epsx]
		self.Zy = np.zeros((2,1)) # Observer state [y, epsy]

		self.firstTime = True # used in DOCK2D mission to know if its the first pass in the loop
		self.dubin_path = []
		self.K,self.K1,self.Kdy1,self.K10= 1.,10,1,10
		# State extend
		self.u_SE = 0


		# --- Subs

		self.mode_msg = None
		self.sub_mode = rospy.Subscriber('/modepub', mode_msg, self.Mode_callback)
		rospy.wait_for_message('/modepub', mode_msg,  timeout=None)

		# rospy.logwarn("HERE")

		self.sub_pose_robot = rospy.Subscriber('/pose_robot_R0', PoseStamped, self.Pose_Robot_callback)
		rospy.wait_for_message('/pose_robot_R0', PoseStamped, timeout=None)

		self.sub_vsb_pose = rospy.Subscriber('/vSBPosition', PoseStamped, self.Pose_vSB_callback)

		self.sub_vsb_speed = rospy.Subscriber('/vSBSpeed', PoseStamped, self.Speed_vSB_callback)

		self.vel_robot_RB = np.zeros((3,1))
		self.sub_vel_robot = rospy.Subscriber('/vel_robot_RB', Twist, self.Vel_Robot_callback)

		self.psi_ref = 0
		self.sub_desired_heading = rospy.Subscriber('/desired_heading', Float32, self.Desired_Heading_callback)


		rospy.wait_for_service('/lambert_ref')
		connected = False
		while not connected:
			try:
				self.client_ref_lambert = rospy.ServiceProxy('/lambert_ref', lambert_ref_serv)
				connected = True
			except rospy.ServiceException as exc:
				rospy.logwarn(f'[MISSION] Lambert ref service cannot be reached - {str(exc)}')
				connected = False
		resp = self.client_ref_lambert(True)
		self.ref_lamb = np.array([[resp.lambert_ref.x], [resp.lambert_ref.y]])

		self.inte_1 = 0

		# --- Pubs
		self.pub_target = rospy.Publisher('/control_target', Point, queue_size=10)

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

		if self.flag_mission_traj: 
			rospy.wait_for_service('/get_traj')
			self.client_traj = rospy.ServiceProxy('/get_traj', traj_serv)
			self.traj = self.client_traj()
			self.i = 0

		# figure()
		# plot(self.traj.trajx)
		# plot(self.traj.trajy)
		# grid()
		# figure()
		# plot(self.traj.trajdx)
		# plot(self.traj.trajdy)
		# grid()
		# show(block=True)


		# self.control_target = np.array([[self.traj.trajx[0]], [self.traj.trajy[0]], [self.traj.trajdx[0]], [self.traj.trajdy[0]]])
		self.control_target = np.zeros((4,1))

		
		
		# Control variables

		self.time = 0
		# self.path_points = get_path_points()
		# self.error_state = initiate_error_state(self.pose_robot, self.vel_robot_RB, 0, self.path_points)
		self.last_u1, self.last_u2 = 0, 0

		self.s = 0


		# --- Init done
		rospy.loginfo('[CONTROLLER] Controller node Initialized')

	def loop(self): 

		while not rospy.is_shutdown():

			start_time = time.perf_counter()

			# mode = self.client_mode(True)
			mode = self.mode_msg

			time_subscribers1 = time.perf_counter()

			# gains = self.client_gains(True)

			time_subscribers2 = time.perf_counter()


			if mode.mode == "AUTO":	
				x_rob, y_rob, psi_rob = self.pose_robot.flatten()
				x_SB, y_SB, psi_SB = self.pose_vsb.flatten()
				dx_SB, dy_SB, r_SB = self.speed_vsb.flatten() 
				u1 = 0.0
				u2 = 0.0

				if mode.mission == "VSB" :
					# ---
					# u1 - Forward speed
					kp = 1.5
					ki = 0
					# Error in robot frame
					err_x_in_Rrob = (x_SB-x_rob)*math.cos(psi_rob) + (y_SB-y_rob)*math.sin(psi_rob)
					err_y_in_Rrob = -(x_SB-x_rob)*math.sin(psi_rob) + (y_SB-y_rob)*math.cos(psi_rob)

					# print(f'err {err_x_in_Rrob}')
					uSB_RB = dx_SB*math.cos(psi_rob) + dy_SB*math.sin(psi_rob)
					# u1 = (uSB_RB + kp*err_x_in_Rrob)
					
					# ---
					# u2 - Turning speed
					# Kp = gains.kp_2.data
					Kp = 1
					u_SB =  dx_SB*math.cos(psi_SB) + dy_SB*math.sin(psi_SB)
					v_SB = -dx_SB*math.sin(psi_SB) + dy_SB*math.cos(psi_SB)

					# if sqrt(err_x_in_Rrob**2+err_y_in_Rrob**2) > 3:
					# 	# print("1")
					# 	self.inte_1 = self.inte_1 + err_x_in_Rrob*self.dT

					# 	u1 = kp*err_x_in_Rrob + ki*self.inte_1 # u_SB en feedforward ?
					# 	# u1 = uSB_RB

					# 	psi_des = math.atan2((y_SB-y_rob), (x_SB-x_rob))
					# 	if psi_des <0:
					# 		psi_des = psi_des + 2*pi

					# elif u_SB > self.u_SB_thresh: 
					# 	# print("2")
					# 	u1 = u_SB#uSB_RB

					# 	psi_speed = math.atan2(v_SB, u_SB)
					# 	if psi_speed <0:
					# 		psi_speed = psi_speed + 2*pi
					# 	psi_des = psi_SB + psi_speed

					# else: 
					# 	# print("3")
					# 	u1 = 0
					# 	psi_des = psi_SB

					# #print(f'psi_des {psi_des} r_sb {r_SB} psi_rob {psi_rob}')
					# # u1 = 0
					# u2 = r_SB + Kp*sawtooth(psi_des - psi_rob)

					# Approche Mat.H Tracking x_SB, y_SB
					target = np.array([[x_SB], [y_SB], [dx_SB], [dy_SB]])
					u1, u2, self.state_error_integral = command_h(self.pose_robot, target, self.state_error_integral, self.dT, epsx, epsy)
					# 
					# Ajouter la possibilité de passer en norm(u+V)/COG une fois que le robot à converger ? 



				elif mode.mission == "CAP":
					u1 = 1.

					if self.psi_ref <0: 
						self.psi_ref = 2*np.pi +self.psi_ref

					# print(self.psi_ref, psi_rob)
					Kp = .95
					u2 = Kp*sawtooth(self.psi_ref - psi_rob)
					
					# rospy.loginfo(f'[CONTROLLER] CAP desired = {psi_des} robot = {psi_rob}')
					# print(f'des {psi_des*180/math.pi} rob {psi_rob*180/math.pi} u2 {u2}')


				elif mode.mission == "TRAJ":
					# For Matrix H control (Trajectory Following):
					if(command_type == "H"):
						# Update target in trajectory tracking
						if self.i < len(self.traj.trajx): 
							target = np.array([[self.traj.trajx[self.i]], [self.traj.trajy[self.i]], [self.traj.trajdx[self.i]], [self.traj.trajdy[self.i]]])
							self.i += 1
						else: 
							self.i = 0
							rospy.loginfo('[CONTROLLER] Trajectory completed')
						# print(target)
						
						# print(f'avant {self.control_target}')
						# target = update_target_pathtracking(self.control_target, self.pose_robot, self.vel_robot_RB, self.traj, self.dT)
						# print(f'apres {target}')
						# ---
						self.control_target = target
						u1, u2, self.state_error_integral = command_h(self.pose_robot, self.control_target, self.state_error_integral, self.dT, epsx, epsy)
					
					elif(command_type == "Model_based_guidance"): 
						# Update target in trajectory tracking
						if self.i < len(self.traj.trajx): 
							target = np.array([[self.traj.trajx[self.i]], [self.traj.trajy[self.i]], [self.traj.trajdx[self.i]], [self.traj.trajdy[self.i]]])
							self.i += 1
						else: 
							self.i = 0
							rospy.loginfo('[CONTROLLER] Trajectory completed')
						
						self.control_target = target
						u1, u2, self.state_error_integral = command_MBG(self.pose_robot, self.control_target, self.state_error_integral, self.dT)
					
					elif(command_type == "ADRC_guidance"): 
						# Update target in trajectory tracking
						if self.i < len(self.traj.trajx): 
							target = np.array([[self.traj.trajx[self.i]], [self.traj.trajy[self.i]], [self.traj.trajdx[self.i]], [self.traj.trajdy[self.i]]])
							self.i += 1
						else: 
							self.i = 0
							rospy.loginfo('[CONTROLLER] Trajectory completed')
						
						self.control_target = target
						u1, u2, self.V, self.Zx, self.Zy = command_ADRCG(self.pose_robot, self.vel_robot_RB, self.control_target, self.dT, self.V, self.Zx, self.Zy, self.last_u1, self.last_u2)
					
					elif(command_type == "LOS_TT"): 
						# Update target in trajectory tracking
						if self.i < len(self.traj.trajx): 
							target = np.array([[self.traj.trajx[self.i]], [self.traj.trajy[self.i]], [self.traj.trajdx[self.i]], [self.traj.trajdy[self.i]]])
							self.i += 1
						else: 
							self.i = 0
							rospy.loginfo('[CONTROLLER] Trajectory completed')
						
						self.control_target = target
						u1, u2, self.state_error_integral = command_LOSTT(self.pose_robot, self.control_target, self.state_error_integral, self.dT)
					
					elif(command_type == "State_Extent"): 
						# Update target in trajectory tracking
						if self.i < len(self.traj.trajx): 
							target = np.array([[self.traj.trajx[self.i]], [self.traj.trajy[self.i]], [self.traj.trajdx[self.i]], [self.traj.trajdy[self.i]]])
							self.i += 1
						else: 
							self.i = 0
							rospy.loginfo('[CONTROLLER] Trajectory completed')
						
						self.control_target = target
						u1, u2, self.state_error_integral = command_State_Extent(self.pose_robot, self.vel_robot_RB , self.control_target, self.state_error_integral, self.dT, self.u_SE)
						self.u_SE = u1

					# For Path Following AUV
					# if(command_type == "AUV"):

					# 	# vitesse = np.array([[self.last_u1],[0],[self.last_u2]]) #seulement pour mode simulation
					# 	vitesse = self.vel_robot_RB

					# 	u_target = 1

					# 	u1, u2, ds, xs, ys= command_auv_model(vitesse, self.error_state, u_target, self.path_points)						

					# 	self.control_target = np.array([[xs],[ys],[0]])

					# 	self.error_state = update_error_state(self.error_state, vitesse, self.pose_robot, ds, self.dT, self.path_points)

					# # For simple LOS
					# if(command_type == "LOS"):

					# 	u_target = 1

					# 	target = get_target_los(self.pose_robot, self.path_points)
					# 	u1, u2, self.state_error_integral = command_los(self.pose_robot, target, u_target,self.state_error_integral, self.dT)

					# 	trg = Point()
					# 	trg.x = target[0]
					# 	trg.y = target[1]
					# 	trg.z = 0

					# 	self.control_target = np.array([target[0][0], target[1][0], 0])

					# if(command_type == "FBLIN" or command_type == "SLID"):
						
					# 	# vitesse = np.array([[self.last_u1],[0],[self.last_u2]]) #seulement pour mode simulation
					# 	vitesse = self.vel_robot_RB
					# 	start_time_traj = time.perf_counter()

					# 	target, d_target, dd_target, self.s = get_target_traj(self.s, self.dT, self.path_points)
					# 	self.control_target = np.array([target[0][0], target[1][0], 0])						# print(self.s)
					# 	start_time_com = time.perf_counter()

					# 	target = target[:2]
					# 	d_target = d_target[:2]
					# 	dd_target = dd_target[:2] 
					# 	dn_state = (J(self.pose_robot) @ vitesse)[:2]

					# 	self.state_error_integral += (target - self.pose_robot[:2]) * self.dT

					# 	if(command_type == "FBLIN"):
					# 		k = dd_target + 0 * (d_target - dn_state) + 0.5*(target - self.pose_robot[:2]) + 0 * self.state_error_integral
					# 	elif(command_type == "SLID"):
					# 		k = 1*np.sign((d_target - dn_state) + 2*(target - self.pose_robot[:2]))

					# 	u1, u2 = command_fblin(self.pose_robot, vitesse, k, self.dT)

					# 	# print(self.last_u1)
					# 	end_time_com = time.perf_counter()

					# 	# self.control_target = target
				
				elif mode.mission == "DOCK2Dtfdefini":
					TURNING_RADIUS = 7.5
					
					if self.firstTime == True: 
						# --- Creation of Dubin Path
						tf = 30.0 #TODOOOOOOOOOOOO
						v_target, v_robot = 1.0,2.0 #TODOOOOOOOOOOOO



						# target_x0, target_y0, target_psi0 = -2.0, -22.0, 3*np.pi/2 # TODOOOOOOOOOOOOOO
						# x,y = deg_to_Lamb(lon, lat)	
						# print(lon, lat)
						#start
						lat_trg, lon_trg = 48.197426, -3.013215
						y_temp,x_temp = deg_to_Lamb(lon_trg, lat_trg)

						target_x0 = x_temp - self.ref_lamb[0, 0]
						target_y0 = y_temp - self.ref_lamb[1, 0]

						#arrivee
						lat_trgf, lon_trgf = 48.197489, -3.01523208
						y_temp,x_temp = deg_to_Lamb(lon_trgf, lat_trgf)

						target_xf = x_temp - self.ref_lamb[0, 0]
						target_yf = y_temp - self.ref_lamb[1, 0]

						target_psi0 = np.arctan2(target_yf - target_y0, target_xf - target_x0)  #3*np.pi/2

						final_pose = np.array([[target_x0 + v_target*tf*cos(target_psi0)], [target_y0 + v_target*tf*sin(target_psi0)], [target_psi0]]) # pose.position.z = psi
						self.dubin_path = get_dubins_path_callback(self.pose_robot, final_pose, TURNING_RADIUS)

						self.firstTime = False
						plt.figure("coucou")
						X = [self.dubin_path[k].x for k in range(len(self.dubin_path))]
						Y = [self.dubin_path[k].y for k in range(len(self.dubin_path))]

						plt.plot(X,Y)
						plt.axis("equal")
						plt.show(block=False)
						pause(0.001)

				elif mode.mission == "DOCK2D":
					TURNING_RADIUS = 6
					modeChosen = ['L','S','L']
					
					if self.firstTime == True: 
						# --- Creation of Dubin Path
						
						v_target, v_robot = 1.0,1.5 #TODOOOOOOOOOOOO



						# target_x0, target_y0, target_psi0 = -2.0, -22.0, 3*np.pi/2 # TODOOOOOOOOOOOOOO
						# x,y = deg_to_Lamb(lon, lat)	
						# print(lon, lat)
						#start
						lat_trg, lon_trg = 48.19761513, -3.013166
						y_temp,x_temp = deg_to_Lamb(lon_trg, lat_trg)

						target_x0 = x_temp - self.ref_lamb[0, 0]
						target_y0 = y_temp - self.ref_lamb[1, 0]

						#arrivee
						lat_trgf, lon_trgf = 48.19757937, -3.01426649
						y_temp,x_temp = deg_to_Lamb(lon_trgf, lat_trgf)

						target_xf = x_temp - self.ref_lamb[0, 0]
						target_yf = y_temp - self.ref_lamb[1, 0]

						x0_robot, y0_robot,robot_psi0 = self.pose_robot.flatten()
						X0 = np.array([[x0_robot], [y0_robot]])
						Y0 = np.array([[target_x0],[target_y0]])

						target_psi0 = np.arctan2(target_yf - target_y0, target_xf - target_x0)  #3*np.pi/2
						tf, d1,d2,d3 = computeTfLSL(X0, Y0, robot_psi0, target_psi0, TURNING_RADIUS, v_robot, v_target, 0.0)
						final_pose = np.array([[target_x0 + v_target*tf*cos(target_psi0)], [target_y0 + v_target*tf*sin(target_psi0)], [target_psi0]]) # pose.position.z = psi
						#self.dubin_path = get_dubins_path_callback(self.pose_robot, final_pose, TURNING_RADIUS)
						print(f'tf : {tf}')

						xf, yf, psif = final_pose.flatten()
						self.dubin_path = get_dubins_pathCS(self.pose_robot, final_pose, TURNING_RADIUS, modeChosen)
						
						self.firstTime = False
						plt.figure("coucou")
						X = [self.dubin_path[k].x for k in range(len(self.dubin_path))]
						Y = [self.dubin_path[k].y for k in range(len(self.dubin_path))]

						plt.plot(X,Y)
						plt.axis("equal")
						plt.show(block=False)
						pause(0.001)
						


					
					# try:
					u1 = 1.5
					# Compute U
					x, y, psi = self.pose_robot.flatten()
					B = np.array([[cos(psi), 0], 
									[sin(psi), 0], 
									[0, 1]])
					
					X0 = B @ np.array([[u1],
										[u2]])
					x0,y0,psi0 = X0.flatten()
					V_0 = np.array([[x0],
									[y0]])
					W_0 = np.array([[psi0]])
					
					lievre, lievreCc,lievreS = interrogation_chemin(self.s,self.dubin_path)

					smax = max([p.s for p in self.dubin_path])
					lievreX,lievreY,lievrePsi = lievre.flatten()
					E = lievre-self.pose_robot
					errX, errY, theta = E.flatten()
					theta = sawtooth1D(theta, "pi")
					errXY = np.array([[errX],
									[errY]])
					R = np.array([[np.cos(lievrePsi), -np.sin(lievrePsi)],
								[np.sin(lievrePsi),  np.cos(lievrePsi)]])
					
					errX_Fresnet = inv(R) @ errXY
					
					s1, y1 = errX_Fresnet.flatten() 
					sp = u1 * cos(theta) - self.K1 * s1
					lievreXp_Fresnet = np.array([[sp], [0]])
					dot_X_e_F = lievreXp_Fresnet - inv(R) @ V_0

					s1p,y1p = dot_X_e_F.flatten()
					psip_lievre = lievreCc * sp

					delta = -np.arctan(self.Kdy1 * y1)
					deltaPrime = -self.Kdy1 / (1 + (self.Kdy1 * y1) ** 2)
					deltaP = deltaPrime * y1p

					Err_Ang = sawtooth1D(delta - theta, "pi")
					if Err_Ang > np.pi: Err_Ang -= 2 * np.pi
					elif Err_Ang < -np.pi: Err_Ang += 2 * np.pi
						
					dot_Theta_Control = deltaP + self.K * Err_Ang
					dot_psi = psip_lievre - dot_Theta_Control
					u2 = sawtooth1D(dot_psi, modulo = "pi")

					if sp < 0: sp = 0
					self.s = self.s + sp*self.dT

					# except Exception as e:
					# 	rospy.loginfo(f"Erreur : {e}")
					# 	u1 = 0.
					# 	u2 = 0.


				start_after_com = time.perf_counter()

				self.time += self.dT

				# print(u1, u2)
				# ---
				# Build and publish command message
				if abs(u1) > MAX_SPEED_FWRD:
					u1 = np.sign(u1)*MAX_SPEED_FWRD
				if abs(u2) > MAX_SPEED_TURN:
					u2 = np.sign(u2)*MAX_SPEED_TURN	

				# Favore rotation -> don't move forward when turning is required	
				if abs(u2) > 0.1:
					# print('sat rota')
					u1 = 0

				self.last_u1, self.last_u2 = u1, u2

				cmd = cmd_msg()
				cmd.u1 = Float64(u1)
				cmd.u2 = Float64(u2)

				self.pub_cmd.publish(cmd)

				self.pub_target.publish(Point(self.control_target[0], self.control_target[1], self.control_target[2]))


			self.rate.sleep()
			end_time = time.perf_counter()

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

	def Vel_Robot_callback(self, msg): 
		self.vel_robot_RB = np.array([[msg.linear.x], [msg.linear.y], [msg.angular.z]])

	def Current_Target_callback(self, req):
		trgt = Point()

		trgt.x, trgt.y = self.control_target[0, 0], self.control_target[1, 0]

		return trgt
	
	def Mode_callback(self, msg):
		self.mode_msg = msg
		
	def Desired_Heading_callback(self, msg): 
		self.psi_ref = msg.data


if __name__ == '__main__':
    rospy.init_node('controller')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        controller = ControllerNode()
        controller.loop()
    except rospy.ROSInterruptException: pass