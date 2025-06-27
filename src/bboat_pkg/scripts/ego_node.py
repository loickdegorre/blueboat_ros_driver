#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, Point, PoseStamped, Twist
from scipy.interpolate import CubicSpline

from bboat_pkg.srv import next_target_serv, next_target_servResponse, lambert_ref_serv, current_target_serv, path_description_serv, path_description_servResponse
from codac import *
from dataclasses import dataclass

from bboat_lib import deg_to_Lamb, sawtooth

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


@dataclass
class Object:
    x: float = 0.
    y: float = 0.
    theta: float = 0.
    v: float = 0.

class SepNavV(Sep):
	def __init__(self, boat, obstacles, r = 15, epsilon = 0.1):
		Sep.__init__(self, 2)

		self.boat = boat
		self.obstacles = obstacles
		self.r = r
		self.epsilon = epsilon
		
		self.x = VectorVar(3)
		self.So = [
			SepInverse(
				AnalyticFunction(
					[self.x],
						sqr((o.x-boat.x) - self.x[0]*self.x[2]) + 
						sqr((o.y-boat.y) - self.x[1]*self.x[2]) -
						# sqr((o.x-boat.x)*sin(boat.theta)+(o.y-boat.y)*cos(boat.theta)+(o.v*cos(o.theta-Interval.half_pi())*sin(boat.theta)+o.v*sin(o.theta-Interval.half_pi())*cos(boat.theta)-self.x[1])*self.x[2]) + 
						# sqr((o.x-boat.x)*cos(boat.theta)-(o.y-boat.y)*sin(boat.theta)+(o.v*cos(o.theta-Interval.half_pi())*cos(boat.theta)-o.v*sin(o.theta-Interval.half_pi())*sin(boat.theta)-self.x[0])*self.x[2]) -
						sqr(r)
				), Interval(-oo, 0)
			)
			for o in self.obstacles
		]

		self.Sp = [SepProj(s, [0, 1], IntervalVector([Interval(0, 150)]), self.epsilon) for s in self.So]
		if len(self.Sp) == 1:
			self.Su = self.Sp[0]
		else:
			self.Su = SepUnion(self.Sp[0], self.Sp[1])
			for s in self.Sp[2:]:
				self.Su = SepUnion(self.Su, s)

		self.v = VectorVar(2)
		self.f_fwd = AnalyticFunction([self.v], [self.v[1], self.v[0]])
		self.f_bwd = AnalyticFunction([self.v], [self.v[1], self.v[0]])
		self.S = SepTransform(self.Su, self.f_fwd, self.f_bwd)

	def separate(self, x):
		return self.S.separate(x)

class SepNav(Sep):
	def __init__(self, boat, obstacles, r = 15, epsilon = 0.1):
		Sep.__init__(self, 2)

		self.boat = boat
		self.obstacles = obstacles
		self.r = r
		self.epsilon = epsilon
		
		self.x = VectorVar(3)
		self.So = [
			SepInverse(
				AnalyticFunction(
					[self.x],
						sqr((o.x-boat.x) - self.x[0]*self.x[2]) + 
						sqr((o.y-boat.y) - self.x[1]*self.x[2]) -
						# sqr((o.x-boat.x)*sin(boat.theta)+(o.y-boat.y)*cos(boat.theta)+(o.v*cos(o.theta-Interval.half_pi())*sin(boat.theta)+o.v*sin(o.theta-Interval.half_pi())*cos(boat.theta)-self.x[1])*self.x[2]) + 
						# sqr((o.x-boat.x)*cos(boat.theta)-(o.y-boat.y)*sin(boat.theta)+(o.v*cos(o.theta-Interval.half_pi())*cos(boat.theta)-o.v*sin(o.theta-Interval.half_pi())*sin(boat.theta)-self.x[0])*self.x[2]) -
						sqr(r)
				), Interval(-oo, 0)
			)
			for o in self.obstacles
		]

		self.Sp = [SepProj(s, [0, 1], IntervalVector([Interval(0, 150)]), self.epsilon) for s in self.So]
		if len(self.Sp) == 1:
			self.Su = self.Sp[0]
		else:
			self.Su = SepUnion(self.Sp[0], self.Sp[1])
			for s in self.Sp[2:]:
				self.Su = SepUnion(self.Su, s)

		self.v = VectorVar(2)
		self.f_fwd = AnalyticFunction([self.v], [self.v[1]*cos(self.v[0]), self.v[1]*sin(self.v[0])])
		self.f_bwd = AnalyticFunction([self.v], [atan2(self.v[1], self.v[0]), sqrt(sqr(self.v[0])+sqr(self.v[1]))])
		self.S = SepTransform(self.Su, self.f_fwd, self.f_bwd)

	def separate(self, x):
		return self.S.separate(x)


class EgoNode(): 
	def __init__(self): 

		self.ref_lamb = np.zeros((2,1))

		# if not self.mission_simu: 
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
		self.ref_lamb[0,0] = resp.lambert_ref.x
		self.ref_lamb[1,0] = resp.lambert_ref.y


		self.mission_file = rospy.get_param('/bboat_egonav_node/waypoint_filepath')
		
		self.waypoint = self.Parse_Mission_File()

		self.obstacles_file = rospy.get_param('/bboat_egonav_node/obstacle_filepath')

		self.obstacles = self.Parse_Obstacles_File()

		self.desired_heading_publisher = rospy.Publisher('/desired_heading', Float32, queue_size=10)

		self.x = 0.0
		self.y = 0.0
		self.psi = 0.0
		self.vel_robot_RB = np.array([[0.0], [0.0], [0.0]])
		self.sub_heading = rospy.Subscriber('/pose_robot_R0', PoseStamped, self.heading_callback)
		self.vel_heading = rospy.Subscriber('/vel_robot_RB', Twist, self.velocity_callback)

		# Converting obstacles in lambert coordinates
		resp = self.client_ref_lambert(True)
		self.ref_lamb[0,0] = resp.lambert_ref.x
		self.ref_lamb[1,0] = resp.lambert_ref.y

		self.dT = 0.1
		self.rate = rospy.Rate(1/self.dT)

		self.fig_v = Figure2D("EgoNav", GraphicOutput.VIBES)
		self.fig_v.set_window_properties([50, 50], [800, 800])
		self.fig_v.set_axes(axis(0, Interval(-2, 2)), axis(1, Interval(-2, 2)))

		# --- Init done
		rospy.loginfo('[MISSION] Mission Node Start')

	def loop(self): 
		while not rospy.is_shutdown():
			# Computing the desired heading
			desired_heading = np.arctan2(self.waypoint[0, 1] - self.y, self.waypoint[0, 0] - self.x)
			heading = self.psi
			if heading > np.pi:
				heading -= 2*np.pi
			SV = SepNavV(Object(self.x, self.y, heading, self.vel_robot_RB[0, 0]), [Object(o[0], o[1], 0, 0) for o in self.obstacles])
			x = IntervalVector([Interval(-2, 2), Interval(-2, 2)])

			paving = pave(x, SV, 0.5)
			self.fig_v.draw_paving(paving)

			x0 = IntervalVector([Interval(-np.pi, np.pi), Interval(1, 1.1)])
			S = SepNav(Object(self.x, self.y, heading, self.vel_robot_RB[0, 0]), [Object(o[0], o[1], 0, 0) for o in self.obstacles])

			pav = pave(x0, S, 0.1)
			list_connected_subset = pav.connected_subsets(PavingInOut.outer_complem)
			found = False

			h = desired_heading
			diff = []
			L = []
			for connected_subset in list_connected_subset:
				if found:
					break
				boxes = connected_subset.boxes()
				for box in boxes:
					if box[0].contains(desired_heading):
						self.desired_heading_publisher.publish(Float32(desired_heading))
						found = True
						break
					else:
						L.append(box[0].lb())
						L.append(box[0].ub())
						diff.append(sawtooth(box[0].lb() - desired_heading))
						diff.append(sawtooth(box[0].ub() - desired_heading))
			if not found:
				if len(L) > 0:
					diff = np.asarray(diff)
					i = np.argmin(np.abs(diff))
					print(L)
					h = L[i]
					self.desired_heading_publisher.publish(Float32(h))
					rospy.logwarn(f"[EgoNav] psi_d={desired_heading} psi_*={h}")

				else:
					rospy.logwarn("[EGONAV] No solution found to avoid collisions!")

			self.rate.sleep()
	
	def new_loop(self): 
		while not rospy.is_shutdown():
			# Computing the desired heading
			desired_heading = np.arctan2(self.waypoint[0, 1] - self.y, self.waypoint[0, 0] - self.x)
			heading = self.psi
			if heading > np.pi:
				heading -= 2*np.pi
			SV = SepNavV(Object(self.x, self.y, heading, self.vel_robot_RB[0, 0]), [Object(o[0], o[1], 0, 0) for o in self.obstacles])
			x = IntervalVector([Interval(-2, 2), Interval(-2, 2)])

			paving = pave(x, SV, 0.5)
			self.fig_v.draw_paving(paving)

			x0 = IntervalVector([Interval(-3*np.pi, 3*np.pi), Interval(1, 1.1)])
			S = SepNav(Object(self.x, self.y, heading, self.vel_robot_RB[0, 0]), [Object(o[0], o[1], 0, 0) for o in self.obstacles])

			pav = pave(x0, S, 0.1)
			list_connected_subset = pav.connected_subsets(PavingInOut.outer)

			found = False
			h = desired_heading
			for connected_subset in list_connected_subset:
				if found:
					break
				
				box = connected_subset.box()
				if box.contains(desired_heading):
					if np.abs(box[0].lb() - desired_heading) < np.abs(box[0].ub() - desired_heading):
						h = box[0].lb()
					else:
						h = box[0].ub()
					found = True
					rospy.logwarn(f"[EgoNav] psi_d={desired_heading} psi_*={h}")
					break
			
			self.desired_heading_publisher.publish(Float32(desired_heading))
			self.rate.sleep()

	def old_loop(self): 
		while not rospy.is_shutdown():
			# Computing the desired heading
			desired_heading = np.arctan2(self.waypoint[0, 1] - self.y, self.waypoint[0, 0] - self.x)
			heading = self.psi
			if heading > np.pi:
				heading -= 2*np.pi
			S = SepNav(Object(self.x, self.y, heading, self.vel_robot_RB[0, 0]), [Object(o[0], o[1], 0, 0) for o in self.obstacles])
			x = IntervalVector([Interval(-np.pi, np.pi), Interval(1, 1.1)])

			paving = pave(x, S, 0.1)
			# self.fig.draw_paving(paving)
			list_connected_subset = paving.connected_subsets(PavingInOut.outer_complem)
			found = False

			self.ax.clear()
			self.ax.set_xlim(-np.pi, np.pi)
			self.ax.set_ylim(1, 1.1)
			self.ax.xaxis.set_inverted(True)
			for o in self.obstacles:
				self.ax.plot(sawtooth(self.psi - np.arctan2(o[1] - self.y, o[0] - self.x)), 1.05, "or")

			self.ax.plot(sawtooth(self.psi - np.arctan2(self.waypoint[0, 1] - self.y, self.waypoint[0, 0] - self.x)), 1.05, "og")

			L = []
			for connected_subset in list_connected_subset:
				if found:
					break
				boxes = connected_subset.boxes()
				for box in boxes:
					self.ax.add_patch(Rectangle((self.psi - box[0].lb(), box[1].lb()), box[0].ub()-box[0].lb(), box[1].ub()-box[1].lb()))
					if box[0].contains(desired_heading):
						self.desired_heading_publisher.publish(Float32(desired_heading))
						found = True
						break
					else:
						L.append(np.abs(sawtooth(box[0].lb() - desired_heading)))
						L.append(np.abs(sawtooth(box[0].ub() - desired_heading)))
			if not found:
				if len(L) > 0:
					h = np.min(np.asarray(L))
					self.desired_heading_publisher.publish(Float32(h))
					self.ax.plot(sawtooth(self.psi - h), 1.05, "o", color="crimson")

					rospy.loginfo(f"[EgoNav] Requested heading collide an obstacle! Safe heading {h}")
				else:
					rospy.logwarn("[EGONAV] No solution found to avoid collisions!")

			plt.pause(.0001)
			plt.show(block=False)
			self.rate.sleep()

	def heading_callback(self, msg):
		self.x = msg.pose.position.x
		self.y = msg.pose.position.y
		self.psi = msg.pose.position.z

	def velocity_callback(self, msg): 
		self.vel_robot_RB = np.array([[msg.linear.x], [msg.linear.y], [msg.angular.z]])

	def	Parse_Mission_File(self): 
		file = open(self.mission_file)
		lines = file.readlines()
		lat = []
		long = []
		px = []
		py = []

		for line in lines: 
			tab = line.split(",")
			lat.append(float(tab[0])) 
			long.append(float(tab[1]))

		for i in range(len(lat)):
			y,x = deg_to_Lamb(long[i], lat[i]) # renvoie y x
			# rospy.loginfo(f'[TRAJ] p {p}')
			px.append(x- self.ref_lamb[0,0]) 
			py.append(y- self.ref_lamb[1,0])
			
		# px = [0, 10, 10, 20]
		# py = [0, 0, 10, 20]
		# print(px, py)

		points = np.array([px,py]).transpose()

		file.close()
		return points
	
	def	Parse_Obstacles_File(self): 
		file = open(self.obstacles_file)
		lines = file.readlines()
		lat = []
		long = []
		px = []
		py = []

		for line in lines: 
			tab = line.split(",")
			lat.append(float(tab[0])) 
			long.append(float(tab[1]))

		for i in range(len(lat)):
			y,x = deg_to_Lamb(long[i], lat[i]) # renvoie y x
			# rospy.loginfo(f'[TRAJ] p {p}')
			px.append(x- self.ref_lamb[0,0]) 
			py.append(y- self.ref_lamb[1,0])
			
		# px = [0, 10, 10, 20]
		# py = [0, 0, 10, 20]
		# print(px, py)

		points = np.array([px,py]).transpose()

		file.close()
		return points

if __name__ == '__main__':
	rospy.init_node('mission')

	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		mission = EgoNode()
		mission.loop()
	except rospy.ROSInterruptException: pass
