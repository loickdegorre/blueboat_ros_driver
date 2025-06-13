#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose, Point
from scipy.interpolate import CubicSpline

from bboat_pkg.srv import next_target_serv, next_target_servResponse, lambert_ref_serv, current_target_serv, path_description_serv, path_description_servResponse
from lib.bboat_lib import *
import codac
from dataclasses import dataclass


@dataclass
class Object:
    x: float = 0.
    y: float = 0.
    theta: float = 0.
    v: float = 0.


class SepNav(Sep):
	def __init__(self, boat, obstacles, r = 1, epsilon = 0.1):
		Sep.__init__(self, 2)

		self.boat = boat
		self.obstacles = obstacles
		self.r = r
		self.epsilon = epsilon
		
		self.v = VectorVar(2)
		self.f_fwd = AnalyticFunction([self.v], [self.v[0]*sin(self.v[1]), self.v[0]*cos(self.v[1])])
		self.f_bwd = AnalyticFunction([self.v], [atan2(self.v[0], self.v[1]), sqrt(sqr(self.v[0])+sqr(self.v[1]))])

		self.x = VectorVar(3)
		self.So = [
			SepInverse(
				(
					[self.x],
					[
						sqr((o.x-boat.x)*cos(boat.theta)-(o.y-boat.y)*sin(boat.theta)+(o.v*cos(o.theta-pi/2)*cos(boat.theta)-o.v*sin(o.theta-pi/2)*sin(boat.theta)-self.x[0])*self.x[2]) +
						sqr((o.x-boat.x)*sin(boat.theta)+(o.y-boat.y)*cos(boat.theta)+(o.v*cos(o.theta-pi/2)*sin(boat.theta)+o.v*sin(o.theta-pi/2)*cos(boat.theta)-self.v[1])*self.v[2]) - 
						sqr(r)
					]
				), Interval(-oo, 0))
			for o in self.obstacles
		]

		self.S = SepTransform(SepUnion([SepProj(self.So, Interval(0, oo), self.epsilon) for s in self.s]), self.f_fwd, self.f_bwd)

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


		self.mission_file = rospy.get_param('/bboat_ego_node/mission_filepath')
		
		self.mission_points = self.Parse_Mission_File()

		self.obstacle_file = rospy.get_param('/bboat_ego_node/obstacle_filepath')

		self.obstacles = self.Parse_Obstacles_File()


		self.desired_heading_publisher = rospy.Publisher('/desired_heading', Float64, queue_size=10)

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

		self.dT = 0.02

		# --- Init done
		rospy.loginfo('[MISSION] Mission Node Start')

	def loop(self): 
		while not rospy.is_shutdown():
			# Computing the desired heading
			desired_heading = np.arctan2(self.current_target[1] - self.y, self.current_target[0] - self.x)

			S = SepNav(Object(self.x, self.y, self.psi, self.vel_robot_RB[0, 0]), [Object(o[0], o[1], 0, 0) for o in self.obstacles])
			x = IntervalVector([Interval(-np.pi, np.pi), Interval(self.vel_robot_RB[0, 0]).inflate(0.1)])
			S.separate(x)

			paving = S.paving(x, 0.1)
			list_connected_subset = paving.connected_subset()
			found = False
			for connected_subset in list_connected_subset:
				if connected_subset.contains(IntervalVector([Interval(desired_heading), Interval(self.vel_robot_RB[0, 0]).inflate(0.1)])):
					self.desired_heading_publisher.publish(Float64(desired_heading))
					found = True
					break

			if not found:
				L = []
				for connected_subset in list_connected_subset:
					boxes = connected_subset.get_boxes()
					for box in boxes:
						L.append(2*np.arctan(np.tan((box[0].lb()-desired_heading)/2)))
						L.append(2*np.arctan(np.tan((box[0].ub()-desired_heading)/2)))
				heading = np.min(np.asarray(L))
				self.desired_heading_publisher.publish(Float64(heading))

			rospy.sleep(self.dT)

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
		t = []
		dx = [] #cartesian speed x
		dy = [] #cartesian speed y

		for line in lines: 
			tab = line.split(",")
			lat.append(float(tab[0])) 
			long.append(float(tab[1]))
			t.append(float(tab[2]))
			dx.append(float(tab[3]))
			dy.append(float(tab[4]))

		for i in range(len(lat)):
			y,x = deg_to_Lamb(long[i], lat[i]) # renvoie y x
			# rospy.loginfo(f'[TRAJ] p {p}')
			px.append(x- self.ref_lamb[0,0]) 
			py.append(y- self.ref_lamb[1,0])
			
		# px = [0, 10, 10, 20]
		# py = [0, 0, 10, 20]
		# print(px, py)

		points = np.array([px,py,t,dx,dy]).transpose()

		file.close()
		return points
	
	def	Parse_Obstacles_File(self): 
		file = open(self.obstacles_file)
		lines = file.readlines()
		lat = []
		long = []
		px = []
		py = []
		t = []
		dx = [] #cartesian speed x
		dy = [] #cartesian speed y

		for line in lines: 
			tab = line.split(",")
			lat.append(float(tab[0])) 
			long.append(float(tab[1]))
			t.append(float(tab[2]))
			dx.append(float(tab[3]))
			dy.append(float(tab[4]))

		for i in range(len(lat)):
			y,x = deg_to_Lamb(long[i], lat[i]) # renvoie y x
			# rospy.loginfo(f'[TRAJ] p {p}')
			px.append(x- self.ref_lamb[0,0]) 
			py.append(y- self.ref_lamb[1,0])
			
		# px = [0, 10, 10, 20]
		# py = [0, 0, 10, 20]
		# print(px, py)

		points = np.array([px,py,t,dx,dy]).transpose()

		file.close()
		return points

if __name__ == '__main__':
	rospy.init_node('mission')

	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		mission = EgoNode()
		mission.loop()
	except rospy.ROSInterruptException: pass
