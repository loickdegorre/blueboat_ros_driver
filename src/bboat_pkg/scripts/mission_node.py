#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose, Point


from bboat_pkg.srv import next_target_serv, next_target_servResponse, lambert_ref_serv, current_target_serv
from lib.bboat_lib import *

class MissionNode(): 
	def __init__(self): 

		self.mission_file = rospy.get_param('/bboat_mission_node/filepath')
		
		self.mission_points = self.Parse_Mission_File()

		self.current_target = self.mission_points[0,:]

		self.next_point_index = 0


		self.ref_lamb = np.zeros((2,1))


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


		rospy.Service('/next_target', next_target_serv, self.Next_Target_callback)

		rospy.Service('/current_target', current_target_serv, self.Current_Target_callback)

		#print(f'mission : first ref lambert = {self.ref_lamb[0,0]} | {self.ref_lamb[1,0]}')

		# --- Init done
		rospy.loginfo('[MISSION] Mission Node Start')



	def loop(self): 
		while not rospy.is_shutdown():
			rospy.sleep(1)

	def Next_Target_callback(self, req): 
		rospy.loginfo('[MISSION] Next target service callback')
		next_point = Pose()
		if self.next_point_index < len(self.mission_points): 
			rospy.loginfo('[MISSION] Next point, continue mission')
			continue_mission = True
			lat, lon = self.mission_points[self.next_point_index, 0], self.mission_points[self.next_point_index, 1]


			next_point_lambert = deg_to_Lamb(lon, lat)

			#print(f'mission : next point lambert {next_point_lambert[0]} | {next_point_lambert[1]}')

			resp = self.client_ref_lambert(True)
			self.ref_lamb[0,0] = resp.lambert_ref.x
			self.ref_lamb[1,0] = resp.lambert_ref.y


			#print(f'mission : next point lambert apres {next_point_lambert[0]- self.ref_lamb[0,0]} | {next_point_lambert[1]- self.ref_lamb[1,0]}')

			next_point.position.x = next_point_lambert[0]- self.ref_lamb[0,0]
			next_point.position.y = next_point_lambert[1]- self.ref_lamb[1,0]	

			self.current_target = np.array([[next_point.position.x], [next_point.position.y]])
			self.next_point_index +=1
		else:
			rospy.loginfo('[MISSION] Last point, end mission')
			continue_mission = False

		response = next_target_servResponse()
		response.next_trgt_pose = next_point
		response.continuing_mission = continue_mission	
		return response

	def	Parse_Mission_File(self): 
		file = open(self.mission_file)
		lines = file.readlines()
		px = []
		py = []
		for line in lines: 
			tab = line.split(",")
			px.append(float(tab[0]))
			py.append(float(tab[1]))
		points = np.array([px,py])
		return points.transpose()

	def Ref_Lambert_callback(self, msg):
		self.ref_lamb[0,0] = msg.x
		self.ref_lamb[1,0] = msg.y

	def Current_Target_callback(self, req):
		trgt = Point()
		trgt.x, trgt.y = self.current_target[0,0], self.current_target[1,0]
		return trgt


if __name__ == '__main__':
	rospy.init_node('mission')

	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		mission = MissionNode()
		mission.loop()
	except rospy.ROSInterruptException: pass
