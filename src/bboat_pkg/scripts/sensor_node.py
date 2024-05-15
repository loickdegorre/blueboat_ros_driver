#!/usr/bin/env python3

import rospy
from math import pi, cos, sin
import numpy as np

from geometry_msgs.msg import PoseStamped, Point, TransformStamped
import tf

from std_msgs.msg import Float64

from sensor_msgs.msg import NavSatFix

# from pyproj import Transformer

from bboat_pkg.srv import reset_lamb_serv, lambert_ref_serv

from lib.bboat_lib import *

lat_standard, lon_standard = 48.4180211,-4.4721604 # Bat M ENSTA


class SensorNode():
	'''
		Sensor node
		Subscribers
			- Mavros Global position 
			- Mavros Heading
		Publishers
			- Robot pose - x, y, psi in local Lambert frame
			- Lambert ref
		Serivce Provider
			- Reset Lambert reference
			- Send Lambert reference

	'''
	def __init__(self):

		self.flag_set_ref_lamb = True
		self.ref_lamb = np.zeros((2,1))
		self.pose_R0=np.zeros((3,1)) # x, y, psi local frame
		self.vel_R0=np.zeros((3,1)) #dx, dy, dpsi local frame

		# --- Subs
		self.sub_pose_global = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.Pose_Global_callback)
		rospy.wait_for_message('/mavros/global_position/global', NavSatFix, timeout=None)
		self.sub_heading = rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self.Heading_callback)

		
		# --- Pubs
		self.pub_pose_R0 = rospy.Publisher('/pose_robot_R0', PoseStamped, queue_size=10)

		# --- Service
		rospy.Service('/reset_lamb_ref', reset_lamb_serv, self.Reset_Lamb_callback)
		rospy.Service('/lambert_ref', lambert_ref_serv, self.Lambert_Ref_callback)


		# --- TF Broadcaster robot state
		self.tf_broadcaster = tf.TransformBroadcaster()

		# --- Init done
		rospy.loginfo('[SENSOR] Sensor node Start')

	def loop(self): 
		while not rospy.is_shutdown():
			# rospy.loginfo('[SENSOR] Heartbeat')
			# ---
			# Build and publish pose msg
			pose_msg = PoseStamped()
			pose_msg.header.frame_id = 'ref_lamb'
			pose_msg.header.stamp = rospy.Time.now()
			pose_msg.pose.position.x = self.pose_R0[0,0]
			pose_msg.pose.position.y = self.pose_R0[1,0]
			pose_msg.pose.position.z = self.pose_R0[2,0] #psi
			# print(f'cap : {self.pose_R0[2,0]}')
			self.pub_pose_R0.publish(pose_msg)


			t = TransformStamped()
			t.header.stamp = rospy.Time.now()
			t.header.frame_id = 'map_ned'
			t.child_frame_id = 'ref_lamb'
			t.transform.translation.x = self.ref_lamb[0]
			t.transform.translation.y = self.ref_lamb[1]
			t.transform.translation.z = 0.0
			q = quaternion_from_euler(0,0,0)
			t.transform.rotation.x = q[0]
			t.transform.rotation.y = q[1]
			t.transform.rotation.z = q[2]
			t.transform.rotation.w = q[3]
			self.tf_broadcaster.sendTransformMessage(t)

			t = TransformStamped()
			t.header.stamp = rospy.Time.now()
			t.header.frame_id = 'map_ned'
			t.child_frame_id = 'ref_lamb'
			t.transform.translation.x = self.pose_R0[0]
			t.transform.translation.y = self.pose_R0[1]
			t.transform.translation.z = 0.0
			q = quaternion_from_euler(0,0,self.pose_R0[2])
			t.transform.rotation.x = q[0]
			t.transform.rotation.y = q[1]
			t.transform.rotation.z = q[2]
			t.transform.rotation.w = q[3]
			self.tf_broadcaster.sendTransformMessage(t)


	def Pose_Global_callback(self, msg): 
		'''
			Parse GPS pose into local robot pose in lambert frame
		'''
		# rospy.loginfo('[SENSOR] Pose Local Callback')

		# --- Check GPS fix
		if msg.status.status >= 0:
			lat, lon = msg.latitude, msg.longitude
		else: # --- Send ref lat and lon if no GPS fix - for testing
			rospy.logwarn('[SENSOR] No GPS Fix') 
			lat,lon = lat_standard, lon_standard
		# print(f'sensor lat {lat} lon {lon}')
		x,y = deg_to_Lamb(lon, lat)	
		# print(f'sensor pose lambert avant {x} | {y}')

		# --- First call or when requested with service - Set lambert ref to current position
		if self.flag_set_ref_lamb:
			rospy.loginfo('[SENSOR] Reset Lambert reference point to current pose') 
			self.ref_lamb[0], self.ref_lamb[1] = x,y
			rospy.loginfo(f'[SENSOR] New ref : [{self.ref_lamb[0]},{self.ref_lamb[1]}]')
			self.flag_set_ref_lamb = False
		# --- Update robot pose in local frame

		self.pose_R0[0,0] = x - self.ref_lamb[0]
		self.pose_R0[1,0] = y - self.ref_lamb[1]

	def Heading_callback(self, msg): 
		'''
			Parse Heading message
		'''
		# rospy.loginfo('[SENSOR] Heading Callback')	
		self.pose_R0[2,0] = pi/180*msg.data


	def Reset_Lamb_callback(self,req):
		'''
			Triggered when reset of lambert ref is required
		'''
		# rospy.loginfo('[SENSOR] Reset Lamb Ref serv callback')
		self.flag_set_ref_lamb = True
		return self.flag_set_ref_lamb

	def Lambert_Ref_callback(self, req):
		'''
			Sends current Lambert frame reference when triggered
		'''
		resp = Point()
		resp.x = self.ref_lamb[0,0]
		resp.y = self.ref_lamb[1,0]
		return resp




# Main function.
if __name__ == '__main__':
    rospy.init_node('sensor')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        sensor = SensorNode()
        sensor.loop()
    except rospy.ROSInterruptException: pass
