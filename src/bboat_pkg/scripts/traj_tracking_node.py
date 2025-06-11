#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose, Point
from scipy.interpolate import CubicSpline

from bboat_pkg.srv import  lambert_ref_serv, traj_serv, traj_servResponse
from bboat_lib import *
from command_lib import *

dT = 0.02 # controller node rate
# DEDICATED TO TRAJECTORY TRACKING MISSION 
# Parse mission file
# Transforms points into cartesian coordinates
# Generates Polynoms for each segment
# Makes trajectory available as a service for controller node

class TrajTrackingNode(): 
    def __init__(self): 
        self.mission_file = rospy.get_param('/bboat_traj_tracking_node/filepath')

        self.ref_lamb = np.zeros((2,1))
        rospy.loginfo('[TRAJ] Waiting for lambert ref service')
        rospy.wait_for_service('/lambert_ref')
        connected = False
        while not connected:
            try:
                self.client_ref_lambert = rospy.ServiceProxy('/lambert_ref', lambert_ref_serv)
                connected = True
            except rospy.ServiceException as exc:
                rospy.logwarn(f'[TRAJ] Lambert ref service cannot be reached - {str(exc)}')
                connected = False
        resp = self.client_ref_lambert(True)
        self.ref_lamb[0,0] = resp.lambert_ref.x
        self.ref_lamb[1,0] = resp.lambert_ref.y

        rospy.loginfo(f'[TRAJ] Lambert ref {self.ref_lamb}')

        self.waypoints, self.traj = self.Parse_Mission_File()

        self.traj_service = rospy.Service('/get_traj', traj_serv, self.handle_traj_service)

        rospy.loginfo('[TRAJ] Traj Node Initialized')

    def loop(self): 
        while not rospy.is_shutdown():
            rospy.sleep(1)

    def Parse_Mission_File(self): 
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
        traj = calc_traj(points, dT)

        file.close()
        return points, traj
    
    def handle_traj_service(self, req):
        trajx = self.traj[0,:]
        trajy = self.traj[1,:]
        trajdx = self.traj[2,:]
        trajdy = self.traj[3,:]
        return traj_servResponse(trajx, trajy, trajdx, trajdy)
    
if __name__ == '__main__':
    rospy.init_node('traj_tracking')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        mission = TrajTrackingNode()
        mission.loop()
    except rospy.ROSInterruptException: pass


    