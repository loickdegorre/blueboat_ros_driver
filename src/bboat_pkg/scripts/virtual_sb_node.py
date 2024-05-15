#!/usr/bin/env python3

from geometry_msgs.msg import Twist, TransformStamped, PoseStamped, PointStamped, Pose
import tf

from matplotlib.pyplot import *

import rospy

import numpy as np
import time
import math
from numpy import mean,pi,cos,sin,sinc,sqrt,tan,arctan,arctan2,tanh,arcsin,arccos,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,vstack,hstack,diag,median,\
                    sign,sum,meshgrid,cross,linspace,append,round,trace,rint
from numpy.random import randn,rand,uniform
from numpy.linalg import inv, det, norm, eig,qr


from bboat_pkg.srv import next_target_serv, mode_serv, mode_servResponse
from lib.bboat_lib import *



# Sailboat Controller - Path following
def control_SB(x,q, a, b, r, ψ, ζ):
    x=x.flatten()
    m=array([[x[0]],[x[1]]])
    θ=x[2]
    e = det(hstack(((b-a)/norm(b-a),m-a)))
    if abs(e) > r:
        q = sign(e)
    φ = angle(b-a)
    θ_bar = φ - arctan(e/r)
    if cos(ψ-θ_bar) + cos(ζ) < 0 or (abs(e) < r and cos(ψ-φ)+cos(ζ) < -0.1) :
        θ_bar = (ψ-pi)-q*ζ
        # θ_bar = -ψ-q*ζ
    δr=1*sawtooth(θ-θ_bar)

    if abs(δr) > pi/4: 
        δr = pi/4*sign(δr)
    δsmax=(pi/4)*(cos(ψ-θ_bar)+1)
    u=array([[δr],[δsmax]])
    return u,q, θ_bar



# Validation waypoint (b) - Copied from line_follow.py (helios_ros2)
def validation(a,b,m,h):
    '''
        Validates reach of the b waypoint if robot is passed the point or in a rad radius circle around b
    '''
    rad = 5 # Security radius around target - [m]
    return (np.dot((b-a).T,(b-m))<0 or np.linalg.norm(b-m)<rad )
#----------------------------------------

class VirtualSBNode():
    '''
        Sailboat Simulation node
        subscribers
            - Robot pose in local frame - for init 
        Publishers
            - Virtual sailboat position and heading in local frame
            - Virtual sailboat speed in local frame
        Service Client
            - Mission client - next waypoint and continuing order
            - Mode client - Sailboat iddles when in manual mode
    '''
    def __init__(self):

        # ------
        ## Virtual sailboat parameters
        # Init -> In the first round, SB takes position of Helios
        p0,p1,p2,p3,p4,p5,p6,p7,p8,p9 = 0.1,50,6000,1000,2000,1,1,2,300,10000
        self.P = np.array([[p0], [p1], [p2], [p3], [p4], [p5], [p6], [p7], [p8], [p9]])
        self.Z = np.array([[0.], [0.], [0.], [0.], [0.]])
        self.dZ = np.array([[0.], [0.], [0.], [0.], [0.]])
        self.δsmax = pi/4
        self.δs = self.δsmax
        self.δr = 0
        self.q = 1
        self.W = np.array([[self.δr], [self.δsmax]])
        self.flag_first_round = 1

        self.θ_bar = self.Z[2,0]

        # ------
        ## Data Storage -> Printing
        self.Z_store = []
        self.x_SB_store = []
        self.y_SB_store = []
        self.dx_SB_store = []
        self.dy_SB_store = []
        self.vel_norm_store = []
        self.psi_store = []
        self.psi_sent_store = []
        self.time_store = []


        # ------
        ## Param Simu
        self.awind, self.ψ = 3, -pi/2 # Vent -> Rendre parametrable
        self.r, self.ζ = 5, pi/4+0.2

        # --- Mission service client
        rospy.wait_for_service('/next_target')
        connected = False
        while not connected:
            try:
                self.client_next_target = rospy.ServiceProxy('/next_target', next_target_serv)
                connected = True
            except rospy.ServiceException as exc:
                rospy.logwarn(f'[VSB] Next Target service cannot be reached - {str(exc)}')
                connected = False
        # First target request
        resp = self.client_next_target(True)
        rospy.loginfo(f'[VSB] First target aquired : [{resp.next_trgt_pose.position.x},{resp.next_trgt_pose.position.y}]')

        self.flag_continue_mission = resp.continuing_mission

        # --- Waypoint initialization
        self.a = array([[0.0],[0.0]])   
        self.b = array([[resp.next_trgt_pose.position.x],[resp.next_trgt_pose.position.y]])

        self.flag_first_target = 1 


        # ------
        ## Loop timing - force loop period to dT while using spin_once
        self.dT = 0.02
        self.rate = rospy.Rate(1/self.dT)

        # ------
        # Robot pose subscriber
        self.sub_pose_robot = rospy.Subscriber('/pose_robot_R0', PoseStamped,  self.Pose_Robot_callback)
        self.pose_robot = PoseStamped()
        rospy.wait_for_message('/pose_robot_R0', PoseStamped, timeout=None)



        # ------
        ## Publishers
        self.pub_position = rospy.Publisher('/vSBPosition',PoseStamped,  queue_size=10)
        self.pub_speed = rospy.Publisher('/vSBSpeed',PoseStamped,  queue_size=10)

        # ------
        ## Tf Broadcaster SB state
        self.tf_broadcaster = tf.TransformBroadcaster()

        # ------
        self.validation_count = 0

        # ------
        # Mode service client subscription
        rospy.wait_for_service('/mode')   
        connected = False
        while not connected:
            try:
                self.client_mode = rospy.ServiceProxy('/mode', mode_serv)
                connected = True
            except rospy.ServiceException as exc:
                rospy.logwarn(f'[VSB] Mode service cannot be reached - {str(exc)}')
                connected = False

        # --- Matplotlib plotting
        self.flag_plotting = False
        if self.flag_plotting:
            self.fig = figure()
            self.plot_wind = self.fig.add_subplot(111, aspect='equal')

        # --- Init done
        rospy.loginfo('[VSB] VSB node Start')

    def loop(self):
        while not rospy.is_shutdown():

            x, y, psi, v, w = self.Z.flatten()
            dx, dy, r, dv, dw = self.dZ.flatten()

            resp = self.client_mode(True)

            if resp.mission == "MI3" and resp.mode == "AUTO":
                msg_pose = PoseStamped()
                msg_pose.pose.position.x, msg_pose.pose.position.y = self.b[0,0], self.b[1,0]
                self.pub_position.publish(msg_pose)

            # --- Control + Model Integration
            if self.flag_continue_mission and resp.mode == "AUTO" and resp.mission == "VSB": # Sailboat stops at the last point or if in manual mode
                self.W, self.q, self.θ_bar = control_SB(self.Z, self.q, self.a, self.b, self.r, self.ψ, self.ζ) 
                self.dZ, self.δs = f_SB(self.Z, self.W, self.ψ, self.awind, self.P)
                self.Z = self.Z + self.dZ*self.dT

                # --- Build and publish Messages pose + speed
                msg_pos = PoseStamped()
                msg_spe = PoseStamped()
                msg_pos.header.stamp = rospy.Time.now()
                msg_pos.header.frame_id = 'ref_lamb'
                msg_pos.pose.position.x, msg_pos.pose.position.y, msg_pos.pose.position.z = x, y, psi

                msg_spe.pose.position.x, msg_spe.pose.position.y, msg_spe.pose.position.z = dx, dy, r


                self.pub_position.publish(msg_pos)
                self.pub_speed.publish(msg_spe)
                
            else:
                # rospy.loginfo('[VSB] Iddle')
                rospy.sleep(10)

                self.dZ = np.zeros((5,1))
                self.Z = self.Z + self.dZ*self.dT



            # --- Broadcast SB tf -> rviz
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = 'ref_lamb'
            t.child_frame_id = 'vSB'
            t.transform.translation.x = self.Z[0,0]
            t.transform.translation.y = self.Z[1,0]
            t.transform.translation.z = 0.0
            q = quaternion_from_euler(0,0,self.Z[2,0])
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransformMessage(t)

            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = 'ref_lamb'
            t.child_frame_id = 'target'
            t.transform.translation.x = self.b[0,0]
            t.transform.translation.y = self.b[1,0]
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = -1.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 0.0
            self.tf_broadcaster.sendTransformMessage(t)


            # --- Validation
            # Validation logic - 5 consecutive validation checks to switch call switch target
            m = np.array([[x], [y]])
            h = np.array([[self.pose_robot.pose.position.x], [self.pose_robot.pose.position.y]])
            if self.validation_count>0:
                self.validation_count+=1
            if(validation(self.a,self.b,m,h)):
                if self.validation_count==0:
                    self.validation_count+=1
                if self.validation_count==5: 
                    # Request new target when waypoint is validated
                    resp = self.client_next_target(True)
                    rospy.loginfo(f'[VSB] Next target aquired : [{resp.next_trgt_pose.position.x},{resp.next_trgt_pose.position.y}]')
                    self.a = self.b
                    self.b = array([[resp.next_trgt_pose.position.x],[resp.next_trgt_pose.position.y]])
                    self.flag_continue_mission = resp.continuing_mission
            else:
                self.validation_count=0

            self.x_SB_store.append(self.Z[0,0])
            self.y_SB_store.append(self.Z[1,0])

            # Plotting
            if self.flag_plotting:
                cla()
                self.plot_wind.grid()

                plot([self.a[0,0], self.b[0,0]], [self.a[1,0], self.b[1,0]], 'k')
                plot(self.x_SB_store, self.y_SB_store, '--r')
                plot(self.Z[0,0], self.Z[1,0], 'or')
                x_rob, y_rob, psi_rob = self.pose_robot.pose.position.x, self.pose_robot.pose.position.y, self.pose_robot.pose.position.z
                plot(x_rob, y_rob, 'ob')
                plot([x_rob, x_rob+5*cos(psi_rob)], [y_rob, y_rob+5*sin(psi_rob)], 'b')

                plot([self.Z[0,0], self.Z[0,0]+5*cos(self.Z[2,0])], [self.Z[1,0], self.Z[1,0]+5*sin(self.Z[2,0])], 'r')
                plot([self.Z[0,0], self.Z[0,0]+5*cos(self.θ_bar)], [self.Z[1,0], self.Z[1,0]+5*sin(self.θ_bar)], 'g')

                wind = 10*self.awind * np.array([[cos(self.ψ)], [sin(self.ψ)]])
                quiver(self.Z[0,0]+3, self.Z[1,0]+3, wind[0,0], wind[1,0])
                pause(.001)
                show(block=False)

            self.rate.sleep()

    def Pose_Robot_callback(self, msg):
        '''
            Parse robot pose message
        '''
        self.pose_robot = msg
        if self.flag_first_round: # VSB initiated on robot pose at first
            self.a[0] = msg.pose.position.x
            self.a[1] = msg.pose.position.y
            self.Z[0] = msg.pose.position.x
            self.Z[1] = msg.pose.position.y
            self.flag_first_round = 0


# Main function.
if __name__ == '__main__':
    rospy.init_node('virtual_sb')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        vsb = VirtualSBNode()
        vsb.loop()
    except rospy.ROSInterruptException: pass