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


from bboat_pkg.srv import next_target_serv, mode_serv, mode_servResponse, reset_vsb_serv
from bboat_pkg.msg import mode_msg
from bboat_lib import *
from command_lib import *

WIND_ANGLE = -pi/4 #-pi/2
WIND_SPEED = 1.5

#----------------------------------------
# Sailboat Model function
def f_SB(x,u, ψ, awind, P):
    x,u=x.flatten(),u.flatten()
    p0,p1,p2,p3,p4,p5,p6,p7,p8,p9 = P.flatten()
    θ=x[2]; v=x[3]; w=x[4]; δr=u[0]; δsmax=u[1];
    w_ap = np.array([[awind*cos(ψ-θ) - v],[awind*sin(ψ-θ)]])
    ψ_ap = angle(w_ap)
    a_ap = norm(w_ap)
    sigma = cos(ψ_ap) + cos(δsmax)
    if sigma < 0 :
        δs = pi + ψ_ap
    else :
        δs = -sign(sin(ψ_ap))*δsmax
    fr = p4*v*sin(δr)
    fs = p3*(a_ap**2)* sin(δs-ψ_ap)
    # print(f'merde   {sin(δs-ψ_ap)}') 

    dx=v*cos(θ) + p0*awind*cos(ψ)
    dy=v*sin(θ) + p0*awind*sin(ψ)
    dv=(fs*sin(δs)-fr*sin(δr)-p1*v**2)/p8
    dw=(fs*(p5-p6*cos(δs)) - p7*fr*cos(δr) - p2*w*v)/p9
    xdot=np.array([ [dx],[dy],[w],[dv],[dw]])
    return xdot,δs 


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

def control_SB_obstacles(z,q, a, b, r, ψ, ζ, detected_obstacles, N_control): 
    x, y, theta, _, _ = z.flatten()

    m = np.array([[x],[y]])
    
    e = det(hstack(((b-a)/norm(b-a),m-a)))

    if abs(e) > r:
        q = sign(e)
    
    if len(detected_obstacles) > 0:
        rospy.loginfo('[VSB] EVITEMENT')
        N_ctrl = np.sum(N_control, axis=0) 
        theta_bar = arctan2(N_ctrl[1,0], N_ctrl[0,0])

    else: 
        φ = angle(b-a)
        theta_bar = φ - arctan(e/r)

        if cos(ψ-theta_bar) + cos(ζ) < 0 or (abs(e) < r and cos(ψ-φ)+cos(ζ) < -0.1) :
            # theta_bar = -boat.psi_wind - boat.q*boat.zeta
            theta_bar = (ψ-pi) - q*ζ

    dr=0.3*sawtooth(theta-theta_bar)
    dsmax=(pi/4)*(cos(ψ-theta_bar)+1)
    u=array([[dr],[dsmax]])
    return u, q, theta_bar


# Validation waypoint (b) - Copied from line_follow.py (helios_ros2)
def validation(a,b,m,h):
    '''
        Validates reach of the b waypoint if robot is passed the point or in a rad radius circle around b
    '''
    rad = 2 # Security radius around target - [m]
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
        self.Z = np.array([[0.], [0.], [pi/4.], [0.], [0.]])
        self.dZ = np.array([[0.], [0.], [0.], [0.], [0.]])
        self.δsmax = pi/4
        self.δs = self.δsmax
        self.δr = 0
        self.q = 1
        self.W = np.array([[self.δr], [self.δsmax]])
        self.flag_first_round = 1

        self.θ_bar = self.Z[2,0]

        # ------
        ## Param Simu
        self.awind, self.ψ = WIND_SPEED, WIND_ANGLE # Vent -> Rendre parametrable
        self.r, self.ζ = 5, pi/4+0.2

        # ----
        ## OBSTACLES
        self.Re = 20
        self.N_visu_X, self.N_visu_Y, self.N_control = [], [], []
        


        self.obstacle_file = rospy.get_param('/bboat_vsb_node/obstacle_filepath')
        
        # self.obstacles = [[30, 30 ,5, 20]] # CHANGE TO READING FROM FILE

        self.obstacles = self.Parse_Obstacle_File()
        # self.obstacles=[]
        self.detected_obstacles = []

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

        self.u_SB_store = []
        self.v_SB_store = []
        self.r_SB_store = []

        self.Z_prec = np.zeros((5,1))




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

        AB = self.b-self.a
        n_ort = array([[-1*AB[1,0]],[AB[0,0]]])
        n_ort = n_ort/norm(n_ort)
        self.N = n_ort@n_ort.T
        self.vhat = 50*(AB)/norm(AB) #  np.array([[100, 100]]).T # Uniform field 
        

        self.flag_first_target = 1 


        self.log_flag = rospy.get_param('/bboat_vsb_node/log')

        self.plot_flag = rospy.get_param('/bboat_vsb_node/plot')


        # ------
        ## Loop timing - force loop period to dT while using spin_once
        self.dT = 0.05
        self.rate = rospy.Rate(1/self.dT)

        # ------
        # Robot pose subscriber
        self.sub_pose_robot = rospy.Subscriber('/pose_robot_R0', PoseStamped,  self.Pose_Robot_callback)
        self.pose_robot = PoseStamped()
        rospy.wait_for_message('/pose_robot_R0', PoseStamped, timeout=None)

        self.vel_robot_RB = np.zeros((3,1))
        # self.u_robot_store = []
        # self.v_robot_store = []
        # self.r_robot_store = []
        # self.sub_vel_robot = rospy.Subscriber('/vel_robot_RB', Twist, self.Vel_Robot_callback)

        self.u1, self.u2 = 0, 0
        # self.u1_store = []
        # self.u2_store = []
        # self.sub_cmd = rospy.Subscriber('/command', cmd_msg, self.Command_callback)


        # ------
        ## Publishers
        self.pub_position = rospy.Publisher('/vSBPosition',PoseStamped,  queue_size=10)
        self.pub_speed = rospy.Publisher('/vSBSpeed',PoseStamped,  queue_size=10)

        self.pub_a = rospy.Publisher('/a', Point, queue_size=10)
        self.pub_b = rospy.Publisher('/b', Point, queue_size=10)

        if self.log_flag or self.plot_flag: 
            print('1')
            while self.pub_a.get_num_connections() <1:
                rospy.loginfo("[VSB] Waiting for logger or plotter to catch a and b...")
                rospy.sleep(1)
        elif self.log_flag and self.plot_flag:
            print('2')
            while self.pub_a.get_num_connections() <2:
                rospy.loginfo("[VSB] Waiting for logger and plotter to catch a and b...")
                rospy.sleep(1)

        msg = Point()
        msg.x=self.a[0,0]
        msg.y=self.a[1,0]
        self.pub_a.publish(msg)
        msg = Point()
        msg.x=self.b[0,0]
        msg.y=self.b[1,0]
        self.pub_b.publish(msg)


        # ------
        ## Tf Broadcaster SB state
        self.tf_broadcaster = tf.TransformBroadcaster()

        # ------
        self.validation_count = 0

        # ------
        # Mode service client subscription
        # rospy.wait_for_service('/mode')   
        # connected = False
        # while not connected:
        #     try:
        #         self.client_mode = rospy.ServiceProxy('/mode', mode_serv)
        #         connected = True
        #     except rospy.ServiceException as exc:
        #         rospy.logwarn(f'[VSB] Mode service cannot be reached - {str(exc)}')
        #         connected = False

        self.mode_msg = None
        self.sub_mode = rospy.Subscriber('/modepub', mode_msg, self.Mode_callback)
        rospy.wait_for_message('/modepub', mode_msg,  timeout=None)


        rospy.Service('/reset_vsb', reset_vsb_serv, self.Reset_VSB_Service_callback)

        # --- Init done
        rospy.loginfo('[VSB] VSB node Initialized - waiting for VSB mode')

        while not (self.mode_msg.mode == 'AUTO' and self.mode_msg.mission == 'VSB'): 
            rospy.sleep(1)

        rospy.loginfo('[VSB] Starting VSB')

    def loop(self):
        while not rospy.is_shutdown():

            x, y, psi, v, w = self.Z.flatten()
            dx, dy, r, dv, dw = self.dZ.flatten()
            u_SB, v_SB = 0,0

            # --- Control + Model Integration
            ## Without obstacles
            # if self.flag_continue_mission and self.mode_msg.mode == "AUTO" and self.mode_msg.mission == "VSB": # Sailboat stops at the last point or if in manual mode
            #     self.W, self.q, self.θ_bar = control_SB(self.Z, self.q, self.a, self.b, self.r, self.ψ, self.ζ) 
            #     self.dZ, self.δs = f_SB(self.Z, self.W, self.ψ, self.awind, self.P)

            #     self.Z = self.Z + self.dZ*self.dT

            #     self.Z_prec = self.Z

               
            ## With obstacles
            if self.flag_continue_mission and self.mode_msg.mode == "AUTO" and self.mode_msg.mission == "VSB": # Sailboat stops at the last point or if in manual mode
                self.SeekObstacles()
                self.ValidateObstacle()
                self.N_visu_X, self.N_visu_Y, self.N_control = [], [], []
                self.PotentialField()
                
                self.W, self.q, self.θ_bar = control_SB_obstacles(self.Z, self.q, self.a, self.b, self.r, self.ψ, self.ζ, self.detected_obstacles, self.N_control) 
                self.dZ, self.δs = f_SB(self.Z, self.W, self.ψ, self.awind, self.P)

                self.Z = self.Z + self.dZ*self.dT

                self.Z_prec = self.Z
                
            # else:
            #     # rospy.loginfo('[VSB] Iddle')
            #     self.Z = self.Z + self.dZ*self.dT
            #     # rospy.sleep(10)

            #     # self.dZ = np.zeros((5,1))
            #     # self.Z = self.Z + self.dZ*self.dT

            #     # self.Z = np.array([[self.pose_robot.pose.position.x], [self.pose_robot.pose.position.y], [self.pose_robot.pose.position.z], [0], [0]])
                
            #     u_rob, v_rob, r_rob = self.vel_robot_RB.flatten()

            #     self.dZ[0,0] = u_rob*cos(self.Z[2,0]) + v_rob*sin(self.Z[2,0])
            #     self.dZ[1,0] = -u_rob*sin(self.Z[2,0]) + v_rob*cos(self.Z[2,0])
            #     self.dZ[2,0] = r_rob

            #     self.Z = self.Z + self.dZ*self.dT
            #     # self.Z[2,0] = self.pose_robot.pose.position.z

            #     # print(f'dZ {self.dZ[0,0]}, {self.dZ[1,0]} - dT {self.dT}')
            #     # print(f'Z avant {self.Z}')
            #     # print(f'Z apres {self.Z}')
 

            # --- Build and publish Messages pose + speed

            x, y, psi, v, w = self.Z.flatten()
            dx, dy, r, dv, dw = self.dZ.flatten()
            msg_pos = PoseStamped()
            msg_spe = PoseStamped()
            msg_pos.header.stamp = rospy.Time.now()
            msg_pos.header.frame_id = 'ref_lamb'
            msg_pos.pose.position.x, msg_pos.pose.position.y, msg_pos.pose.position.z = x, y, psi

            msg_spe.pose.position.x, msg_spe.pose.position.y, msg_spe.pose.position.z = dx, dy, r


            self.pub_position.publish(msg_pos)
            self.pub_speed.publish(msg_spe)

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
                # if self.validation_count==0:
                    # self.validation_count+=1

                self.validation_count+=1
                if self.validation_count >= 5:  #self.validation_count==5: 
                    # Request new target when waypoint is validated
                    resp = self.client_next_target(True)
                    rospy.loginfo(f'[VSB] Next target aquired : [{resp.next_trgt_pose.position.x},{resp.next_trgt_pose.position.y}]')
                    self.a = self.b
                    self.b = array([[resp.next_trgt_pose.position.x],[resp.next_trgt_pose.position.y]])
                    self.flag_continue_mission = resp.continuing_mission
                    msg = Point()
                    msg.x=self.a[0,0]
                    msg.y=self.a[1,0]
                    self.pub_a.publish(msg)
                    msg = Point()
                    msg.x=self.b[0,0]
                    msg.y=self.b[1,0]
                    self.pub_b.publish(msg)
                    self.validation_count = 0

                    #Maj vhat for obstacles
                    AB = self.b-self.a
                    self.vhat = 50*(AB)/norm(AB) #  np.array([[100, 100]]).T # Uniform field 
        

            else:
                self.validation_count=0


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

    def Vel_Robot_callback(self, msg): 
        self.vel_robot_RB = np.array([[msg.linear.x], [msg.linear.y], [msg.angular.z]])

    def Command_callback(self, msg): 
        '''
            Parse command msg
            Turn forward and turning speed to 1100 - 2000 values to override
        '''
        self.u1, self.u2 = msg.u1.data, msg.u2.data

    def Reset_VSB_Service_callback(self, req): 
        rospy.loginfo('[VSB] Reset VSB to current bboat pos')
        self.Z = np.array([[self.pose_robot.pose.position.x], [self.pose_robot.pose.position.y], [self.pose_robot.pose.position.z], [0], [0]])

        resp = True
        return resp
    
    def Mode_callback(self, msg):
        self.mode_msg = msg


    def PotentialField(self):
        '''
            Computes the potential field associated with the obstacles, the line (and the wind)
            N_obs_control is the vector relative to 1 obstacle, calculated at the current position of the obstacle and used for control
            N_control is the sum of all N_obs_control + the line ( + the wind)
            N_visu_X and N_visu_Y are used only for plotting, they represent the field calculated at each point of the grid
        '''
        # X_field = arange(self.xmin,self.xmax,5)
        # Y_field = arange(self.ymin,self.ymax,5)
        X_field = arange(-100,100,5)
        Y_field = arange(-100,100,5)
        P1, P2 = meshgrid(X_field,Y_field)

        N_visu_obst_X, N_visu_obst_Y, N_obs_control = 0, 0, 0


        # Vector field associated with the line - Uniform field + Field pushing to line
        N_line_control = 10*self.vhat # 
        N_visu_line_X = N_line_control[0,0]*ones(P1.shape)
        N_visu_line_Y = N_line_control[1,0]*ones(P1.shape)
        self.N_control.append(N_line_control)
        self.N_visu_X.append(N_visu_line_X)
        self.N_visu_Y.append(N_visu_line_Y)

        # N_back_to_line_control = 10*self.N@(self.a - self.Z[0:2])
        # self.N_control.append(N_back_to_line_control)
        # --- Trouver une solution pour le print
        # N_visu_back_to_line_X = N_back_to_line_control[0,0]*ones(P1.shape)
        # N_visu_back_to_line_Y = N_back_to_line_control[1,0]*ones(P1.shape)
        # self.N_visu_X.append(N_visu_back_to_line_X)
        # self.N_visu_Y.append(N_visu_back_to_line_Y)

        # Vector field associated with the wind
        N_wind_control = 1*array([[cos(WIND_ANGLE)],[sin(WIND_ANGLE)]]) # Should it depend on the wind speed ?
        self.N_control.append(N_wind_control)
        N_wind_visu_X = N_wind_control[0,0]*ones(P1.shape)
        N_wind_visu_Y = N_wind_control[1,0]*ones(P1.shape)
        self.N_visu_X.append(N_wind_visu_X)
        self.N_visu_Y.append(N_wind_visu_Y)


        # Vector field associated with the obstacles
        for i in range(0, len(self.detected_obstacles), 1):
            obs = self.detected_obstacles[i] #self.obstacles: 
            R = obs[2]
            q = obs[0:2]#obs[0]

            sigma = R+self.Re
            K = 850*sigma/2 # 850000

            T1, T2 = findTangentPoint(q, self.Z, R)
            plot(T1[0], T1[1], "or")
            plot(T2[0], T2[1], "or")

            # Repulsive field components associated with the obstacle - Gaussian field
            z = gaus2d(P1, P2, q, sigma)

            z_control_tmp = gaus2d(self.Z[0] , self.Z[1], q, sigma)
            N_obs_control = N_obs_control -K*array([ -((self.Z[0] - q[0])/sigma**2)*z_control_tmp, -((self.Z[1] - q[1])/sigma**2)*z_control_tmp])
            
            N_visu_obst_X = N_visu_obst_X + K*((P1 - q[0])/sigma**2)*z
            N_visu_obst_Y = N_visu_obst_Y + K*((P2 - q[1])/sigma**2)*z

            # Adds repulsive field associated with the tangent points
            gain = 3000*R  
            # T1
            Nq_control = gain*(self.Z[0:2] - T1)/(norm((self.Z[0:2] - T1))**3) 
            N_obs_control = N_obs_control + Nq_control

            Vect_visu = array([P1 - T1[0], P2 - T1[1]])
            N_visu_obst_X = N_visu_obst_X + gain*Vect_visu[0]/(sqrt(Vect_visu[0]**2 + Vect_visu[1]**2)**(3)) 
            N_visu_obst_Y = N_visu_obst_Y + gain*Vect_visu[1]/(sqrt(Vect_visu[0]**2 + Vect_visu[1]**2)**(3)) 

            # T2
            Nq_control = gain*(self.Z[0:2] - T2)/(norm((self.Z[0:2] - T2))**3) 
            N_obs_control = N_obs_control + Nq_control
            
            Vect_visu = array([P1 - T2[0], P2 - T2[1]])
            N_visu_obst_X = N_visu_obst_X + gain*Vect_visu[0]/(sqrt(Vect_visu[0]**2 + Vect_visu[1]**2)**(3)) 
            N_visu_obst_Y = N_visu_obst_Y + gain*Vect_visu[1]/(sqrt(Vect_visu[0]**2 + Vect_visu[1]**2)**(3)) 


            self.N_visu_X.append(N_visu_obst_X)
            self.N_visu_Y.append(N_visu_obst_Y)
            self.N_control.append(N_obs_control)



    def SeekObstacles(self): 
        '''
            Detects if the boat is close to an obstacle and adds it to the list of detected obstacles
        '''
        temp = []
        for i in range(0, len(self.obstacles), 1):
            obs = self.obstacles[i]
            R = obs[2]
            q = obs[0:2]
            # if norm(self.pose_robot[0:2].T - q) <= R + self.Re:
            if norm(self.Z[0:2].T - q) <= R + self.Re:
                self.detected_obstacles.append(obs)
            else: 
                temp.append(obs)
        self.obstacles = temp
               
    def ValidateObstacle(self): 
        temp = []
        for i in range(0, len(self.detected_obstacles), 1):
            ob = self.detected_obstacles[i]
            if dot((self.b-self.a).T, self.b - self.Z[0:2]) + 10<= dot(self.b.T-ob[0:2], (self.b-self.a)) or norm(self.Z[0:2].T - ob[0:2]) > 3*(ob[2]+self.Re) : #  or 
                self.obstacles.append(ob)
                
            else: 
                temp.append(ob)
        self.detected_obstacles = temp

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
        return obstacles




# Main function.
if __name__ == '__main__':
    rospy.init_node('virtual_sb')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        vsb = VirtualSBNode()
        vsb.loop()
    except rospy.ROSInterruptException: pass