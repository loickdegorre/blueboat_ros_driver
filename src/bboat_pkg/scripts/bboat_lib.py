#!/usr/bin/env python3


from pyproj import Transformer
import numpy as np
import time
import math
from numpy import mean,pi,cos,sin,sinc,sqrt,tan,arctan,arctan2,tanh,arcsin,arccos,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,vstack,hstack,diag,median,\
                    sign,sum,meshgrid,cross,linspace,append,round,trace,rint
from numpy.random import randn,rand,uniform
from numpy.linalg import inv, det, norm, eig,qr


from geometry_msgs.msg import PoseStamped, Point, Twist
from std_msgs.msg import Float64, Bool


from bboat_pkg.srv import *


from matplotlib.pyplot import *

from bboat_pkg.msg import *

MAX_SPEED_FWRD = 2.5 #m/s Bluerobotics website 
MAX_SPEED_TURN = 0.7 #rad/s -> ~30cm between thruster and center

U2_THRESH = 0.75


lat_standard, lon_standard = 48.4180211,-4.4721604 # Bat M ENSTA


def deg_to_Lamb (x1,y1):
	# transformer=Transformer.from_crs(4326,2154,always_xy=True)
	# point=[(x1,y1)]
	# for pt in transformer.itransform(point):
	# 	return pt
	R = 6378137  # Rayon moyen de la Terre en mètres
	x = R*y1*pi/180
	y = R*(x1*pi/180)*cos(y1*pi/180)
	return y,x
	
# def convert_xy2latlon(x, y, lat0, lon0):
#     R = 6378137  # Rayon moyen de la Terre en mètres
#     lat0_rad = np.radians(lat0)
#     dlat = x / R
#     dlon = y / (R * np.cos(lat0_rad))
#     lat = lat0 + dlat * 180 / np.pi
#     lon = lon0 + dlon * 180 / np.pi
#     return lat, lon

#----------------------------------------
def sawtooth(x):
    '''
    permet d'avoir une différence d'angle nulle entre theta = 0 et theta = 2*pi
    :param x: différence d'angle
    :return: différence comprise entre [-pi,pi[
    '''
    return (x+np.pi)%(2*np.pi)-np.pi

#----------------------------------------
def f_dubins(x,u):
	''' 
		Dubins car model function
	'''
	x, y, ψ = x.flatten()
	u1, u2 = u.flatten()

	dx = u1*math.cos(ψ)
	dy = u1*math.sin(ψ)
	dpsi = u2

	return np.array([[dx], [dy], [dpsi]])

# #----------------------------------------
# # Sailboat Model function
# def f_SB(x,u, ψ, awind, P):
#     x,u=x.flatten(),u.flatten()
#     p0,p1,p2,p3,p4,p5,p6,p7,p8,p9 = P.flatten()
#     θ=x[2]; v=x[3]; w=x[4]; δr=u[0]; δsmax=u[1];
#     w_ap = np.array([[awind*cos(ψ-θ) - v],[awind*sin(ψ-θ)]])
#     ψ_ap = angle(w_ap)
#     a_ap=norm(w_ap)
#     sigma = cos(ψ_ap) + cos(δsmax)
#     if sigma < 0 :
#         δs = pi + ψ_ap
#     else :
#         δs = -sign(sin(ψ_ap))*δsmax
#     fr = p4*v*sin(δr)
#     fs = p3*(a_ap**2)* sin(δs-ψ_ap)
#     dx=v*cos(θ) + p0*awind*cos(ψ)
#     dy=v*sin(θ) + p0*awind*sin(ψ)
#     dv=(fs*sin(δs)-fr*sin(δr)-p1*v**2)/p8
#     dw=(fs*(p5-p6*cos(δs)) - p7*fr*cos(δr) - p2*w*v)/p9
#     xdot=np.array([ [dx],[dy],[w],[dv],[dw]])
#     return xdot,δs 

#----------------------------------------
def angle(x):
    x=x.flatten()
    return arctan2(x[1],x[0])

#----------------------------------------
def sawtooth(x):
    return (x+pi)%(2*pi)-pi 

#----------------------------------------
# Quat from Euler - Copied from boat_simulator.py (helios_ros2)
def quaternion_from_euler(roll, pitch, yaw):
	ai = roll / 2.0
	aj = pitch / 2.0
	ak = yaw / 2.0
	ci = math.cos(ai)
	si = math.sin(ai)
	cj = math.cos(aj)
	sj = math.sin(aj)
	ck = math.cos(ak)
	sk = math.sin(ak)
	cc = ci*ck
	cs = ci*sk
	sc = si*ck
	ss = si*sk

	q = np.empty((4, ))
	q[0] = cj*sc - sj*cs
	q[1] = cj*ss + sj*cc
	q[2] = cj*cs - sj*sc
	q[3] = cj*cc + sj*ss

	return q

#----------------------------------------	
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

#----------------------------------------

# Genere les coefs d'un polynome d'ordre 3
def coef_polynoms_order3(t0, t1, x0, x1, dx0, dx1): 
    A = np.array([[t0**3, t0**2, t0, 1], [t1**3, t1**2, t1, 1], [3*t0**2, 2*t0, 1, 0], [3*t1**2, 2*t1, 1, 0]])
    B = np.array([x0, x1, dx0, dx1])

    return np.linalg.solve(A, B)

#----------------------------------------


# Genere les coefs d'un polynome d'ordre 5 (Implique de définir des accélérations au WP)
def coef_polynoms_order5(t0, t1, x0, x1, dx0, dx1, ddx0, ddx1): 
    A = np.array([[t0**5, t0**4, t0**3, t0**2, t0, 1], [t1**5, t1**4, t1**3, t1**2, t1, 1], 
                    [5*t0**4, 4*t0**3, 3*t0**2, 2*t0, 1, 0], [5*t1**4, 4*t1**3, 3*t1**2, 2*t1, 1, 0], 
                    [20*t0**3, 12*t0**2, 6*t0, 2, 0, 0], [20*t1**3, 12*t1**2, 6*t1, 2, 0, 0]])
    B = np.array([x0, x1, dx0, dx1, ddx0, ddx1])

    return np.linalg.solve(A, B)

#----------------------------------------

def calc_traj(points, dT): 
	px, py, tt, ddx, ddy = points[:,0], points[:,1], points[:,2], points[:,3], points[:,4]

	for i in range(len(px)-1): 
		# print(i)
		t0, t1 = tt[i], tt[i+1]
		x0, x1 = px[i], px[i+1]
		y0, y1 = py[i], py[i+1]
		dx0, dx1 = ddx[i], ddx[i+1]
		dy0, dy1 = ddy[i], ddy[i+1]

		# 4th order for speed continuity - Contant speed at WPs
		ax, bx, cx, dx, ex, fx = coef_polynoms_order5(t0, t1, x0, x1, dx0, dx1, 0, 0) 
		ay, by, cy, dy, ey, fy = coef_polynoms_order5(t0, t1, y0, y1, dy0, dy1, 0, 0)
		
		t = np.linspace(t0, t1, int((t1-t0)/dT))
		# x = ax*t**3 + bx*t**2 + cx*t + dx
		# y = ay*t**3 + by*t**2 + cy*t + dy
		# dotx = 3*ax*t**2 + 2*bx*t + cx
		# doty = 3*ay*t**2 + 2*by*t + cy

		x = ax*t**5 + bx*t**4 + cx*t**3 + dx*t**2 + ex*t + fx
		y = ay*t**5 + by*t**4 + cy*t**3 + dy*t**2 + ey*t + fy
		dotx = 5*ax*t**4 + 4*bx*t**3 + 3*cx*t**2 + 2*dx*t + ex
		doty = 5*ay*t**4 + 4*by*t**3 + 3*cy*t**2 + 2*dy*t + ey
		
		if i == 0: 
			traj = np.array([x, y, dotx, doty])
		else: 
			traj = np.hstack((traj, np.array([x, y, dotx, doty])))
	return traj
	