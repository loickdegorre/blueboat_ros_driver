
# from roblib import *
from easydubins import dubin_path
dubin_path.MAX_LINE_DISTANCE = 0.1
dubin_path.MAX_CURVE_DISTANCE = 0.1
dubin_path.MAX_CURVE_ANGLE = 0.1
from bboat_pkg.msg import PathFollowing2DPoint
import numpy as np
from numpy import pi, cos, sin, array, sqrt, arctan2



def sawtooth1D(psi, modulo = "2pi"):
    if modulo == "2pi": return psi%(2*pi)
    if modulo == "pi": return (psi + pi) % (2 * pi) - pi
        
    
def interrogation_chemin(s,L):
    index = find_index_in_interval_np(s, L)
    if index == None : return None, None, None
    p = L[index]
    lievreX, lievreY, lievreC, lievreS, lievrePsi = p.x, p.y, p.c, p.s, p.theta
    return array([[lievreX],
                     [lievreY],
                     [lievrePsi]]), lievreC,lievreS

def find_index_in_interval_np(s, pathCSY): #on donne une valeur s et une liste 1D, le code renvoie le premier indice entre lesquels se trouve notre valeur s
    s_list = [p.s for p in pathCSY]
    for i in range(len(s_list) - 1):
        if s_list[i] <= s < s_list[i + 1]:
            return i
    return None


def get_straight_line(pose_robot, tf,v ):
	x0, y0, psi0 = pose_robot.flatten()
	try:
		state0 = [x0, y0, psi0]
		path=[[x0+v*tf*cos(psi0),y0+v*tf*sin(psi0),psi0] for k in range(1000)]
		path_points = []
		
		
		for k in range(0,len(path)):
			p = PathFollowing2DPoint()
			
			if k == 0 :
				p.x = x0
				p.y = y0
				p.s = 0.0
				p.c = 0.0
				p.theta = psi0

			else:
				xk_1,yk_1,_= path[k-1]
				xk,yk,_  = path[k]

				sk_1 = path_points[-1].s
				sk = sk_1 + sqrt((yk_1-yk)**2+(xk_1-xk)**2)

				thetak_1 = path_points[-1].theta
				thetak = arctan2(yk-yk_1, xk-xk_1)
				ck = (thetak-thetak_1)/sk

				p.x = xk
				p.y = yk
				p.s = sk
				p.c = ck
				p.theta = thetak
			
			path_points.append(p)

		return path_points
	
	except Exception as e:
		print(f"Erreur dans le serveur : {e}")
		return path_points
	

def get_dubins_path_callback(pose_robot, final_pose_robot, turning_radius):
	x0, y0, psi0 = pose_robot.flatten()
	xf, yf, psif = final_pose_robot.flatten()
	try:
		state0 = [x0, y0, psi0]
		state1 = [xf, yf, psif]
		solution = dubin_path.dubins_path(state0, state1, radius=turning_radius)

		path = dubin_path.get_projection(state0, state1, solution)
		path_points = []
		
		
		for k in range(0,len(path)):
			p = PathFollowing2DPoint()
			
			if k == 0 :
				p.x = x0
				p.y = y0
				p.s = 0.0
				p.c = 0.0
				p.theta = psi0

			else:
				xk_1,yk_1,_= path[k-1]
				xk,yk,_  = path[k]

				sk_1 = path_points[-1].s
				sk = sk_1 + sqrt((yk_1-yk)**2+(xk_1-xk)**2)

				thetak_1 = path_points[-1].theta
				thetak = arctan2(yk-yk_1, xk-xk_1)
				ck = (thetak-thetak_1)/sk

				p.x = xk
				p.y = yk
				p.s = sk
				p.c = ck
				p.theta = thetak
			
			path_points.append(p)

		return path_points
	
	except Exception as e:
		print(f"Erreur dans le serveur : {e}")
		return path_points


def computeTfLSL(X0, Y0, psi0, psiY, R, v, vY, t0):
    x0,y0 = Y0.flatten()
    A = Y0-X0-R*np.array([[-sin(psi0)+sin(psiY)],[cos(psi0)-cos(psiY)]]) + vY/v*np.array([[cos(psiY)],[sin(psiY)]])*R*(psiY-psi0)
    Ax,Ay = A.flatten()
    normA = np.sqrt(Ax**2+Ay**2)
    d2 = normA**2/(-vY/v*(Ax*cos(psiY)+Ay*sin(psiY))+np.sqrt((vY/v)**2*( (Ax*cos(psiY)+Ay*sin(psiY))**2-normA**2 )+ normA**2))
    alpha1_plus = (np.arctan2(Ay/d2+vY/v*sin(psiY), Ax/d2+vY/v*cos(psiY))-psi0)
    alpha3_plus = (psiY-(psi0+alpha1_plus))

    if alpha1_plus + alpha3_plus > 2*np.pi : reste = -2*np.pi*R/v
    else: reste = 0.0
    alpha1_plus = alpha1_plus%(2*np.pi)
    alpha3_plus = alpha3_plus%(2*np.pi)
    d1,d3 = R*alpha1_plus,R*alpha3_plus
    tf = t0 + d1/v+d2/v+d3/v#-2*np.pi*R/v
    return tf, d1,d2,d3

def get_dubins_pathCS(pose_robot, final_pose_robot, turning_radius, modeChosen):
	x0, y0, psi0 = pose_robot.flatten()
	xf, yf, psif = final_pose_robot.flatten()
	try:
		state0 = [x0, y0, psi0]
		state1 = [xf, yf, psif]

		solutions = dubin_path.all_dubins_paths([x0, y0,psi0], [xf, yf, psif], radius=turning_radius)
		for k in range(len(solutions)):
			solution = solutions[k]
			mode = solution[0]
			print(f"mode : {mode}")
			if mode == modeChosen: 
				path = np.array(dubin_path.get_projection([x0, y0,psi0], [xf, yf, psif], solution))

		path_points = []
		
		
		for k in range(0,len(path)):
			p = PathFollowing2DPoint()
			
			if k == 0 :
				p.x = x0
				p.y = y0
				p.s = 0.0
				p.c = 0.0
				p.theta = psi0

			else:
				xk_1,yk_1,_= path[k-1]
				xk,yk,_  = path[k]

				sk_1 = path_points[-1].s
				sk = sk_1 + sqrt((yk_1-yk)**2+(xk_1-xk)**2)

				thetak_1 = path_points[-1].theta
				thetak = arctan2(yk-yk_1, xk-xk_1)
				ck = (thetak-thetak_1)/sk

				p.x = xk
				p.y = yk
				p.s = sk
				p.c = ck
				p.theta = thetak
			
			path_points.append(p)

		return path_points
	
	except Exception as e:
		print(f"Erreur dans le serveur : {e}")
		return path_points