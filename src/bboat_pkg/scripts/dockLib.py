
from roblib import *
from easydubins import dubin_path
dubin_path.MAX_LINE_DISTANCE = 0.1
dubin_path.MAX_CURVE_DISTANCE = 0.1
dubin_path.MAX_CURVE_ANGLE = 0.1
from bboat_pkg.msg import PathFollowing2DPoint


def sawtooth1D(psi, modulo = "2pi"):
    if modulo == "2pi": return psi%(2*pi)
    if modulo == "pi": return (psi + np.pi) % (2 * np.pi) - np.pi
        
    
def interrogation_chemin(s,L):
    index = find_index_in_interval_np(s, L)
    if index == None : return None, None, None
    p = L[index]
    lievreX, lievreY, lievreC, lievreS, lievrePsi = p.x, p.y, p.c, p.s, p.theta
    return np.array([[lievreX],
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
				sk = sk_1 + np.sqrt((yk_1-yk)**2+(xk_1-xk)**2)

				thetak_1 = path_points[-1].theta
				thetak = np.arctan2(yk-yk_1, xk-xk_1)
				ck = (thetak-thetak_1)/sk

				p.x = xk
				p.y = yk
				p.s = sk
				p.c = ck
				p.theta = thetak
			
			path_points.append(p)

		return path_points
	
	except Exception as e:
		rospy.loginfo(f"Erreur dans le serveur : {e}")
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
				sk = sk_1 + np.sqrt((yk_1-yk)**2+(xk_1-xk)**2)

				thetak_1 = path_points[-1].theta
				thetak = np.arctan2(yk-yk_1, xk-xk_1)
				ck = (thetak-thetak_1)/sk

				p.x = xk
				p.y = yk
				p.s = sk
				p.c = ck
				p.theta = thetak
			
			path_points.append(p)

		return path_points
	
	except Exception as e:
		rospy.loginfo(f"Erreur dans le serveur : {e}")
		return path_points