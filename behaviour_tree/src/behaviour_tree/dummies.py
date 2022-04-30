import numpy as np
import matplotlib.pyplot as plt
from behaviour_tree.local_planner.spiral_generator import SpiralGenerator
from behaviour_tree.local_planner.velocity_planner import VelocityPlanner


class obstacle():
    def __init__(self, distance=7.5):
        self.dist = distance
    
    def slow(self):
        """
        Returning obstacle with z velocity < 0.75
        """
        len = 1000
        x = np.random.uniform(-0.2,0.2,len)
        y = np.random.uniform(-0.2,0.2,len)
        z = np.random.uniform(self.dist-0.25,self.dist+0.25,len)
        obj = {
            'obj_len': [len],
            'obj_x': x,
            'obj_y': y,
            'obj_z': z,
            'xc': [0],
            'zc': [np.mean(z)],
            'vxc': [0],
            'vzc': [0.5],
        }

        return obj
    
    def fast(self):
        """
        Returning obstacle with z velocity > 0.75,
        but no more than 1
        """
        len = 1000
        x = np.random.uniform(-0.2,0.2,len)
        y = np.random.uniform(-0.2,0.2,len)
        z = np.random.uniform(self.dist-0.25,self.dist+0.25,len)
        obj = {
            'obj_len': [len],
            'obj_x': x,
            'obj_y': y,
            'obj_z': z,
            'xc': [0],
            'zc': [np.mean(z)],
            'vxc': [0],
            'vzc': [0.8],
        }
        # plt.plot(z,x,'ro')
        # plt.xlabel('x')
        # plt.ylabel('y')
        # plt.show()

        return obj
    
    def free(self):
        """
        No obstacle
        """
        obj = {
            'obj_len': [],
            'obj_x': [],
            'obj_y': [],
            'obj_z': [],
            'xc': [],
            'zc': [],
            'vxc': [],
            'vzc': [],
        }

        return obj
    def jammed(self,n_offset, offset):
        """
        Returning jamming condition where with z velocity << 0.75,
        and there's no possible path exists
        """
        len = 1000
        x = []
        
        y = []
        z = []
        r = offset*(n_offset//2)
        print(r)
        for i in range(n_offset): 
            x_ = np.random.uniform(-0.2-r+(i*offset),0.2-r+(i*offset),len)
            y_ = np.random.uniform(-0.2,0.2,len)
            z_ = np.random.uniform(self.dist-0.25,self.dist+0.25,len)
            
            x.append(x_)
            y.append(y_)
            z.append(z_)
        obj = {
            'obj_len': [len for i in range (n_offset)],
            'obj_x': x,
            'obj_y': y,
            'obj_z': z,
            'xc': [(offset*i-r) for i in range (n_offset)],
            'zc': [np.mean(z) for i in range (n_offset)],
            'vxc': [0 for i in range (n_offset)],
            'vzc': [0.3 for i in range (n_offset)],
        }
        # plt.plot(z,x,'ro')
        # plt.xlabel('x')
        # plt.ylabel('y')
        # plt.show()

        return obj

class waypoints_dummies():
    """
    Generating waypoints, with default maximum acceleration 1 m/s2
    """
    def __init__(self,a_max=0.01):
        self.a_max = a_max
        self.path_generator = SpiralGenerator()
        self.vp = VelocityPlanner(a_max)
    
    def straight(self,v=1):
        x = np.linspace(0,50,501)
        y = [0.01 for i in range(501)]
        yaw = [0.01 for i in range(501)]
        curv = [0 for i in range(501)]
        path = [x,y,yaw,curv]
        waypoint = self.vp.nominal_profile(path,v,0)
        
        return waypoint
    
    def turn_left(self,v=1):
        xf = 5
        yf = 5
        yawf = np.pi/2
        path = self.path_generator.optimize_spiral(
            xf,
            yf,
            yawf
        )
        curv = self.path_generator.get_curvature()
        path.append(curv)
        waypoint = self.vp.nominal_profile(path,v,0)
        
        
        return waypoint
    
    def diagonal(self,v=1):
        t = np.arctan(1)
        x = np.linspace(0,50,501)
        y = np.linspace(0,50,501)
        yaw = [t for i in range(501)]
        curv = [0 for i in range(501)]
        path = [x,y,yaw,curv]
        waypoint = self.vp.nominal_profile(path,v,0)
        
        return waypoint

# wp = waypoints_dummies().straight()
# v = []
# x = []
# for i in range (len(wp)):
#     v.append(wp[i][3])
#     x.append(wp[i][0])
# plt.plot(x,v,'bo')
# plt.xlabel('x')
# plt.ylabel('v')
# plt.show()
