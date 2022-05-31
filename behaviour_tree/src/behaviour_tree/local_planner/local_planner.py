

"""
This work is licensed under the terms of the MIT license.
For a copy, see <https://opensource.org/licenses/MIT>

Author: Vermillord
Based On: Motion Planning for Self-Driving Cars Course Final Project
"""

import numpy as np
import copy
from behaviour_tree.local_planner.spiral_generator import SpiralGenerator
from math import sin, cos, pi, sqrt
import matplotlib.pyplot as plt

class LocalPlanner(object):
    def __init__(self, waypoints, ld_dist, num_paths, path_offset):
        self._wp = waypoints
        self._ld = ld_dist
        self._num_paths = num_paths
        self._path_offset = path_offset
        self._path_generator = SpiralGenerator()
        self._curvature = []
    
    def get_lookahead_index(self, x, y):
        min_idx       = 0
        min_dist      = np.inf
        for i in range(len(self._wp)):
            dist = np.linalg.norm(np.array([
                    self._wp[i][0] - x,
                    self._wp[i][1] - y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i

        total_dist = min_dist
        lookahead_idx = min_idx
        for i in range(min_idx + 1, len(self._wp)):
            if total_dist >= self._ld:
                break
            total_dist += np.linalg.norm(np.array([
                    self._wp[i][0] - self._wp[i-1][0],
                    self._wp[i][1] - self._wp[i-1][1]]))
            lookahead_idx = i
        
        return lookahead_idx
    
    def get_goal_state_set(self, goal_state, ego_state):
        """
        Returns an array of goal state set that offsetted laterally from
        the main goal state by an amount of path_offset.
        
        args:
            goal_state: Goal state for the vehicle to reach according to the
                UTM frame.
                Format: [x_goal, y_goal, v_goal] with [m, m, m/s] units
            waypoints: The current waypoints needed to be tracked by the vehicle.
                Format: [[x0, y0, t0, v0, c0],
                         [x1, y1, t1, v1, c1],
                         ...,
                         [xn, yn, tn, vn, cn]] with [m, m, rad, 1/m, m/s] units
            ego_state: The current ego state of the vehicle based on the UKF
                estimation result.
                Format: [ego_x, ego_y, ego_yaw, ego_v]
                    ego_x and ego_y: Position (m)
                    ego_yaw: Heading (rad) [-pi, pi]
                    ego_v: Speed (m/s)
        
        return:
            goal_state_set: Set of goal state set that are offsetted laterally
                from the original goal state in the local vehicle frame.
                The set are used by the planner to create multiple optional
                paths for the vehicle.
                Format: [[x0, y0, t0, v0],
                         [x1, y1, t1, v1],
                         ...,
                         [xn, yn, tn, vn]]
                    n is the total number of generated goal state. The units
                    are in [m, m, rad, m/s]
        """
        
        goal_state_local = np.copy(goal_state)
        
        goal_state_local[0] -= ego_state[0]
        goal_state_local[1] -= ego_state[1]
        
        goal_x = goal_state_local[0]*np.cos(-ego_state[2]) - goal_state_local[1]*np.sin(-ego_state[2])
        goal_y = goal_state_local[0]*np.sin(-ego_state[2]) + goal_state_local[1]*np.cos(-ego_state[2])
        
        goal_t = goal_state_local[2] - ego_state[2]
        
        goal_v = goal_state[3]
        
        if goal_t > np.pi:
            goal_t -= 2*np.pi
        elif goal_t < -np.pi:
            goal_t += 2*np.pi
            
        print(goal_t)
        goal_state_set = []
        
        for i in range(self._num_paths):
            offset = (i - self._num_paths // 2)*self._path_offset
            
            x_offset = offset * np.cos(goal_t + np.pi/2)
            y_offset = offset * np.sin(goal_t + np.pi/2)
            
            goal_state_set.append([goal_x + x_offset,
                                   goal_y + y_offset,
                                   goal_t,
                                   goal_v])
            print(goal_state_set[i][0],goal_state_set[i][1],goal_state_set[i][2])
        return goal_state_set
    
    def plan_paths(self, goal_state_set):
        """
        Plans the path using the spiral_generator for each goal in the
        goal_state_set
        """
        
        paths = []
        path_validity = []
        
        for goal_state in goal_state_set:
            path = self._path_generator.optimize_spiral(goal_state[0],
                                                        goal_state[1],
                                                        goal_state[2])
            
            self._curvature.append(self._path_generator.get_curvature())
            
            if np.linalg.norm([path[0][-1] - goal_state[0], 
                               path[1][-1] - goal_state[1], 
                               path[2][-1] - goal_state[2]]) > 0.1:
                # paths.append(path)
                path_validity.append(False)
            else:
                paths.append(path)
                path_validity.append(True)
            # plt.plot(path[0],path[1])
            # plt.show()
            
        return paths, path_validity
    
    def transform_paths(self, paths, ego_state):
        """
        Transforms the local paths to the global frame
        """
        egs = np.copy(ego_state)
        
        transformed_paths = []
        j = 0
        for path in paths:
            x_transformed = []
            y_transformed = []
            t_transformed = []
            
            for i in range(len(path[0])):
                x_transformed.append(egs[0] + path[0][i]*cos(egs[2]) - \
                                                    path[1][i]*sin(egs[2]))
                y_transformed.append(egs[1] + path[0][i]*sin(egs[2]) + \
                                                    path[1][i]*cos(egs[2]))
                t_transformed.append(path[2][i] + egs[2])
            
            transformed_paths.append([x_transformed, y_transformed, t_transformed, self._curvature[j]])
            j += 1
            
        return transformed_paths