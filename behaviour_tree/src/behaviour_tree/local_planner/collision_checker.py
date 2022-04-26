"""
This work is licensed under the terms of the MIT license.
For a copy, see <https://opensource.org/licenses/MIT>

Author: Vermillord
Based On: Motion Planning for Self-Driving Cars Course Final Project
"""

import numpy as np
import scipy.spatial
from math import sin, cos, pi, sqrt

class CollisionChecker(object):
    def __init__(self, circle_offsets, circle_radii, d_weight):
        self._circle_offsets = circle_offsets
        self._circle_radii = circle_radii
        self._d_weight = d_weight
        
    def collision_check(self, paths, obstacles):
        '''
        Returns a bool array on the path collision check according to the
        vehicle local frame.
        
        args:
            paths: Collection of generated paths by the local  in the
                local vehicle frame.
                Each path has the following format:
                    [x_points, y_points, t_points]:
                        x_points: x-axis waypoints (m)
                        y_points: y-axis waypoints (m)
                        t_points: yaw waypoints (rad)
                        
            obstacles: Collection of obstacle points in the local frame.
                The obstacles are defined as x and y points and 
                has the following format:
                    [[x0, y0],
                     [x1, y1],
                     ...
                     [xn, yn]]
                        xn: x-axis obstacle point (m)
                        yn: y-axis obstacle point (m)
        
        return:
            collision_check_array: List of boolean values that determines
                whether a path is free of collision (True) or not (False).
        '''
        collision_check_array = np.zeros(len(paths), dtype=bool)
        
        # Iterate through each paths
        for i in range(len(paths)):
            collision_free = True
            path = paths[i]
            
            # Iterate through each waypoints in the path
            for j in range(len(path[0])):
                circle_locations = np.zeros((len(self._circle_offsets), 2))
                
                circle_locations[:, 0] = path[0][j] + np.array(self._circle_offsets) * cos(path[2][j])
                circle_locations[:, 1] = path[1][j] + np.array(self._circle_offsets) * sin(path[2][j])
                
                # Iterate through each obstacles
                for k in range(0, len(obstacles), 100):
                    
                    # Get spatial distance from the waypoints to each point of the obstacle
                    collision_dists = scipy.spatial.distance.cdist(obstacles[k].reshape(1, -1), circle_locations)
                    collision_dists = np.subtract(collision_dists, self._circle_radii)
                    collision_free = collision_free and not np.any(collision_dists < 0)
                    
                    if not collision_free:
                        break
                if not collision_free:
                    break
                
            collision_check_array[i] = collision_free
            
        return collision_check_array
    
    def path_selection(self, paths, collision_check_array, goal_state):
        '''
        Returns the best path index to choose from the generated paths array.
        
        args:
            paths: Collection of generated paths by the local  in the
                local vehicle frame.
                Each path has the following format:
                    [x_points, y_points, t_points]:
                        x_points: x-axis waypoints (m)
                        y_points: y-axis waypoints (m)
                        t_points: yaw waypoints (rad)
            
            collision_check_array: List of boolean values that determines
                whether a path is free of collision (True) or not (False).
                
            goal_state: Goal state for the vehicle to reach according to the
                UTM frame.
                Format: [x_goal, y_goal, t_goal, v_goal, c_goal]
                    with [m, m, rad, m/s, 1/m] units
                
        returns:
            best_index: The path index that is chosen as the best path for the
                vehicle to traverse. Selection will be based on the cost of
                each path.
        '''
        best_index = None
        best_cost = np.inf
        
        for i in range(len(paths)):
            if collision_check_array[i]:
                path = np.array(paths[i])
                
                # Calculates the path deviation from the goal state
                cost = np.linalg.norm(np.array(goal_state[:-2]) - path[:-1, -1])
                
                # Calculates the path proximity to the blocked paths
                for j in range(len(paths)):
                    if j==i:
                        continue
                    else:
                        if not collision_check_array[j]:
                            cost += self._d_weight * np.abs(i-j)
            else:
                cost = np.inf
                
            if cost < best_cost:
                best_cost = cost
                best_index = i
                
        return best_index