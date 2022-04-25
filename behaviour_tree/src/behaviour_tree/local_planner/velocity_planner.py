"""
This work is licensed under the terms of the MIT license.
For a copy, see <https://opensource.org/licenses/MIT>

Author: Vermillord
Based On: Motion Planning for Self-Driving Cars Course Final Project
"""

import numpy as np
from math import sin, cos, pi, sqrt

class VelocityPlanner(object):
    def __init__(self, a_max):
        self._a_max            = a_max
        
    def nominal_profile(self, path, start_speed, desired_speed):
        """
        Creates a velocity profile for the local planner based on the generated
        path.
        
        args:
            path: The path that the vehicle will follow. Use the transformed
                path.
                Format: [x_points, y_points, t_points]
                        x_points: x point value in m
                        y_points: y point value in m
                        t_points: yaw point value in rad
            start_speed: The instantaneous speed of the vehicle (m/s).
            desired_speed: The desired speed of the vehicle at the end of the
                        path (m/s).
        returns:
            profile: The waypoint profile that the vehicle needs to follow.
                Format: [[x0, y0, t0, v0, c0],
                         [x1, y1, t1, v1, c1],
                         ...,
                         [xn, yn, tn, vn, cn]]
                        with [m, m, rad, m/s, 1/m] units
        """
        
        profile = []
        
        if desired_speed < start_speed:
            accel_distance = calc_distance(start_speed, desired_speed, -self._a_max)
        else:
            accel_distance = calc_distance(start_speed, desired_speed, self._a_max)
            
        ramp_end_index = 0
        distance = 0.0
        while (ramp_end_index < len(path[0])-1) and (distance < accel_distance):
            distance += np.linalg.norm([path[0][ramp_end_index+1] - path[0][ramp_end_index],
                                        path[1][ramp_end_index+1] - path[1][ramp_end_index]])
            ramp_end_index += 1
            
        vi = start_speed
        for i in range(ramp_end_index):
            dist = np.linalg.norm([path[0][i+1] - path[0][i],
                                   path[1][i+1] - path[1][i]])
            if desired_speed < start_speed:
                vf = calc_final_speed(vi, -self._a_max, dist)
                # clamp speed to desired speed
                if vf < desired_speed:
                    vf = desired_speed
            else:
                vf = calc_final_speed(vi, self._a_max, dist)
                # clamp speed to desired speed
                if vf > desired_speed:
                    vf = desired_speed

            profile.append([path[0][i], path[1][i], path[2][i], vi, path[3][i]])
            vi = vf
            
        for i in range(ramp_end_index+1, len(path[0])):
            # x, y flip
            profile.append([path[1][i], path[0][i], path[2][i]-np.pi/2, desired_speed, path[3][i]])
        
        return profile
            
def calc_distance(v_i, v_f, a):
    return (v_f**2 - v_i**2)/(2*a)

def calc_final_speed(v_i, a, d):
    temp = v_i**2 + 2*a*d
    if temp < 0:
        return 0
    else:
        return np.sqrt(temp)