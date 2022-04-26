# -*- coding: utf-8 -*-
"""
Created on Thu Feb 18 14:21:50 2021

@author: Vermillord
"""

import numpy as np
import matplotlib.pyplot as plt

class Transform(object):
    def __init__(self, coords):
        self._coords = coords
        
    def rot(self, theta):
        rot_matrix = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        rot_coord = []
        for coord in self._coords:
            rot_coord.append(np.matmul(rot_matrix, coord))
    
        return rot_coord