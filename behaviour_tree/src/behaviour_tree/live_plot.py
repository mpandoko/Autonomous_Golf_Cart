#!/usr/bin/env python3
"""
Created on May 01, 2022
@author: dexterdmonkey
Visualizer of waypoints, states and object
in Autonomus Golfcart
"""

from curses import curs_set
from locale import currency
from turtle import color
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import behaviour_tree.condition as cond 


mission_waypoint = cond.mission_waypoint()
a = np.min(mission_waypoint[:,1])-10
b = np.max(mission_waypoint[:,1])+10

gridsize = (6,9)
fig = plt.figure(figsize=(6,9))
ax1 = plt.subplot2grid(gridsize, (0,0),colspan=6,rowspan=5)
ax1.set_ylim([a,b])
plt.grid()
ax2 = plt.subplot2grid(gridsize, (0,6),colspan=3,rowspan=3)
plt.grid()

x = np.linspace(0,10,100)
y = np.linspace(5,5,100)


def animate(i):
    try:
        waypoint = cond.waypoint()
        curr_state = cond.pose()
        obstacles = cond.obstacles_classifier()
        obj_ = []
        x_ = []
        z_ = []
        for obstacle in obstacles:
            x, z = cond.occupancy_grid(obstacle,1)
            x_ = x_+x
            z_ = z_+z
        for i in range (len(x_)):
            obj_.append([x_[i],z_[i]])
        obj_ = np.array(obj_)
    
        ax1.cla()
        
        ax1.plot(mission_waypoint[:,0],mission_waypoint[:,1], color='black')
        ax1.plot(curr_state[0],curr_state[1],'ro')
        ax1.plot(waypoint[:,0], waypoint[:,1], color='blue',ls="--")
        ax1.scatter(x_, z_, label="Objject",color='black')
        ax1.set_xlabel('x (m)')
        ax1.set_ylabel('y (m)')
        ax1.set_title('Trajectory')
        
        ax2.plot(i,curs_set[3])
        ax1.set_xlabel('t (s)')
        ax1.set_ylabel('v (m/s)')
        ax1.set_title('Velocity')
    except:
        pass

anim = FuncAnimation(fig,animate,interval=100,repeat_delay=5)
 
# Show the plot
plt.tight_layout()
plt.show()