#!/usr/bin/env python3
"""
Created on May 01, 2022
@author: dexterdmonkey
Visualizer of waypoints, states and object
in Autonomus Golfcart
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy

import behaviour_tree.condition as cond 

rospy.init_node("local_planner", anonymous=True)
mission_waypoint = cond.mission_waypoint()
a = np.min(mission_waypoint[:,1])-10
b = np.max(mission_waypoint[:,1])+10

gridsize = (6,9)
fig = plt.figure(figsize=(6,9))
ax1 = plt.subplot2grid(gridsize, (0,0),colspan=6,rowspan=5)
ax2 = plt.subplot2grid(gridsize, (0,6),colspan=3,rowspan=3)

x = np.linspace(0,10,100)
y = np.linspace(5,5,100)


def animate(i):
    try:
        waypoint = cond.waypoint()
        curr_state = cond.pose()
        obstacles = cond.obstacles_classifier()
        x_ = []
        y_ = []
        idx = cond.get_start_and_lookahead_index(waypoint,curr_state[0],curr_state[1],0)
        yaw = curr_state[2]-waypoint[idx[0]][2]
        for obstacle in obstacles:
            x, y = cond.occupancy_grid(obstacle,1)
            x_ = x_+x
            y_ = y_+y
        x_ = np.array(x_)
        y_ = np.array(y_)
        x_ = x_*np.cos(-yaw)-y_*np.sin(-yaw) + curr_state[0]
        y_ = x_*np.sin(-yaw)+y_*np.cos(-yaw) + curr_state[1]
        x = []
        y = []
        for i in range (len(waypoint)):
            x.append(waypoint[i][0])
            y.append(waypoint[i][1])
        ax1.cla()
        
        ax1.plot(mission_waypoint[:,0],mission_waypoint[:,1], color='0.8', linewidth = 2.5, ls="--")
        ax1.plot(curr_state[0],curr_state[1],'ro')
        ax1.plot(x, y, color='blue', linewidth=2)
        ax1.scatter(x_, y_, label="Object",color='black')
        ax1.set_xlabel('x (m)')
        ax1.set_ylabel('y (m)')
        ax1.set_title('Trajectory')
        ax1.set_ylim([a,b])
        ax1.grid()
        
        ax2.plot(i,curr_state[3])
        ax2.set_xlabel('t (s)')
        ax2.set_ylabel('v (m/s)')
        ax2.set_title('Velocity')
        ax2.set_xlim([i-10,i+10])
    except:
        pass

anim = FuncAnimation(fig,animate,interval=100,repeat_delay=5)
 
# Show the plot
plt.grid()
plt.tight_layout()
plt.show()