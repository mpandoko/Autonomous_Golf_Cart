#!/usr/bin/env python

########## SIMPLE NAVIGATION LIVE PLOT
###### Written by DimasAP (github.com/dispectra)
###### init. code 2020-12-18

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.ticker import MultipleLocator
import rospy
import time
import sys
import os
from pkg_ta.msg import Control
from pkg_ta.msg import ukf_states


########### INITIAL DATA
rospy.init_node('liveplot')
print("[INFO] rosnode 'liveplot' initialized")

waypoints_path = rospy.get_param('~waypoints_path', 'wp_17des_lurus_1.npy')
waypoints_path = os.path.abspath(sys.path[0] + '/../src/waypoints/waypoints/' + waypoints_path)
# waypoints_path = "wp_17des_lurus_2.npy"
wp = np.load(waypoints_path)

ukf_x = np.min(wp[:,0])
ukf_y = np.min(wp[:,1])
ukf_x_hist = np.empty(1)
ukf_y_hist = np.empty(1)
ukf_xy_hist_updated = False
ukf_yaw = wp[0,2]
ukf_yaw_x = np.array([ukf_x, ukf_x + np.cos(ukf_yaw+np.pi/2)])
ukf_yaw_y = np.array([ukf_y, ukf_y + np.sin(ukf_yaw+np.pi/2)])
ukf_v = 0.

cs_x = ukf_x
cs_y = ukf_y
cs_yaw = ukf_yaw
cs_yaw_x = np.array([cs_x, cs_x + np.cos(cs_yaw+np.pi/2)])
cs_yaw_y = np.array([cs_y, cs_y + np.sin(cs_yaw+np.pi/2)])
cs_ehead = 0.
cs_ecstr = 0.
cs_v = 0.
cs_xy_r = np.array([wp[0,0], wp[0,1]])

########### UPDATE DATA
def callbackUkf(msg_u):
	global ukf_x
	global ukf_y
	global ukf_x_hist
	global ukf_y_hist
	global ukf_xy_hist_updated
	global ukf_yaw
	global ukf_yaw_x
	global ukf_yaw_y
	global ukf_v
	ukf_x = msg_u.x
	ukf_y = msg_u.y
	if ukf_xy_hist_updated == False:
		ukf_xy_hist_updated = True
		ukf_x_hist = np.array([ukf_x])
		ukf_y_hist = np.array([ukf_y])
	else:
		ukf_x_hist = np.append(ukf_x_hist, ukf_x)
		ukf_y_hist = np.append(ukf_y_hist, ukf_y)
	ukf_yaw = msg_u.yaw_est
	# ukf_yaw = msg_u.yaw_dydx-np.pi/2
	ukf_yaw_x = np.array([ukf_x, ukf_x + np.cos(ukf_yaw+np.pi/2)])
	ukf_yaw_y = np.array([ukf_y, ukf_y + np.sin(ukf_yaw+np.pi/2)])
	ukf_v = np.sqrt(msg_u.vx**2 + msg_u.vy**2)

def callbackCtlr(msg_c):
	global cs_x
	global cs_y
	global cs_yaw
	global cs_yaw_x
	global cs_yaw_y
	global cs_ehead
	global cs_ecstr
	global cs_v
	global cs_xy_r
	cs_x = msg_c.actual_x
	cs_y = msg_c.actual_y
	cs_yaw = msg_c.actual_yaw
	cs_yaw_x = np.array([cs_x, cs_x + np.cos(cs_yaw+np.pi/2)])
	cs_yaw_y = np.array([cs_y, cs_y + np.sin(cs_yaw+np.pi/2)])
	cs_ehead = msg_c.error_yaw
	cs_ecstr = msg_c.error_lateral
	cs_v = msg_c.actual_speed
	cs_xy_r = np.array([msg_c.ref_x, msg_c.ref_y])

rospy.Subscriber('/ukf_states', ukf_states, callbackUkf)
rospy.Subscriber('/control_signal', Control, callbackCtlr)
print("[INFO] Subscribed to /ukf_states and /control_signal")

########### PLOT DATA
print("[INFO] Starting to plot..")
mpl.rcParams['toolbar'] = 'None'
fig = plt.figure(figsize=(3.5, 3.5))
fig.canvas.set_window_title('Navigation Live Plot')
ax = plt.gca()
ax.format_coord = lambda x, y: ''

def update_plot(i):
	# REF https://pythonprogramming.net/live-graphs-matplotlib-tutorial/
	global ukf_x
	global ukf_y
	global ukf_x_hist
	global ukf_y_hist
	global ukf_xy_hist_updated
	global ukf_yaw
	global ukf_yaw_x
	global ukf_yaw_y
	global ukf_v
	global cs_x
	global cs_y
	global cs_yaw
	global cs_yaw_x
	global cs_yaw_y
	global cs_ehead
	global cs_ecstr
	global cs_v
	global cs_xy_r

	# plt.clf()
	ax.clear()

	## Waypoints
	# ax.plot(wp[:,0], wp[:,1], c="red", label=r"$(x,y)_{ref}$", lw=1, ls="--"))
	ax.scatter(wp[:,0], wp[:,1], c="red", label=r"$(x,y)_{ref}$", s=0.5, zorder=2, alpha=0.2)
	
	## Instantaneous waypoint reference
	ax.scatter(cs_xy_r[0], cs_xy_r[1], marker="o", zorder=3, c="red", s=8)

	## Actual position trail
	if ukf_xy_hist_updated:
		hist_amount = min(len(ukf_x_hist), len(ukf_y_hist))
		ax.scatter(ukf_x_hist[:hist_amount], ukf_y_hist[:hist_amount], c="blue", s=0.5, label=r"$(x,y)_{t}$", zorder=5)

	## Current pose (from /control_signal message)
	# ax.plot(cs_yaw_x, cs_yaw_y, c="orange")
	# ax.scatter(cs_x, cs_y, c="orange", s=10)

	## Current pose (from /ukf_states message)
	# ax.plot(ukf_yaw_x, ukf_yaw_y, c="black")
	# ax.scatter(ukf_x, ukf_y, c="black", s=10, zorder=6)
	ax.quiver(ukf_x, ukf_y, np.cos(ukf_yaw+np.pi/2), np.sin(ukf_yaw+np.pi/2),
			color="#26bee0", angles='uv', pivot="tail", minshaft=0.5, lw=1, edgecolor="black", 
			headaxislength=6.9, headlength=8, headwidth=6, zorder=6)
	
	## Live numeric info box
	ax.text(0.02,0.03, 
			 r"$\psi_t$ (rad): {0:0.3f}" "\n" r"$\rho_t$ (m): {1:0.3f}" "\n" r"$v_t$ (m/s): {2:0.3f}"
			 .format(cs_ehead, cs_ecstr, ukf_v),
			 fontsize=7, bbox={'facecolor': 'white', 'alpha': 0.5, 'pad':1},
			 transform=ax.transAxes, zorder=10)
	ax.legend(loc="upper right", fontsize=7)

	ax.set_xlim(np.min(wp[:,0])-2, np.max(wp[:,0])+2)
	ax.set_ylim(np.min(wp[:,1])-2, np.max(wp[:,1])+2)
	ax.axis('equal')
	ax.xaxis.set_minor_locator(MultipleLocator(1))
	ax.yaxis.set_minor_locator(MultipleLocator(1))
	ax.xaxis.set_major_locator(MultipleLocator(5))
	ax.yaxis.set_major_locator(MultipleLocator(5))
	ax.grid(color='gray', linestyle='--', linewidth=0.8, which='major', alpha=0.5)
	ax.grid(color='gray', linestyle='--', linewidth=0.5, which='minor', alpha=0.2)

	ax.tick_params(labelsize=6)
	ax.yaxis.offsetText.set_visible(False)
	ax.xaxis.offsetText.set_visible(False)
	_yofflabel = ax.yaxis.offsetText.get_text()
	_xofflabel = ax.xaxis.offsetText.get_text()
	ax.set_ylabel(r"$y_{utm}$ " + _yofflabel + " (m)", fontsize=7)
	ax.set_xlabel(r"$x_{utm}$ " + _xofflabel + " (m)", fontsize=7)	

live_plot = animation.FuncAnimation(fig, update_plot, interval=30)
plt.show()