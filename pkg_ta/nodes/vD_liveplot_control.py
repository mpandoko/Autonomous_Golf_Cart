#!/usr/bin/env python

########## SIMPLE CONTROLLER PARAMETER LIVE PLOT
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
from pkg_ta.msg import LogArduino
from std_msgs.msg import Float32


########### INITIAL DATA
rospy.init_node('controller_param')
print("[INFO] ROSnode 'controller_param' initialized")

waypoints_path = rospy.get_param('~waypoints_path', 'wp_17des_lurus_1.npy')
waypoints_path = os.path.abspath(sys.path[0] + '/../src/waypoints/waypoints/' + waypoints_path)
# waypoints_path = "wp_17des_lurus_2.npy"
wp = np.load(waypoints_path)

cs_t
cs_ehead_deg
cs_ecstr
cs_v
cs_v_r
cs_steer
cs_cvtr_r

ardu_t
ardu_ad

lg_t
lg_val



########### UPDATE DATA
def callbackUkf(msg):
	global x_u
	global y_u
	global x_u_hist
	global y_u_hist
	global xy_u_hist_updated
	global yaw_u
	global dir_x_u
	global dir_y_u
	global v_u
	x_u = msg.x
	y_u = msg.y
	if xy_u_hist_updated == False:
		xy_u_hist_updated = True
		x_u_hist = np.array([x_u])
		y_u_hist = np.array([y_u])
	else:
		x_u_hist = np.append(x_u_hist, x_u)
		y_u_hist = np.append(y_u_hist, y_u)
	yaw_u = msg.yaw_est
	dir_x_u = np.array([x_u, x_u + np.cos(yaw_u)])
	dir_y_u = np.array([y_u, y_u + np.sin(yaw_u)])
	v_u = np.sqrt(msg.vx**2 + msg.vy**2)

def callbackCtlr(msg):
	global x_c
	global y_c
	global yaw_c
	global dir_x_c
	global dir_y_c
	global ehead_c
	global ecstr_c
	global v_c
	x_c = msg.actual_x
	y_c = msg.actual_y
	yaw_c = msg.actual_yaw
	dir_x_c = np.array([x_c, x_c + np.cos(yaw_c)])
	dir_y_c = np.array([y_c, y_c + np.sin(yaw_c)])
	ehead_c = msg.error_yaw
	ecstr_c = msg.error_lateral
	v_c = msg.actual_speed

def callbackLgain(msg):
	global lyap_gain
	lyap_gain = msg.data

def callbackArdu(msg):
	global ardu_steer_a

	plot2_val = 

rospy.Subscriber('/ukf_states', ukf_states, callbackUkf)
rospy.Subscriber('/control_signal', Control, callbackCtlr)
rospy.Subscriber('/lyap_gain', Float32, callbackLgain)
rospy.Subscriber('/logging_arduino', LogArduino, callbackArdu)
print("[INFO] Subscribed to /ukf_states, /control_signal, /lyap_gain and /logging_arduino")



########### PLOT DATA
print("[INFO] Starting to plot..")
mpl.rcParams['toolbar'] = 'None'
fig = plt.figure(figsize=(8, 4))
fig.canvas.set_window_title('Controller Live Plot')
ax1 = fig.add_subplot(611)
ax2 = fig.add_subplot(612)
ax3 = fig.add_subplot(613)
ax4 = fig.add_subplot(614)
ax5 = fig.add_subplot(615)
ax6 = fig.add_subplot(616)

def update_plot(i):
	# REF https://pythonprogramming.net/live-graphs-matplotlib-tutorial/
	global x_u
	global y_u
	global x_u_hist
	global y_u_hist
	global xy_u_hist_updated
	global yaw_u
	global dir_x_u
	global dir_y_u
	global v_u
	global x_c
	global y_c
	global yaw_c
	global dir_x_c
	global dir_y_c
	global ehead_c
	global ecstr_c
	global v_c

	# plt.clf()
	ax1.clear()
	ax1.plot(cs_t, [0] * len(cs_t))
	ax1.plot(cs_t, cs_ecstr)
	ax1.set_xlim(np.min(cs_t), np.max(cs_t))
	ax1.set_ylabel("ecstr")
	ax1.grid()

	ax2.clear()
	ax2.plot(cs_t, [0] * len(cs_t))
	ax2.plot(cs_t, cs_ehead_deg)
	ax2.set_xlim(np.min(cs_t), np.max(cs_t))
	ax2.set_ylabel("ehead_degree")
	ax2.grid()

	ax3.clear()
	ax3.plot(cs_t, cs_steer, label="Steer Control Signal (CTLR)")
	# plt.plot(ardu_t[:], ardu_sd_ad[:,0], label="Setpoint Steer LLC (ARDU)")
	ax3.plot(ardu_t, ardu_ad, label="Actual Steer (ARDU)")
	ax3.set_xlim(np.min(cs_t), np.max(cs_t))
	ax3.legend()
	ax3.set_ylabel("steer angle")
	ax3.grid()

	ax4.clear()
	ax4.plot(cs_t, cs_v_r, label="ref")
	ax4.plot(cs_t, cs_v, label="actual")
	ax4.set_xlim(np.min(cs_t), np.max(cs_t))
	ax4.set_ylabel("v_ref")
	ax4.legend()
	ax4.grid()

	ax5.clear()
	ax5.plot(cs_t, cs_cvtr_r)
	ax5.set_xlim(np.min(cs_t), np.max(cs_t))
	ax5.set_ylabel("curvature")
	ax5.grid()

	ax6.clear()
	ax6.plot(lg_t, lg_val)
	ax6.set_xlim(np.min(lg_t), np.max(lg_t))
	ax6.set_ylabel("lyap_gain")
	ax6.grid()

live_plot = animation.FuncAnimation(fig, update_plot, interval=30)
plt.show()