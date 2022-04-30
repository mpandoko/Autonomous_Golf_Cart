#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Created on Apr 22, 2022
@author: dexterdmonkey
Behaviour Tree of Autonomus Golfcart

This is listener source code,
subscribing and processong the data needed for behaviour trees processes
"""


"""
.. argparse::
   :module: py_trees.demos.trees
   :func: command_line_argument_parser
   :prog: py-trees-demo-trees
.. graphviz:: dot/demo-trees.dot
.. image:: images/trees.gif
"""


##############################################################################
# Imports
##############################################################################

import os
import rospy
import numpy as np

#import dari local_planner
from behaviour_tree.local_planner.collision_checker import CollisionChecker
from behaviour_tree.local_planner.local_planner import LocalPlanner
from behaviour_tree.local_planner.velocity_planner import VelocityPlanner

from multiprocessing.pool import RUN
from persepsi.msg import obj_points
from behaviour_tree.msg import Planner, ukf_states

from behaviour_tree.dummies import obstacle,waypoints_dummies

RUN = False

def state_callback(msg_nav):
    global local_state
    
    local_state['x'] = msg_nav.x
    local_state['y'] = msg_nav.y
    local_state['v'] = np.sqrt(msg_nav.vx**2 + msg_nav.vy**2)
    local_state['yaw'] = msg_nav.yaw_est
     
def perception_callback(data):
    global obj
    
    obj['obj_len'] = data.obj_len
    obj['obj_x'] = data.obj_x
    obj['obj_y'] = data.obj_y
    obj['obj_z'] = data.obj_z
    obj['xc'] = data.xc
    obj['zc'] = data.zc
    obj['vxc'] = data.vxc
    obj['vzc'] = data.vzc
    
def planner_callback(planner_msg):
    global wp_planner
    
    wp_planner['wp_type'] = planner_msg.wp_type
    wp_planner['x'] = planner_msg.x
    wp_planner['y'] = planner_msg.y
    wp_planner['yaw'] = planner_msg.yaw
    wp_planner['v'] = planner_msg.v
    wp_planner['curv'] = planner_msg.curv

def mission_waypoint(mtype='real',file='lurus_ica_2.npy'):
    if (mtype=='real'):
        mission_waypoint = np.load(os.path.abspath(__file__+'/../waypoints/'+file))
    elif (mtype=='simulation'):
        mission_waypoint = np.array(waypoints_dummies().straight())
    return mission_waypoint
    

def waypoint():
    """
    Return updated waypoint from published waypoints
    """
    global wp_planner
    
    curr_waypoint = []
    if (wp_planner['wp_type']==None):
        curr_waypoint = mission_waypoint()
    else:
        for i in range (len(wp_planner['x'])):
            curr_waypoint.append([wp_planner['x'][i],wp_planner['y'][i],wp_planner['yaw'][i],wp_planner['v'][i],wp_planner['curv'][i]])
    return curr_waypoint

def pose():
    global local_state
    global RUN
    wp = mission_waypoint(mtype='simulation')
    
    # parameter adjustment for yaw and waypoints
    # where the local state.
    # is in subtracted by first waypoint and first ever local state when the car is ran
    global first_yaw
    if (not RUN):
        first_yaw = local_state['yaw']
        print('fy = ',first_yaw)
    else:
        first_yaw = first_yaw
    
    # Step 1: Yaw disamakan dalam UTM, dalam kasus ini, Waypoints dianggap sudah UTM
    curr_state = [local_state['x'], local_state['y'], local_state['yaw']-(first_yaw-wp[0][2]), local_state['v']]
    print("current_local state = ", curr_state)
    RUN = False
    return curr_state

#Mereturn jarak kendaraan saat ini dengan titik tujuan
def d_rem(curr_state,waypoint):
    waypoint = waypoint-[waypoint[0][0],waypoint[0][1],0,0,0]
    #State mobil sekarang
    x1 = curr_state[0]
    y1 = curr_state[1]
    
    #Titik akhir tujuan
    x2 = waypoint[-1][0]
    y2 = waypoint[-1][1]

    # print('x1x2y1y2 = ')
    # print(x1,x2,y1,y2)

    d_remain = np.sqrt((x2-x1)**2+(y2-y1)**2) #meter
    return d_remain

#Memisahkan data object points untuk setiap object
def obstacles_classifier():
    # global obj
    obj = obstacle().free()
    
    # print('x,z = (',obj['xc'],obj['zc'],')')
    # print('vx,vz = (',obj['vxc'],obj['vzc'],')')

    obs = []
    a = 0
    for i in range (len(obj['obj_len'])):
        b = int(a + obj['obj_len'][i])
        obj_row = {
            'id': i,
            'obj_x': obj['obj_x'][a:b],
            'obj_y': obj['obj_y'][a:b],
            'obj_z': obj['obj_z'][a:b],
            'xc': obj['xc'][i],
            'zc': obj['zc'][i],
            'vxc': obj['vxc'][i],
            'vzc': obj['vzc'][i],
        }
        obs.append(obj_row)
        a = b
    
    return obs

# Memeriksa apakah ada objek. dimana:
# - waypoint = [x,y,yaw,curve,v]
# - obj_ adalah matriks objek dalam occupancy grid

def leader_selection(waypoint):
    c_location = rospy.get_param('~c_location', [-1.0, 1.0, 3.0]) # m
    c_rad = rospy.get_param('~c_rad', [1.5, 1.5, 1.5]) # m
    d_weight = rospy.get_param('~d_weight', 0.5)
    #Colllision Check Class
    waypoint = waypoint-[waypoint[0][0],waypoint[0][1],0,0,0]
    cc = CollisionChecker(c_location, c_rad, d_weight)
    obstacles = obstacles_classifier()
    obj_coll_id = []
    obj_coll_zc = []
    obc_coll_vzc = []

    x_p = []
    y_p = []
    t_p = []
    for i in range (len(waypoint)):
        x_p.append(waypoint[i][0])
        y_p.append(waypoint[i][1])
        t_p.append(waypoint[i][2])
    path = [x_p, y_p, t_p]
    paths = [path]

    for obstacle in obstacles:
        x = obstacle['obj_x']
        z = obstacle['obj_z']

        # Assign object points to array
        obj_ = np.zeros([len(x), 2])
        for i in range(len(x)):
            obj_[i] = [z[i], x[i]]

        # coll:
        # return True if the path is free collision
        # return False if the path is collided
        coll = cc.collision_check(paths, obj_)

        # Jika dicek terjadi collision
        # yang artinya ada leader, karena ada obstacle pada path
        if (not coll[0]):
            if (not len(obj_coll_zc)) or obj_coll_zc > [obstacle['zc']]:
                obj_coll_id = [obstacle['id']]
                obj_coll_zc = [obstacle['zc']]
                obc_coll_vzc = [obstacle['vzc']]
    return [obj_coll_id, obc_coll_vzc, obj_coll_zc]

def is_leader_ex(waypoint):
    waypoint = waypoint-[waypoint[0][0],waypoint[0][1],0,0,0]
    id = leader_selection(waypoint)[0]
    if (len(id)):
        return True
    else:
        return False
    
def leader_velocity(waypoint):
    waypoint = waypoint-[waypoint[0][0],waypoint[0][1],0,0,0]
    vzc = leader_selection(waypoint)[1]
    return vzc[0]

def leader_distance(waypoint):
    waypoint = waypoint-[waypoint[0][0],waypoint[0][1],0,0,0]
    zc = leader_selection(waypoint)[2]
    return zc[0]

# Occupancy Grid filler for one object,
# with additional grid for object moving based on predicted time.
def occupancy_grid(obstacle, pred_time):
    obj_x = []
    obj_z = []
    vxc = obstacle['vxc']
    vzc = obstacle['vzc']

    for i in range (len(obstacle['obj_x'])):
        obj_x.append(obstacle['obj_x'][i])
        obj_z.append(obstacle['obj_z'][i])

    for i in range(pred_time-1):
        for j in range(len(obj_x)):
            obj_x.append(obj_x[j] + vxc*(i+1))
            obj_z.append(obj_z[j] + vzc*(i+1))
    return obj_z, obj_x


# checking every path generated
# is there free collision path
# The output is either True or None
def possible_path(curr_state,mission_waypoints, pred_time):
    ld_dist = rospy.get_param('~ld_dist', 10.0) # m
    n_offset = rospy.get_param('~n_offset', 5) # m
    offset = rospy.get_param('~offset', 3) # m
    c_location = rospy.get_param('~c_location', [-1, 1, 3]) # m
    c_rad = rospy.get_param('~c_rad', [1.5, 1.5, 1.5]) # m
    d_weight = rospy.get_param('~d_weight', 0.5)
    
    # waypoints = waypoints-[waypoints[0][0],waypoints[0][1],0,0,0]
    lp = LocalPlanner(mission_waypoints, ld_dist, n_offset, offset)
    cc = CollisionChecker(c_location, c_rad, d_weight)
    
    
    ### Generate feasible paths for collision checker
    # Get lookahead index
    ld_idx = lp.get_lookahead_index(curr_state[0], curr_state[1])
    # print('Lookahead: ', mission_waypoints[ld_idx])
    
    # Get offset goal states
    g_set = lp.get_goal_state_set(mission_waypoints[ld_idx], curr_state)
    
    # Plan paths
    path_generated = lp.plan_paths(g_set)
    
    print('Path generated!')
    print('Status:', path_generated[1])
    
    # Assign object points to array
    obstacles = obstacles_classifier()
    obj_ = []
    x_ = []
    z_ = []
    for obstacle in obstacles:
        x, z = occupancy_grid(obstacle,pred_time)
        x_ = x_+x
        z_ = z_+z
    for i in range (len(x_)):
        obj_.append([x_[i],z_[i]])
    obj_ = np.array(obj_)
    
    # Collision check for every path generated
    coll = cc.collision_check(path_generated[0], obj_)
    print("Free collision checker: ",coll)

    for i in range (len(coll)):
        if coll[i] == True:
            return True
        else:
            False


# Set callback data and variables
# Setup dibawah karena agar tidak dipanggil fungsi diatasnya
wp_planner = {
    'wp_type': None,
    'x': [],
    'y': [],
    'yaw': [],
    'v': [],
    'curv': []
}
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
local_state = {
    'x':0.,
    'y':0.,
    'yaw':0.,
    'v':0.,
}

rospy.Subscriber('/object_points', obj_points, perception_callback)
rospy.Subscriber('/ukf_states', ukf_states, state_callback)


def condition():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('condition', anonymous=True)
    rospy.Subscriber('/object_points', obj_points, perception_callback)
    rospy.Subscriber('/ukf_states', ukf_states, state_callback)
    rospy.Subscriber('/wp_planner', Planner, planner_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    condition()