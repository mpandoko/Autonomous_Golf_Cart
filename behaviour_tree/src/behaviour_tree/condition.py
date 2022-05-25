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

def mission_waypoint(mtype='real',file='wp_22Mei1031.npy'):
    if (mtype=='real'):
        mission_waypoint = np.load(os.path.abspath(__file__+'/../../../../waypoints/waypoints/'+file))
    elif (mtype=='simulation'):
        mission_waypoint = np.array(waypoints_dummies().diagonal())
    # print('waypoint misi: ',len(mission_waypoint))
    return mission_waypoint
    
# rospy.init_node('behaviour_tree', anonymous=True)
rospy.Subscriber('/ukf_states', ukf_states, state_callback)
rospy.Subscriber('/wp_planner', Planner, planner_callback)
rospy.Subscriber('/object_points', obj_points, perception_callback)

def waypoint():
    """
    Return updated waypoint from published waypoints
    """
    global wp_planner
    wp_planner_copy = wp_planner.copy()
    curr_waypoint = []
    print('wp_type yang subscribe kondisi.py')
    print(wp_planner['wp_type'])
    if (wp_planner['wp_type']==None):
        curr_waypoint = mission_waypoint()
    else:
        for i in range (len(wp_planner_copy['x'])):
            # print("wp_planner")
            # print(i)  
            # print(len(wp_planner_copy['x']))
            # print(len(wp_planner_copy['y']))
            # print(len(wp_planner_copy['yaw']))
            # print(len(wp_planner_copy['v']))
            # print(len(wp_planner_copy['curv']))
            curr_waypoint.append([wp_planner_copy['x'][i],wp_planner_copy['y'][i],wp_planner_copy['yaw'][i],wp_planner_copy['v'][i],wp_planner_copy['curv'][i]])
            np.array(curr_waypoint)
    # print('curr_waypoint:')
    # for i in range (5):
    #     print(curr_waypoint[i])
    # print(len(curr_waypoint))
    # print('mission waypo: ')
    # for i in range (5):
    #     print( mission_waypoint()[i])
    # print(len(mission_waypoint()))
    return curr_waypoint

def manuver_type():
    global wp_planner
    manuvers = [
        'Global',
        'Track Speed',
        'Follow Leader',
        'Switch Lane',
        'Decelerate to Stop',
        'Stop'
    ]
    manuver = manuvers[wp_planner['wp_type']]
    return manuver

def pose():
    """
    Return updated state estimation of vehicle from published waypoints
    """
    global local_state
    global RUN
    # wp = mission_waypoint(mtype='simulation')
    
    # # parameter adjustment for yaw and waypoints
    # # where the local state.
    # # is in subtracted by first waypoint and first ever local state when the car is ran
    # global first_yaw
    # if (not RUN):
    #     first_yaw = local_state['yaw']
    #     print('fy = ',first_yaw)
    # else:
    #     first_yaw = first_yaw
    
    # Step 1: Yaw disamakan dalam UTM, dalam kasus ini, Waypoints dianggap sudah UTM
    # curr_state = [local_state['x'], local_state['y'], local_state['yaw']-(first_yaw-wp[0][2]), local_state['v']]
    curr_state = [local_state['x'], local_state['y'], local_state['yaw'], local_state['v']]
    # print("current vehicle state: x = %.2f, y = %.2f, yaw = %.4f rad, v = %.2f m/s" %(curr_state[0],curr_state[1],curr_state[2],curr_state[3]))
    RUN = True
    return curr_state

#Memisahkan data object points untuk setiap object
def obstacles_classifier():
    # # Uncoment kalau object real
    global obj
    
    # Uncoment kalau dummies
    obj = obstacle().slow()
    
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

#Mereturn jarak kendaraan saat ini dengan titik tujuan
def d_rem(curr_state,mission_waypoint):
    #State mobil sekarang
    x1 = curr_state[0]
    y1 = curr_state[1]
    
    #Titik akhir tujuan
    x2 = mission_waypoint[-1][0]
    y2 = mission_waypoint[-1][1]

    # print('x1x2y1y2 = ')
    # print(x1,x2,y1,y2)

    d_remain = np.sqrt((x2-x1)**2+(y2-y1)**2) #meter
    return d_remain

# Memeriksa apakah ada objek. dimana:
# - waypoint = [x,y,yaw,curve,v]
# - obj_ adalah matriks objek dalam occupancy grid
def leader_selection(curr_state,waypoint):
    c_location = rospy.get_param('~c_location', [-0.2, 1.2, 2.2]) # m
    c_rad = rospy.get_param('~c_rad', [0.9, 0.9, 0.9]) # m
    d_weight = rospy.get_param('~d_weight', 0.5)
    #Colllision Check Class
    cc = CollisionChecker(c_location, c_rad, d_weight)
    
    
    obstacles = obstacles_classifier()
    obj_coll_id = []
    obj_coll_zc = []
    obc_coll_vzc = []

    idx = get_start_and_lookahead_index(waypoint,curr_state[0],curr_state[1],0)
    yaw = curr_state[2]-waypoint[idx[0]][2]
    
    x_p = []
    y_p = []
    t_p = []
    for i in range (idx[0],len(waypoint)):
        x_p.append(waypoint[i][0]-waypoint[idx[0]][0])
        y_p.append(waypoint[i][1]-waypoint[idx[0]][1])
        t_p.append(waypoint[i][2]-waypoint[idx[0]][2])
    path = [x_p, y_p, t_p]
    paths = [path]

    for obstacle in obstacles:
        # Assign object points to array
        objs_ = occupancy_grid(obstacle,0)
        obj_ = []
        for i in range (len(objs_[0])):
            obj_.append([objs_[0][i],objs_[1][i]])
        obj_ = np.array(obj_)
    
        # print(obj_)
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
                obc_coll_vzc = [obstacle['vzc']+curr_state[3]*np.cos(yaw)]
    return [obj_coll_id, obc_coll_vzc, obj_coll_zc]

def is_leader_ex(curr_state,waypoint):
    id = leader_selection(curr_state,waypoint)[0]
    if (len(id)):
        return True
    else:
        return False
    
def leader_velocity(curr_state,waypoint):
    vzc = leader_selection(curr_state,waypoint)[1]
    if vzc==[]:
        return None
    return vzc[0]

def leader_distance(curr_state,waypoint):
    zc = leader_selection(curr_state,waypoint)[2]
    if zc==[]:
        return None
    return zc[0]

def get_start_and_lookahead_index(waypoint, x, y, ld_dist):
    min_idx       = 0
    min_dist      = np.inf
    for i in range(len(waypoint)):
        dist = np.linalg.norm(np.array([
                waypoint[i][0] - x,
                waypoint[i][1] - y]))
        if dist < min_dist:
            min_dist = dist
            min_idx = i
    # print('==================lookahead idx====================')
    # print(min_idx)
    total_dist = min_dist
    # print('totaldist',total_dist)
    lookahead_idx = min_idx
    for i in range(min_idx + 1, len(waypoint)):
        if total_dist >= ld_dist:
            break
        total_dist += np.linalg.norm(np.array([
                waypoint[i][0] - waypoint[i-1][0],
                waypoint[i][1] - waypoint[i-1][1]]))
        lookahead_idx = i
    # print('===================================================')
    return min_idx,lookahead_idx


# Occupancy Grid filler for one object,
# with additional grid for object moving based on predicted time.
def occupancy_grid(obstacle, pred_time):
    mission = mission_waypoint()
    curr_state = pose()
    idx = get_start_and_lookahead_index(mission,curr_state[0],curr_state[1],0)
    yaw = curr_state[2]-mission[idx[0]][2]
    x_ = []
    y_ = []
    vx_ = obstacle['vzc']
    vy_ = obstacle['vxc']

    # # Real
    for i in range (len(obstacle['obj_x'])):
        x = obstacle['obj_z'][i]
        y = obstacle['obj_x'][i]
        x_.append(x*np.cos(yaw)-y*np.sin(yaw))
        y_.append(x*np.sin(yaw)+y*np.cos(yaw))
    
    # Simulasi
    # for i in range (len(obstacle['obj_x'])):
    #     x = obstacle['obj_z'][i]
    #     y = obstacle['obj_x'][i]
    #     x_.append(x)
    #     y_.append(y)
    
    for i in range(pred_time-1):
        for j in range(len(x_)):
            x_.append(x_[j] + vx_*(i+1))
            y_.append(y_[j] + vy_*(i+1))
    return x_, y_


# checking every path generated
# is there free collision path
# The output is either True or None
def possible_path(curr_state,mission_waypoints, pred_time):
    # print("curr_state_cond")
    # print(curr_state)
    # curr_state = curr_state + [0,0,np.pi,0]
    # print(curr_state)
    ld_dist = rospy.get_param('~ld_dist', 10.0) # m
    n_offset = rospy.get_param('~n_offset', 9) # m
    offset = rospy.get_param('~offset', 1.5) # m
    c_location = rospy.get_param('~c_location', [-0.2, 1.2, 2.2]) # m
    c_rad = rospy.get_param('~c_rad', [0.9, 0.9, 0.9]) # m
    d_weight = rospy.get_param('~d_weight', 0.5)
    print("a")
    # waypoints = waypoints-[waypoints[0][0],waypoints[0][1],0,0,0]
    lp = LocalPlanner(mission_waypoints, ld_dist, n_offset, offset)
    cc = CollisionChecker(c_location, c_rad, d_weight)
    print("b")
    
    
    ### Generate feasible paths for collision checker
    # Get lookahead index
    ld_idx = lp.get_lookahead_index(curr_state[0], curr_state[1])
    # print('Lookahead: ', mission_waypoints[ld_idx])
    print("c")
    # Get offset goal states
    g_set = lp.get_goal_state_set(mission_waypoints[ld_idx], curr_state)
    print("d")
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


# rospy.init_node('condition', anonymous=True)
# Set callback data and variables
# Setup dibawah karena agar tidak dipanggil fungsi diatasnya

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