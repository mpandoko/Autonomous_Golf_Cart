#!/usr/bin/env python3
"""
Created on Sat Apr 10 13:26:19 2021

@author: vermillord,

updated on Apr 25, 2022
@author: mpandoko, dexterdmonkey
"""

import numpy as np
from pygame import K_GREATER
import rospy
import time
import sys
import os
from behaviour_tree.msg import ukf_states
from behaviour_tree.msg import Planner
import matplotlib.pyplot as plt

from behaviour_tree.local_planner.local_planner import LocalPlanner
from behaviour_tree.local_planner.collision_checker import CollisionChecker
from behaviour_tree.local_planner.velocity_planner import VelocityPlanner
import behaviour_tree.condition as cond

rospy.init_node('local_planner')

# # State callback variables
# def state_callback(msg_nav):
#     global state
#     global RUN

#     state['x'] = msg_nav.x
#     state['y'] = msg_nav.y
#     state['v'] = np.sqrt(msg_nav.vx**2 + msg_nav.vy**2)
#     state['yaw'] = msg_nav.yaw_est
#     # state['yaw'] = atan2(2 * (nav_msg.pose.pose.orientation.w * nav_msg.pose.pose.orientation.z + nav_msg.pose.pose.orientation.x * nav_msg.pose.pose.orientation.y), 1 - 2 * (nav_msg.pose.pose.orientation.y**2 + nav_msg.pose.pose.orientation.z**2))
#     # state['yaw'] = msg_nav.yaw_imu

#     RUN = True

def follow_leader():
    freq = rospy.get_param('~freq', 5.) # Hz
    a_max = rospy.get_param('~a_max', 0.005) # m/s^2
    
    pub = rospy.Publisher('/wp_planner', Planner, queue_size=1)
    
    msg = Planner()
    msg.header.frame_id = 'local_planner'
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    last_time = msg.header.stamp.to_sec() - 1./freq
    
    #Current state
    curr_state = cond.pose
    waypoint = cond.waypoint

    print("State estimation received!")
    print("Planning velocity...")
    
    
    ### Calculate the actual sampling time
    msg.header.stamp = rospy.Time.now()
    delta_t = msg.header.stamp.to_sec() - last_time
    last_time = msg.header.stamp.to_sec()
    
    #Asumsi ketika selisih jarak 5m, kecepatan kendaraan biar 1m/s
    Kgap = 0.2
    
    d_min = 2.4
    l_veh = 2.4
    d_des = max(l_veh*curr_state[3]/5, d_min)
    d_act = cond.d_rem
    v_cmd = Kgap*(d_act - d_des)
    
    #generate velocity
    vp = VelocityPlanner(a_max)
    path = [waypoint[1], waypoint[2], waypoint[3]]
    velocity_profile = vp.nominal_profile(path, curr_state[3],v_cmd)[3]
    
    
    ### Send the message
    # Header
    msg.header.seq += 1
    # Type
    msg.wp_type = waypoint[0]
    # Waypoints
    msg.x = waypoint[1]
    msg.y = waypoint[2]
    msg.yaw = waypoint[3]
    msg.v = velocity_profile
    msg.curv = waypoint[5]
    # Publish the message
    pub.publish(msg)
    
def track_speed():
    freq = rospy.get_param('~freq', 5.) # Hz
    a_max = rospy.get_param('~a_max', 0.005) # m/s^2
    
    pub = rospy.Publisher('/wp_planner', Planner, queue_size=1)
    
    msg = Planner()
    msg.header.frame_id = 'local_planner'
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    last_time = msg.header.stamp.to_sec() - 1./freq
    
    #Current state
    curr_state = cond.pose
    waypoint = cond.waypoint

    print("State estimation received!")
    print("Planning velocity...")
    
    
    ### Calculate the actual sampling time
    msg.header.stamp = rospy.Time.now()
    delta_t = msg.header.stamp.to_sec() - last_time
    last_time = msg.header.stamp.to_sec()
    
    v_cmd = 1
    
    #generate velocity
    vp = VelocityPlanner(a_max)
    path = [waypoint[1], waypoint[2], waypoint[3]]
    velocity_profile = vp.nominal_profile(path, curr_state[3],v_cmd)[3]
    
    
    ### Send the message
    # Header
    msg.header.seq += 1
    # Type
    msg.wp_type = waypoint[0]
    # Waypoints
    msg.x = waypoint[1]
    msg.y = waypoint[2]
    msg.yaw = waypoint[3]
    msg.v = velocity_profile
    msg.curv = waypoint[5]
    # Publish the message
    pub.publish(msg)
    
def decelerate_to_stop():
    freq = rospy.get_param('~freq', 5.) # Hz
    a_max = rospy.get_param('~a_max', 0.005) # m/s^2
    
    pub = rospy.Publisher('/wp_planner', Planner, queue_size=1)
    
    msg = Planner()
    msg.header.frame_id = 'local_planner'
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    last_time = msg.header.stamp.to_sec() - 1./freq
    
    #Current state
    curr_state = cond.pose
    waypoint = cond.waypoint

    print("State estimation received!")
    print("Planning velocity...")
    
    
    ### Calculate the actual sampling time
    msg.header.stamp = rospy.Time.now()
    delta_t = msg.header.stamp.to_sec() - last_time
    last_time = msg.header.stamp.to_sec()
    
    v_cmd = 0
    
    #generate velocity
    vp = VelocityPlanner(a_max)
    path = [waypoint[1], waypoint[2], waypoint[3]]
    velocity_profile = vp.nominal_profile(path, curr_state[3],v_cmd)[3]
    
    
    ### Send the message
    # Header
    msg.header.seq += 1
    # Type
    msg.wp_type = waypoint[0]
    # Waypoints
    msg.x = waypoint[1]
    msg.y = waypoint[2]
    msg.yaw = waypoint[3]
    msg.v = velocity_profile
    msg.curv = waypoint[5]
    # Publish the message
    pub.publish(msg)

def switch_lane(waypoints):
    freq = rospy.get_param('~freq', 5.) # Hz
    ld_dist = rospy.get_param('~ld_dist', 5.0) # m
    n_offset = rospy.get_param('~n_offset', 5) # m
    offset = rospy.get_param('~offset', 0.5) # m
    c_location = rospy.get_param('~c_location', [-1.0, 1.0, 3.0]) # m
    c_rad = rospy.get_param('~c_rad', [1.5, 1.5, 1.5]) # m
    d_weight = rospy.get_param('~d_weight', 0.5)
    a_max = rospy.get_param('~a_max', 0.005) # m/s^2
    pred_time = 2 #s
    # waypoints_path = rospy.get_param('~waypoints_path', 'ica_2.npy')
    # waypoints_path = os.path.abspath(sys.path[0] + '/../../pkg_ta/src/waypoints/waypoints/' + waypoints_path)

    # # Load default waypoints
    # waypoints = np.load(waypoints_path) + [0, 0, -np.pi/5, 0, 0]
    # print(waypoints_path)

    # Create local planner classes
    lp = LocalPlanner(waypoints, ld_dist, n_offset, offset)
    cc = CollisionChecker(c_location, c_rad, d_weight)
    vp = VelocityPlanner(a_max)

    # Create publisher and subscriber
    # rospy.Subscriber('/ukf_states', ukf_states, state_callback)
    pub = rospy.Publisher('/wp_planner', Planner, queue_size=1)
    # rate = rospy.Rate(freq) # Hz

    # Wait until we get the actual state
    # print("Waiting for state estimation...")
    # RUN = False
    # while not RUN:
    #     time.sleep(0.02) # 20 ms
    #     pass
    # curr_state = [state['x'], state['y'], state['yaw'], state['v']]

    msg = Planner()
    msg.header.frame_id = 'local_planner'
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    last_time = msg.header.stamp.to_sec() - 1./freq

    # Set waypoints type (0: global, 1: local)
    # wp_type = 0
    
    curr_state = cond.pose

    print("State estimation received!")
    print("Planning local paths...")

    # =============================================================================
    # # Create matplotlib figure
    # plt.figure()
    # plt.ion()
    # =============================================================================

    # prev_x = []
    # prev_y = []
    # prev_yaw = []
    # prev_v = []
    # prev_curv = []

    ### Calculate the actual sampling time
    msg.header.stamp = rospy.Time.now()
    delta_t = msg.header.stamp.to_sec() - last_time
    last_time = msg.header.stamp.to_sec()
    
    ### Local Planner
    print('Generating feasible paths...')
    
    # Format [x, y, t, v]
    print('Current yaw: ', curr_state[2])
    print('Current x, y: ', curr_state[:2])
    # Get lookahead index
    ld_idx = lp.get_lookahead_index(curr_state[0], curr_state[1])
    print('Lookahead yaw: ', waypoints[ld_idx][2])
    
    # Get offset goal states
    g_set = lp.get_goal_state_set(waypoints[ld_idx], waypoints, curr_state)
    
    # Plan paths
    path_generated = lp.plan_paths(g_set)
    
    print('Path generated!')
    print('Status:', path_generated[1])

    obstacles = cond.obstacles_classifier
    # Assign object points to array

    obj_ = np.zeros([len(x), 2])
    for obstacle in obstacles:
        z, x = cond.occupancy_grid(obstacle, pred_time)
        obj_[i] = [z[i], x[i]]
        
# =============================================================================
#     # Plot data for debugging
#     plt.clf()
#     plt.scatter(0, 0, color='k', label='start')
#     for goal in g_set:
#         plt.scatter(goal[0], goal[1], color='b')
#     for path in path_generated[0]:
#         plt.plot(path[0], path[1], color='g')
#     plt.xlabel('x(m)')
#     plt.ylabel('y(m)')
#     plt.legend()
#     plt.show()
# =============================================================================
    
    # Collision Check
    coll = cc.collision_check(path_generated[0], obj_)
    print(coll)
    bp = cc.path_selection(path_generated[0], coll, g_set[n_offset//2])
    
    # Declare variables
    x = []
    y = []
    yaw = []
    v = []
    curv = []
    

    # Set waypoints type
    wp_type = 1
    
    # Transform paths
    tf_paths = lp.transform_paths(path_generated[0], curr_state)
    
    # Generate waypoints with speed and curvature
    # Format [x, y, t, v, curv]
    best_wp = vp.nominal_profile(tf_paths[bp], curr_state[-1], g_set[bp][-1])

    # Add starting waypoints
    wp_0 = curr_state
    wp_0.append(0.0)
    wp_0 = [(best_wp[0][0] + wp_0[0])/2, (best_wp[0][1] + wp_0[1])/2, (best_wp[0][2] + wp_0[2])/2,
            (best_wp[0][3] + wp_0[3])/2, (best_wp[0][4] + wp_0[4])/2]
    #best_wp.insert(0, wp_0)
    
    # Convert waypoints to message format
    for i in range(len(best_wp)):
        x.append(best_wp[i][0])
        y.append(best_wp[i][1])
        yaw.append(best_wp[i][2]+np.pi/5)
        v.append(best_wp[i][3])
        curv.append(best_wp[i][4])
    
    print('Local waypoints used:', bp)

    
    ### Send the message
    # Header
    msg.header.seq += 1
    # Type
    msg.wp_type = wp_type
    # Waypoints
    msg.x = x
    msg.y = y
    msg.yaw = yaw
    msg.v = v
    msg.curv = curv
    # Publish the message
    pub.publish(msg)

    while(True):
        obstacles = cond.obstacles()
        obj_ = np.zeros([len(x), 2])
        for i in range in (len(obstacles)):
            z, x = cond.occupancy_grid(obstacles[i], pred_time)
            obj_[i] = [z[i], x[i]]

        
        # Collision Check
        coll = cc.collision_check([best_wp], obj_)
        if not coll:
            break

