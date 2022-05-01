#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Created on Apr 22, 2022
@author: dexterdmonkey
Behaviour Tree of Autonomus Golfcart

The main tree code,
launch this program by ROS
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


from distutils.log import WARN
import py_trees
import argparse
import sys
import rospy
import numpy as np
import py_trees.console as console
import behaviour_tree.states as states
import behaviour_tree.condition as cond

from multiprocessing.pool import RUN


##############################################################################
# Classes
##############################################################################

def description():
    content = "This is behaviour tree for Autonomus Golf Cart\n"
    content += "\n"
    content += "The tree will select the states based on its environment conditions\n"
    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Golf Cart - Starttoo!".center(79) + "\n" + console.reset
        s += banner_line
        s += "\n"
        s += content
        s += "\n"
        s += banner_line
    else:
        s = content
    return s


def epilog():
    if py_trees.console.has_colours:
        return console.cyan + "And his noodly appendage reached forth to tickle the blessed...\n" + console.reset
    else:
        return None


def command_line_argument_parser():
    parser = argparse.ArgumentParser(description=description(),
                                     epilog=epilog(),
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     )
    parser.add_argument('-r', '--render', action='store_true', help='render dot tree to file')
    return parser

##############################################################################
# Our Trees
##############################################################################

def create_root():
    #parameters
    v_thres = rospy.get_param('~v_threshold', 0.75) # m/s
    a_maxx = rospy.get_param('~a_max', 0.01) # m/s^2
    prediction_time = rospy.get_param('~pred_time', 2) #s
    v_trackspeed = rospy.get_param('~v_trackspeed', 1) #s
    mission_type = rospy.get_param('~waypoint_mission','simulation')
    mission = cond.mission_waypoint(mtype=mission_type)
    
    
    #the root
    root = py_trees.composites.Selector("Selector")
    seq_arrive = py_trees.composites.Sequence("Seq Arrive")
    seq_leader_check = py_trees.composites.Sequence("Seq Leader Check")
    track_speed = states.TrackSpeed(
        name = "Track Speed",
        curr_state=cond.pose(),
        mission_waypoint=mission,
        a_max = a_maxx,
        v_ts = v_trackspeed
    )
    is_arrive = states.IsArrive(
        name = "Arrive?",
        curr_state=cond.pose(),
        mission_waypoint=mission
    )
    stop = states.Stop(name = "Stop")
    is_leader_exist = states.IsLeaderExist(
        name="Leader?",
        curr_state=cond.pose(),
        waypoint=cond.waypoint())
    fallb_states = py_trees.composites.Selector("Select States")
    seq_follow = py_trees.composites.Sequence("Seq Follow Leader")
    seq_switch = py_trees.composites.Sequence("Seq Switch Lane")
    seq_decelerate = py_trees.composites.Sequence("Seq Decelerate")
    #Leader and Path condition
    leader_fast = states.IsLeaderFast(
        name="Leader>?",
        v_threshold= v_thres,
        curr_state=cond.pose(),
        waypoint=cond.waypoint()
    )
    possible_path_exist = states.PossiblePath(
        name="Possible Path?",
        curr_state=cond.pose(),
        mission_waypoint=mission,
        pred_time=prediction_time,
    )
    #Base states
    follow_leader1 = states.FollowLeader(
        name="Follow Leader",
        curr_state=cond.pose(),
        mission_waypoint=mission,
        waypoint=cond.waypoint(),
        a_max = a_maxx
    )
    switch_lane = states.SwitchLane(
        name="Switch hLane",
        curr_state=cond.pose(),
        waypoint_mission=mission,
        pred_time=prediction_time,
        a_max=a_maxx
    )
    decelerate = states.DecelerateToStop(
        name="Decelerate to Stop",
        curr_state=cond.pose(),
        mission_waypoint=mission,
        a_max = a_maxx
    )
    follow_leader2 = states.FollowLeader(
        name="Follow Leader",
        curr_state=cond.pose(),
        mission_waypoint=mission,
        waypoint=cond.waypoint(),
        a_max = a_maxx
    )
    
    root.add_children([seq_arrive, seq_leader_check,track_speed])
    seq_arrive.add_children([is_arrive,decelerate,stop])
    seq_leader_check.add_children([is_leader_exist, fallb_states])
    fallb_states.add_children([seq_follow,seq_switch,seq_decelerate])
    seq_follow.add_children([leader_fast, follow_leader1])
    seq_switch.add_children([possible_path_exist, switch_lane])
    seq_decelerate.add_children([follow_leader2])
    
    return root

##############################################################################
# Main
##############################################################################

def pre_tick_handler(behaviour_tree):
    print("\n--------- Run %s ---------\n" % behaviour_tree.count)

def main():
    """
    Entry point for the demo script.
    """
    args = command_line_argument_parser().parse_args()
    print(description())
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node('behaviour_tree', anonymous=True)

    root = create_root()

    ####################
    # Rendering
    ####################
    if args.render:
        py_trees.display.render_dot_tree(root)
        sys.exit()

    ####################
    # Execute
    ####################
    behav = py_trees.trees.BehaviourTree(root)
    behav.add_pre_tick_handler(pre_tick_handler)
    behav.visitors.append(py_trees.visitors.DebugVisitor())
    behav.setup(timeout=15)
    
    #Tick Tock
    while True:
        try:
            behav.tick()
        except KeyboardInterrupt:
            break
    print("\n")

while not rospy.is_shutdown():
    main()