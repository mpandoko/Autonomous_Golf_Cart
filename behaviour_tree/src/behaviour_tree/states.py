#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Created on Apr 22, 2022
@author: dexterdmonkey
Behaviour Tree of Autonomus Golfcart

The class states which called by trees.py
and execute the function of the state.
"""

import behaviour_tree.condition as cond
import behaviour_tree.actions as act
import py_trees

class TrackSpeed(py_trees.behaviour.Behaviour):
    def __init__(self, name, curr_state, waypoint, a_max, v_ts, topic_name="wp_planner"):
        super(TrackSpeed, self).__init__(name=name)
        self.curr_state = curr_state
        self.waypoint = waypoint
        self.a_max = a_max
        self.v_ts = v_ts
        self.topic_name = topic_name
    
    def setup(self, timeout):
        # self.publisher = rospy.Publisher(self.topic_name, Planner, queue_size=1)
        self.feedback_message = "setup"
        return True
    
    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        
        #the track speed code
        act.track_speed(self.curr_state, self.waypoint, self.a_max, self.v_ts)
        
        print("Vehicle run in constant velocity")
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        return super().terminate(new_status)
    
class IsArrive(py_trees.behaviour.Behaviour):
    def __init__(self, name, curr_state, waypoint):
        super(IsArrive, self).__init__(name=name)
        self.curr_state = curr_state
        self.waypoint = waypoint
    
    def setup(self, timeout):
        self.feedback_message = "setup"
        return True
    
    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        print('current state = ',self.curr_state)
        print('waypoint yaw = ',self.waypoint[0][2])
        #Checking remaining distance of arrival point
        d_remain = cond.d_rem(self.curr_state, self.waypoint)
        print("is vehicle almost arrive?")
        print('the distance is',d_remain,'m again')
        if (d_remain<=4):
            self.feedback_message = "Okey, Let's Stop"
            print("Yes! We will slowdown")
            return py_trees.common.Status.SUCCESS
        else:
            print("Nope, keep running!")
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        return super().terminate(new_status)
    
class Stop(py_trees.behaviour.Behaviour):
    def __init__(self, name, topic_name="wp_planner"):
        super(Stop, self).__init__(name=name)
        self.topic_name = topic_name
    
    def setup(self, timeout):
        # self.publisher = rospy.Publisher(self.topic_name, Planner, queue_size=1)
        self.feedback_message = "setup"
        return True
    
    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        #the stop code
        print("We are in the stop condition, be patient!")
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        return super().terminate(new_status)

class IsLeaderExist(py_trees.behaviour.Behaviour):
    def __init__(self, name, waypoint):
        super(IsLeaderExist, self).__init__(name=name)
        self.waypoint = waypoint
    
    def setup(self, timeout):
        self.feedback_message = "setup"
        return True
    
    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        
        #Checking is there a leader in front of vehicle
        leader = cond.is_leader_ex(self.waypoint)
        if leader:
            self.feedback_message = "Obstacle in front of us"
            print("Oh no, there is leader in front of us")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        return super().terminate(new_status)
    
class IsLeaderFast(py_trees.behaviour.Behaviour):
    def __init__(self, name, v_threshold, waypoint):
        super(IsLeaderFast, self).__init__(name=name)
        self.v_threshold = v_threshold
        self.waypoint = waypoint
    
    def setup(self, timeout):
        self.feedback_message = "setup"
        return True
    
    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        
        #Checking is leader velocity in front of vehicle
        v_leader = cond.leader_velocity(self.waypoint)
        if (v_leader>self.v_threshold):
            self.feedback_message = "but it is fast"
            print("but it is fast")
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "The leader is too slow!"
            print("The leader is too slow!")
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        return super().terminate(new_status)
    
class PossiblePath(py_trees.behaviour.Behaviour):
    def __init__(self, name, waypoint, pred_time):
        super(PossiblePath, self).__init__(name=name)
        self.waypoint = waypoint
        self.pred_time = pred_time
        
    def setup(self, timeout):
        self.feedback_message = "setup"
        return True
    
    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        
        #Checking is leader velocity in front of vehicle
        possible_path = cond.possible_path(self.waypoint, self.pred_time)
        if possible_path:
            self.feedback_message = "and there is possible path"
            print("and there is possible path")
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "and no possible path :("
            print("and no possible path :(")
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        return super().terminate(new_status)
    
class FollowLeader(py_trees.behaviour.Behaviour):
    def __init__(self, name, curr_state, waypoint, a_max, topic_name="wp_planner"):
        super(FollowLeader, self).__init__(name=name)
        self.curr_state = curr_state
        self.waypoint = waypoint
        self.a_max = a_max
        self.topic_name = topic_name
    
    def setup(self, timeout):
        # self.publisher = rospy.Publisher(self.topic_name, Planner, queue_size=1)
        self.feedback_message = "setup"
        return True
    
    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        
        #the follow leader code
        act.follow_leader(self.curr_state, self.waypoint, self.a_max)
        
        print("We should follow the leader!")
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        return super().terminate(new_status)
    
class SwitchLane(py_trees.behaviour.Behaviour):
    def __init__(self, name, waypoint, pred_time, a_max, topic_name="wp_planner"):
        super(SwitchLane, self).__init__(name=name)
        self.topic_name = topic_name
        self.waypoint = waypoint
        self.pred_time = pred_time
        self.a_max = a_max
    
    def setup(self, timeout):
        # self.publisher = rospy.Publisher(self.topic_name, Planner, queue_size=1)
        self.feedback_message = "setup"
        return True
    
    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        
        #the switch lane code
        act.switch_lane(self.waypoint, self.pred_time, self.a_max)
        
        print("We have to switch lane!")
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        return super().terminate(new_status)
    
class DecelerateToStop(py_trees.behaviour.Behaviour):
    def __init__(self, name, curr_state, waypoint, a_max, topic_name="wp_planner"):
        super(DecelerateToStop, self).__init__(name=name)
        self.curr_state = curr_state
        self.waypoint = waypoint
        self.a_max = a_max
        self.topic_name = topic_name
    
    def setup(self, timeout):
        # self.publisher = rospy.Publisher(self.topic_name, Planner, queue_size=1)
        self.feedback_message = "setup"
        return True
    
    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        
        #the decelerate to stop code
        act.decelerate_to_stop(self.curr_state, self.waypoint, self.a_max)
        
        print("Unfortunately, we must stop!")
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        return super().terminate(new_status)