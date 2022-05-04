#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Created on Apr 22, 2022
@author: dexterdmonkey
Behaviour Tree of Autonomus Golfcart

The class states which called by trees.py
and execute the function of the state.
"""

from unicodedata import east_asian_width
import behaviour_tree.condition as cond
import behaviour_tree.actions as act
import py_trees

################################
## Condition ##
################################

class IsArrive(py_trees.behaviour.Behaviour):
    def __init__(self, name, curr_state, mission_waypoint):
        super(IsArrive, self).__init__(name=name)
        self.curr_state = curr_state
        self.mission_waypoint = mission_waypoint
    
    def setup(self, timeout):
        self.feedback_message = "setup"
        return True
    
    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        #Checking remaining distance of arrival point
        d_remain = cond.d_rem(self.curr_state, self.mission_waypoint)
        print("is vehicle almost arrive?")
        if (d_remain<=10):
            self.feedback_message = "Okey, Let's Stop"
            print("Yes! 10 meters again, we will slowdown")
            return py_trees.common.Status.SUCCESS
        else:
            print("Nope the distance is: %2.f meters again, keep running!" % d_remain)
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        return super().terminate(new_status)

class IsLeaderExist(py_trees.behaviour.Behaviour):
    def __init__(self, name, curr_state, waypoint):
        super(IsLeaderExist, self).__init__(name=name)
        self.waypoint = waypoint
        self.curr_state = curr_state
    
    def setup(self, timeout):
        self.feedback_message = "setup"
        return True
    
    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        
        #Checking is there a leader in front of vehicle
        leader = cond.is_leader_ex(self.curr_state,self.waypoint)
        if leader:
            self.feedback_message = "Obstacle in front of us"
            print("Leader in front of us")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        return super().terminate(new_status)
    
class IsLeaderFast(py_trees.behaviour.Behaviour):
    def __init__(self, name, v_threshold, curr_state,waypoint):
        super(IsLeaderFast, self).__init__(name=name)
        self.v_threshold = v_threshold
        self.waypoint = waypoint
        self.curr_state = curr_state
    def setup(self, timeout):
        self.feedback_message = "setup"
        return True
    
    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        
        #Checking is leader velocity in front of vehicle
        v_leader = cond.leader_velocity(self.curr_state,self.waypoint)
        print('with, v:' ,v_leader)
        if v_leader==None:
            print('STATUS: Leader suddenly dissapears')
            print("================== Tick ends, to the next run =====================")
            return py_trees.common.Status.RUNNING
        elif (v_leader>self.v_threshold):
            self.feedback_message = "it is fast"
            print("it is fast")
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "TIt is too slow!"
            print("It is too slow!")
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        return super().terminate(new_status)
    
class PossiblePath(py_trees.behaviour.Behaviour):
    def __init__(self, name, curr_state, mission_waypoint, pred_time):
        super(PossiblePath, self).__init__(name=name)
        self.curr_state = curr_state
        self.mission_waypoint = mission_waypoint
        self.pred_time = pred_time
        
    def setup(self, timeout):
        self.feedback_message = "setup"
        return True
    
    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        
        #Checking is leader velocity in front of vehicle
        possible_path = cond.possible_path(self.curr_state, self.mission_waypoint, self.pred_time)
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
    
################################
## Action ##
################################

class TrackSpeed(py_trees.behaviour.Behaviour):
    def __init__(self, name, curr_state, mission_waypoint, a_max, v_ts, topic_name="wp_planner"):
        super(TrackSpeed, self).__init__(name=name)
        self.curr_state = curr_state
        self.mission_waypoint = mission_waypoint
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
        act.track_speed(self.curr_state, self.mission_waypoint, self.v_ts, self.a_max)
        
        print("STATUS: Vehicle runs in constant velocity")
        print("================== Tick ends, to the next run =====================")
        return py_trees.common.Status.SUCCESS
    
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
        print("STATUS; Vehicle is stop")
        print("================== Tick ends, to the next run =====================")
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        return super().terminate(new_status)
    
class FollowLeader(py_trees.behaviour.Behaviour):
    def __init__(self, name, curr_state, mission_waypoint, waypoint, a_max, topic_name="wp_planner"):
        super(FollowLeader, self).__init__(name=name)
        self.curr_state = curr_state
        self.mission_waypoint = mission_waypoint
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
        success = act.follow_leader(self.curr_state, self.mission_waypoint, self.waypoint, self.a_max)
        if not success:
            print('STATUS: Leader suddenly dissapears')
            print("================== Tick ends, to the next run =====================")
        print("STATUS: Vehicle follows the leader!")
        print("================== Tick ends, to the next run =====================")
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        return super().terminate(new_status)
    
class SwitchLane(py_trees.behaviour.Behaviour):
    def __init__(self, name, curr_state, waypoint_mission, pred_time, a_max, topic_name="wp_planner"):
        super(SwitchLane, self).__init__(name=name)
        self.topic_name = topic_name
        self.curr_state = curr_state
        self.mission_waypoint = waypoint_mission
        self.pred_time = pred_time
        self.a_max = a_max
    
    def setup(self, timeout):
        # self.publisher = rospy.Publisher(self.topic_name, Planner, queue_size=1)
        self.feedback_message = "setup"
        return True
    
    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        
        #the switch lane code
        act.switch_lane(self.curr_state, self.mission_waypoint, self.pred_time, self.a_max)
        print("================== Tick ends, to the next run =====================")
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        return super().terminate(new_status)
    
class DecelerateToStop(py_trees.behaviour.Behaviour):
    def __init__(self, name, curr_state, mission_waypoint, a_max, topic_name="wp_planner"):
        super(DecelerateToStop, self).__init__(name=name)
        self.curr_state = curr_state
        self.mission_waypoint = mission_waypoint
        self.a_max = a_max
        self.topic_name = topic_name
    
    def setup(self, timeout):
        # self.publisher = rospy.Publisher(self.topic_name, Planner, queue_size=1)
        self.feedback_message = "setup"
        return True
    
    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        
        #the decelerate to stop code
        act.decelerate_to_stop(
            self.curr_state,
            self.mission_waypoint[-1][0],
            self.mission_waypoint[-1][1],
            self.mission_waypoint[-1][2],
            self.a_max
        )
        
        print("STATUS: Vehicle decelerates to stop")
        print("================== Tick ends, to the next run =====================")
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        return super().terminate(new_status)