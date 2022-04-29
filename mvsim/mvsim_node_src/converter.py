#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

from re import X
from sqlite3 import converters
import rospy
import tf
# from rospy.numpy_msg import numpy_msg
# from rospy_tutorials.msg import Floats
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from mvsim.msg import ukf_states, Control

class converter:
    def state(self, data):
        # odom = Odometry()
        state_x = data.pose.pose.position.x
        state_y = data.pose.pose.position.y
        quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]

        vx = data.twist.twist.linear.x
        vy = data.twist.twist.linear.y
        rospy.loginfo('state_x = %s', state_x)
        rospy.loginfo('state_y = %s', state_y)
        rospy.loginfo('yaw = %s', yaw)
        rospy.loginfo('vx = %s', vx)
        rospy.loginfo('vy = %s', vy)


        self.state_msg = ukf_states()
        self.state_msg.stamp = rospy.Time.now()
        self.state_msg.x = state_x
        self.state_msg.y = state_y
        self.state_msg.vx = vx
        self.state_msg.vy = vy
        self.state_msg.vx_gnss = 0.31
        self.state_msg.vy_gnss = 0.41
        self.state_msg.v_gnss = 0.51
        self.state_msg.v_tach = 0.5
        self.state_msg.yaw_est = yaw
        self.state_msg.yaw_imu = 1.23
        self.state_msg.yaw_dydx = 1.24
        self.send_state_ = True
        return "sending state"

    def control(self, data2):


        self.control_msg = Twist()
        self.control_msg.linear.x = data2.action_throttle
        self.control_msg.angular.z = data2.action_steer
        self.send_state_ = False
        return "sending control"


    def __init__(self):
        self.send_state_ = False
        self.send_control_ = False
        rospy.Subscriber('/base_pose_ground_truth', Odometry, self.state)
        rospy.Subscriber('/control_signal', Control, self.control)
        pub_state = rospy.Publisher('/ukf_states', ukf_states, queue_size=1)
        pub_control = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        while not rospy.is_shutdown():
            if self.send_state_:
                pub_state.publish(self.state_msg)
            if self.send_control_:
                pub_control.publish(self.control_msg)

if __name__ == '__main__':
    rospy.init_node('converter')
    try:
        converter = converter()
    except rospy.ROSInterruptException:
        pass
