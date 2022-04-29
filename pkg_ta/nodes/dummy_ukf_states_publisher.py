import rospy
import time
import sys
import os
from math import atan2
#sys.path.append(os.path.abspath(sys.path[0] + '/../'))
# from nav_msgs.msg import Odometry
from pkg_ta.msg import ukf_states
from std_msgs.msg import Float32

rospy.init_node('dummy_ukf_states')

pub = rospy.Publisher('/ukf_states', ukf_states, queue_size=1)
freq = 20
rate = rospy.Rate(freq) # Hz

golfi_msg = ukf_states()
golfi_msg.stamp = rospy.Time.now()
last_time = golfi_msg.stamp.to_sec() - 1./freq
golfi_msg.x = 12300
golfi_msg.y = 456
golfi_msg.vx = 0.3
golfi_msg.vy = 0.4
golfi_msg.vx_gnss = 0.31
golfi_msg.vy_gnss = 0.41
golfi_msg.v_gnss = 0.51
golfi_msg.v_tach = 0.5
golfi_msg.yaw_est = 0.2
golfi_msg.yaw_imu = 1.23
golfi_msg.yaw_dydx = 1.24

while not rospy.is_shutdown():
    ### Calculate the actual sampling time
    golfi_msg.stamp = rospy.Time.now()
    pub.publish(golfi_msg)
    rate.sleep()