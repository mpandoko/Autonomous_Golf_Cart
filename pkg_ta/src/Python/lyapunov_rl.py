# This code is written in Python 3 environment

import numpy as np
import datetime as dttm
import time
import sys
import os

########### RL AGENT CLASS ####################
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal

args_num_obs = 4

class ActorNet(nn.Module):
    def __init__(self):
        super(ActorNet, self).__init__()
        self.fc = nn.Linear(args_num_obs, 100)
        self.mu_head = nn.Linear(100, 1)
        self.sigma_head = nn.Linear(100, 1)

    def forward(self, x):
        x = F.relu(self.fc(x))
        mu = 1.0 * F.tanh(self.mu_head(x))
        sigma = F.softplus(self.sigma_head(x))
        return (mu, sigma)

class Agent():
    clip_param = 0.2
    max_grad_norm = 0.5
    ppo_epoch = 10
    buffer_capacity, batch_size = 1000, 32

    def __init__(self):
        self.training_step = 0
        self.anet = ActorNet().float()
        self.buffer = []
        self.counter = 0

    def select_action(self, state):
        state = torch.from_numpy(state).float().unsqueeze(0)
        with torch.no_grad():
            (mu, sigma) = self.anet(state)
        dist = Normal(mu, sigma)
        action = dist.sample()
        action_log_prob = dist.log_prob(action)
        action.clamp(-1.0, 1.0)
        action_return = max(min(action.item(), 1.), -1.)
        return action_return, action_log_prob.item()

    def load_A_param(self, param):
        self.anet.load_state_dict(torch.load(param))
        self.anet.eval()
############## RL Agent class end ##################


# from numba import njit, float64, int64
# from numba.experimental import jitclass

# spec = [('_kp', float64), ('_ki', float64), ('_kd', float64), ('_ff_params', float64[:]),
#         ('_sat_long_max', float64), ('_sat_long_min', float64),
#         ('_ev', float64), ('_ev_last', float64), ('_ev_sum', float64),
#         ('_ev_sum_max', float64), ('_ev_sum_min', float64),
#         ('_ks', float64), ('_kv', float64), ('_l', float64),
#         ('_dead_band_lat', float64), ('_sat_lat_max', float64),
#         ('_sat_lat_min', float64), ('_e_lat', float64), ('_e_yaw', float64),
#         ('_waypoints', float64[:, :]), ('_closest_idx', int64),
#         ('_min_vel_move', float64), ('_max_throttle', float64), ('_min_throttle', float64),
#         ('_kv_yaw', float64), ('_kv_lat', float64),]

# @jitclass(spec)
class Controller(object):
    def __init__(self, kp, ki, kd, feed_forward_params, sat_long,\
                 ks, kv, length, lateral_dead_band, sat_lat,\
                 waypoints, min_vel_move,\
                 max_throttle_move, min_throttle_move, kv_yaw, kv_lat):
        # In this version, the integral term will be clamped based on the
        # saturation value and the feed-forward term

        # The parameters of the longitudinal controller
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._ff_params = feed_forward_params
        self._sat_long_max = max(sat_long[0], sat_long[1])
        self._sat_long_min = min(sat_long[0], sat_long[1])
        self._ev = 0.
        self._ev_last = 0.0
        self._ev_sum = 0.
        self._ev_sum_max = 0. # This value will be updated in each iteration
        self._ev_sum_min = 0. # This value will be updated in each iteration

        # The parameters of the lateral controller
        self._ks = ks
        self._kv = kv
        self._l = length
        self._dead_band_lat = lateral_dead_band
        self._sat_lat_max = np.fmax(sat_lat[0], sat_lat[1])
        self._sat_lat_min = np.fmin(sat_lat[0], sat_lat[1])
        self._e_lat = 0.
        self._e_yaw = 0.

        # Waypoints (n, 5) -> x, y, yaw, v, curvature
        self._waypoints = waypoints
        self._closest_idx = 0

        # Additional Throttle Control (UNTUK TA)
        self._max_throttle = max_throttle_move
        self._min_throttle = min_throttle_move
        self._min_vel_move = min_vel_move
        self._kv_yaw = kv_yaw
        self._kv_lat = kv_lat

        # RL Agent
        self.agent = Agent()
        # agent_param_path = os.path.abspath(sys.path[0] + '/../src/Python/' + "ckpt_3000_ppo_anet_params.pkl")
        agent_param_path = os.path.abspath(__file__ + '/../' + "ckpt_3000_ppo_anet_params.pkl")
        self.agent.load_A_param(agent_param_path)

        # debug purpose
        self._wpidxref = 0
        self._xref = 0.
        self._yref = 0.
        self._yawref = 0.
        self._vref = 0.
        self._cref = 0.
        self._x = 0.
        self._y = 0.
        self._yaw = 0.
        self._v = 0.
        print("Controller class initiated! Type: Lyapunov-RL")

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def reset_integral_derivative(self):
        self._ev_sum = 0.0
        self._ev_last = 0.0

    def get_error(self):
        return self._ev, self._e_lat, self._e_yaw

    def get_closest_index(self):
        return self._closest_idx

    def get_instantaneous_setpoint(self):
        out = np.copy(self._waypoints[self._closest_idx])
        if out[3] <= 0.05:
            None
        else:
            out[3] = np.fmax(out[3] / (1.0 + self._kv_lat*self._e_lat**2 + self._kv_yaw*self._e_yaw**2), self._min_vel_move)
        return out

    def _update_error(self, x, y, v, yaw):
        # Waypoints (n, 5) -> x, y, yaw, v, curvature

        # Find the closest waypoint
        self._closest_idx = np.argmin(np.sum(np.square(self._waypoints[:, :2] - np.array([x, y])), axis=-1))

        # Find the yaw error
        self._e_yaw = self._waypoints[self._closest_idx, 2] - yaw
        self._e_yaw = (self._e_yaw + np.pi) % (2 * np.pi) - np.pi # Wrap the angle to [-pi, pi)

        # Find the lateral or crosstrack error
        if self._closest_idx == 0:
            idx = 1
        else:
            idx = self._closest_idx
        y2 = self._waypoints[idx, 1]
        x2 = self._waypoints[idx, 0]
        y1 = self._waypoints[idx - 1, 1]
        x1 = self._waypoints[idx - 1, 0]
        dy = y2 - y1
        dx = x2 - x1
        c = dx*y1 - dy*x1
        self._e_lat = (dy*x + c - dx*y) \
                        / (np.sqrt(dx**2 + dy**2) + 10**(-32))

        # Find the velocity error
        temp = self._waypoints[self._closest_idx, 3]
        if temp <= 0.1:
            None
        else:
            temp = temp / (1.0 + self._kv_lat*self._e_lat**2 + self._kv_yaw*self._e_yaw**2)
            temp = np.fmax(temp, self._min_vel_move)
        self._ev = temp - v

        # debug purpose
        self._wpidxref = self._closest_idx
        self._xref = self._waypoints[idx, 0]
        self._yref = self._waypoints[idx, 1]
        self._yawref = self._waypoints[self._closest_idx, 2]
        self._vref = self._waypoints[self._closest_idx, 3]
        self._cref = self._waypoints[self._closest_idx, 4]
        self._x = x
        self._y = y
        self._yaw = yaw
        self._v = v

    def _feed_forward_longitudinal(self, v):
        if v < 0.:
            return 0.
        else:
            return self._ff_params[0] * (1. - np.exp(- self._ff_params[1] * v))

    def _feed_forward_lateral(self):
        temp = self._l * self._waypoints[self._closest_idx, -1]
        if np.abs(temp) >= 1.:
            temp = np.sign(temp)
        return np.fmax(np.fmin(np.arcsin(temp), self._sat_lat_max), self._sat_lat_min) # From -pi/2 to pi/2

    # def _feed_forward_lateral(self):
    #     temp = self._l * self._waypoints[self._closest_idx, -1]
    #     return np.fmax(np.fmin(np.arctan(temp), self._sat_lat_max), self._sat_lat_min) # From -pi/2 to pi/2

    def calculate_control_signal(self, dt, x, y, v, yaw):
        # Waypoints (n, 5) -> x, y, yaw, v, curvature
        self._update_error(x, y, v, yaw)

        ##################### LONGITUDINAL CONTROL #######################
        ######### PID Control
        ff_long = self._feed_forward_longitudinal(self._waypoints[self._closest_idx, 3])

        ev_dot = (self._ev - self._ev_last) / dt

        self._ev_sum_max = np.fmax((self._sat_long_max - ff_long)/self._ki, 0.)
        self._ev_sum_min = np.fmin((self._sat_long_min - ff_long)/self._ki, 0.)
        self._ev_sum = self._ev_sum + self._ev * dt
        self._ev_sum = np.fmax(np.fmin(self._ev_sum, self._ev_sum_max), self._ev_sum_min)

        cs_long = ff_long +\
                    self._kp * self._ev +\
                    self._ki * self._ev_sum +\
                    self._kd * ev_dot
        cs_long = np.fmax(np.fmin(cs_long, self._sat_long_max), self._sat_long_min)

        self._ev_last = self._ev

        # Throttle Control
        cs_long = self._waypoints[self._closest_idx, 3]
        if cs_long <= 0.05:
            None
        else:
            cs_long = cs_long / (1.0 + self._kv_lat*self._e_lat**2 + self._kv_yaw*self._e_yaw**2)
            cs_long = np.fmin(np.fmax(cs_long, self._min_throttle), self._max_throttle)

        ###################################################################


        ######################## LATERAL CONTROL ##########################
        ########## STANLEY Controller
        # temp = 0.0
        # if np.abs(self._e_lat) > self._dead_band_lat:
        #     temp = self._e_lat

        # a = self._feed_forward_lateral()
        # b = self._e_yaw
        # c = np.arctan(self._ks * temp / (self._kv + v))
        # d = a + b + c # Use Stanley !
        # cs_lat = max(min(d, self._sat_lat_max), self._sat_lat_min)
        #############################

        ########### LYAPUNOV-RL Controller
        _e_cstrL = self._e_lat
        _e_headL = self._e_yaw
        _vL = v
        _vL = 0.2

        k1 = 0
        # k2 = 34.93900314

        ### varyingGainV1
        # # k2_g1, k2_e1, k2_g2, k2_e2 = 0.5, 0.7, 12.20954626, 0.0001 #0.54339102,  0.59080092, 37.27210872,  0.17824502 #0.29298437,  0.61759203, 20.54114007,  0.21645174 #[3.97991961e-01 6.22594245e-01 3.38306294e+01 6.78729652e-03]
        # # k2_g2 = 11.9723065 # 34.93900314, 5.05381855, 11.9723065, 5.05007289
        # k2 = ((k2_g2-k2_g1)/(k2_e2-k2_e1)*(np.abs(_e_cstrL)-k2_e1)) + k2_g1
        # k2 = max(min(k2, k2_g2), k2_g1)
        ###

        ### varyngGainV4
        _r = 1.7/np.tan(0.4887)
        # if np.abs(_e_cstrL) < 0.1:
        #     _g1, _e1, _g2, _e2 = 15., 0., 10., 0.1
        #     k2 = ( (_g2-_g1)/(_e2-_e1)*(np.abs(_e_cstrL)-_e1) ) + _g1
        # elif np.abs(_e_cstrL) < 0.15:
        #     _g1, _e1, _g2, _e2 = ((2 + 0.15 / _r) * _r), 0.15, 10., 0.1
        #     k2 = ( (_g2-_g1)/(_e2-_e1)*(np.abs(_e_cstrL)-_e1) ) + _g1
        # elif np.abs(_e_cstrL) < _r:
        #     k2 = (2 + np.abs(_e_cstrL) / _r) * _r
        # else:
        #     k2 = (2 + np.abs(_e_cstrL) / _r) * np.abs(_e_cstrL)
        #     if _e_headL * np.sign(_e_cstrL) > -np.pi/2:
        #         k2 = _r
        ###

        ### RL AGENT action
        # v1 (singleInitPose)
        # state = np.array([max(min(np.abs(self._e_yaw)/1, 1.), -1.),
        #                 max(min(-np.abs(self._e_lat)/1, 1.), -1.),
        #                 max(min(v/1, 1.), -1.),
        #                 max(min(np.abs(self._waypoints[self._closest_idx, -1])/1, 1.), -1.)],
        #                 dtype=np.float32)

        # v2 (fit2varyingGainV4)
        state = np.array([max(min(np.abs(self._e_yaw)/1, 1.), -1.),
                        max(min(-np.abs(self._e_lat)/10, 1.), -1.),
                        max(min(0*v/1, 1.), -1.),
                        max(min(0*np.abs(self._waypoints[self._closest_idx, 4])/1, 1.), -1.)],
                        dtype=np.float32)

        agent_action, action_log_prob = self.agent.select_action(state)
        k2 = (agent_action+1)*25
        # k2 = k2 * 1.5
        ###

        wbL = self._l # wheelbase (meter)
        kv = 1.
        _vL = _vL + kv
        saa = 1 if np.abs(_e_headL)<np.deg2rad(5) else (np.sin(_e_headL)/_e_headL)
        _dead_band_lat = 0.025
        cstr_c = 0 if np.abs(_e_cstrL)<(_dead_band_lat) else _e_cstrL
        _sat_lat_max, _sat_lat_min = self._sat_lat_max, self._sat_lat_min

        ### 1st term
        if (k1 == 0):
            a = 0
        else:
            # a = k1 * wbL * (cstr_c**2) / _vL / _e_headL
            _sign = 1 if (np.sign(_e_headL) == 0) else np.sign(_e_headL)
            a = k1 * wbL * (cstr_c**2) / _vL / _sign
        ### 2nd term
        # k2 = 1 if np.abs(_e_cstrL) > 0.5 else 50
        b = k2 * wbL * _e_headL / _vL
        ### 3rd term
        c = wbL * cstr_c * saa #*0.3 / _vL  
        ### Sum
        d = np.arctan(a + b + c)
        cs_lat = max(min(d, _sat_lat_max), _sat_lat_min)

        ###################################################################

        # debug
        print("ehead:{:0.2f} ecstr:{:0.2f} lyap_k2:{:0.2f} lyap_b:{:0.2f} lyap_c:{:0.2f} cs_lat:{:0.2f}"
              .format(self._e_yaw, self._e_lat, k2, b, c, cs_lat))

        return cs_long, cs_lat, k2

# print("Compiling the Controller class ...")
# print("Controller Type: Lyapunov-RL")
# controller = Controller(0.5, 0.1, 0.1, np.array([1., 2.]),
#                         np.array([-1., 1.]), 2.0, 0.1, 2.5,
#                         0.01, np.array([-np.pi/3., np.pi/3.]),
#                         np.random.randn(100, 5), 0.5,
#                         0.2, 0.08, 0.75, 0.75)
# controller.update_waypoints(np.random.randn(100, 5))
# controller.reset_integral_derivative()
# _ = controller.get_error()
# _ = controller.get_instantaneous_setpoint()
# # controller._update_error(0., 0., 1.0, 0.)
# # _ = controller._feed_forward_longitudinal(2.5)
# # _ = controller._feed_forward_lateral()
# _ = controller.calculate_control_signal(0.01, 0., 0., 1.0, 0.)
# print("The Controller class has been compiled !")
#########################################################################################
