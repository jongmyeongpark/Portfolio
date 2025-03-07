#!/usr/bin/env python
# -*-coding:utf-8-*-
"""
Path tracking simulation with Stanley steering control and PID speed control.
author: Atsushi Sakai (@Atsushi_twi)
Ref:
    - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
    - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)
"""
import rospy
import numpy as np
import os, sys
from std_msgs.msg import Int64, Float64
import time
from math import *

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))

K = 3  # control gain
# K = 5  # control gain 평행주차
dt = 0.1  # [s] time difference
L = 1.5    # [m] Wheelbase of vehicle
max_steer = np.radians(27.0)  # [rad] max steering angle
MAX_STEER = 2000
MIN_STEER = -2000


class LowPassFilter(object):
    def __init__(self, cut_off_freqency=5.0, ts=0.1):
        # cut_off_freqency: 차단 주파수
        # ts: 주기

        self.ts = ts
        self.cut_off_freqency = cut_off_freqency
        self.tau = self.get_tau()

        self.prev_data = 30.0

    def get_tau(self):
        return 1 / (2 * np.pi * self.cut_off_freqency)

    def filter(self, data):
        val = (self.ts * data + self.tau * self.prev_data) / (self.tau + self.ts)
        self.prev_data = val
        return val


class State:
    """
    Class representing the state of a vehicle.
    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.lpf = LowPassFilter()

    def update(self, x, y, heading, erp_speed):
        """
        Update the state of the vehicle.
        Stanley Control uses bicycle model.
        :param acceleration: (float) Acceleration
        :param delta: (float) Steerin
        """
        self.x = x
        self.y = y
        self.yaw = heading
        self.yaw = self.yaw % (2.0 * np.pi)
        self.v = erp_speed
        # self.v = self.filter(erp_speed)
    

class Stanley:
    def __init__(self):
        self.target_speed = 10.0 / 3.6  # [m/s]
        self.state = State()
        self.ind = 0
        self.k = K
        # self.curvature_pub = rospy.Publisher("curvature", Float64, queue_size=1)

    # noinspection PyMethodMayBeStatic
    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    # noinspection PyMethodMayBeStatic
    def calc_target_index(self, state, cx, cy):
        """
        Compute index in the trajectory list of the target.
        :param state: (State object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """
        # Calc front axle position
        fx = state.x + L * np.cos(state.yaw)
        fy = state.y + L * np.sin(state.yaw)

        # Search nearest point index
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                        -np.sin(state.yaw + np.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle

    def stanley_control(self, x, y, heading, q_error, erp_speed, rx, ry, ryaw):
        """
        Stanley steering control.
        :param state: (State object)
        :param cx: ([float])
        :param cy: ([float])
        :param cyaw: ([float])
        :param last_target_idx: (int)
        :return: (float, int)
        """

        self.state.update(x, y, heading, erp_speed / 3.6)

        current_target_idx, cross_track_error = self.calc_target_index(self.state, rx, ry)
        # current_target_idx += 2
        # self.curvature_pub.publish(rk[current_target_idx])
        
        # theta_e corrects the heading error

        try:
            theta_e = self.normalize_angle(ryaw[current_target_idx+2] - self.state.yaw)
            # theta_e = self.normalize_angle(avg_ryaw - self.state.yaw)
        except IndexError:
            theta_e = self.normalize_angle(ryaw[-1] - self.state.yaw)
        # theta_d corrects the cross track error
        cross_track_error = (cross_track_error + q_error) * 0.5
        theta_d = np.arctan2(self.k * cross_track_error, self.state.v)
        # Steering control
        delta = theta_e + theta_d
        # print(self.k)
        target_steer = np.rad2deg(delta) * 71

        if target_steer > MAX_STEER:
            target_steer = MAX_STEER
        if target_steer < MIN_STEER:
            target_steer = MIN_STEER

        return -target_steer

    def modified_stanley_control(self, x, y, heading, erp_speed, rx, ry, ryaw):
        self.state.update(x, y, heading, erp_speed / 3.6)
        current_target_idx, _ = self.calc_target_index(self.state, rx, ry)

        # theta_e corrects the heading error
        try:
            # Track 전용
            # additional_idx = min(max(erp_speed / 5, 10), 20)
            
            # Cruising 전용
            additional_idx = min(max(erp_speed * 10 / 8 + 5, 15), 50)  # (speed : 70 ~ 170) -> (index : 10 ~ 20) 50 100
            
            theta_e = self.normalize_angle(ryaw[int(current_target_idx + additional_idx)] - self.state.yaw)
            # print(theta_e)
        except IndexError:
            theta_e = self.normalize_angle(ryaw[-1] - self.state.yaw)

        return -np.rad2deg(theta_e) * 71

    def modified_stanley_control_reverse(self, x, y, heading, erp_speed, rx, ry, ryaw):
        self.state.update(x, y, heading, erp_speed)
        current_target_idx, _ = self.calc_target_index(self.state, rx, ry)

        # theta_e corrects the heading error
        try:
            # Track 전용
            # additional_idx = min(max(erp_speed / 5, 10), 20)
            
            # Cruising 전용
            additional_idx = min(max(erp_speed / 8 - 1.6667, 10), 21)  # (speed : 70 ~ 170) -> (index : 10 ~ 20) 50 100
            
            theta_e = self.normalize_angle(ryaw[int(current_target_idx - additional_idx)] - self.state.yaw)
        except IndexError:
            theta_e = self.normalize_angle(ryaw[0] - self.state.yaw)

        return -np.rad2deg(theta_e) * 71


class StanleyPID:
    def __init__(self, t):
        self.last_q = 0
        self.I_value = 0
        self.time = t

    def D_control(self, q):
        D_value = (q - self.last_q) / self.time

        self.last_q = q
        return D_value

    def I_control(self, q):
        if self.I_value * q <= 0 or abs(q) <= 0.3:
            self.I_value = 0
        self.I_value += q * self.time

        if self.I_value >= 2.0:
            self.I_value = 2.0
        elif self.I_value <= -2.0:
            self.I_value = -2.0

        return self.I_value