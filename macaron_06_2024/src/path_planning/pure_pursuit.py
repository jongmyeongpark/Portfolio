#!/usr/bin/env python
# -- coding: utf-8 --
"""
Path tracking simulation with pure pursuit steering and PID speed control.
author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)
"""
import rospy
import numpy as np
import math
import matplotlib.pyplot as plt
from std_msgs.msg import Int64
import time

from geometry_msgs.msg import Point

# Parameters
k = 0.15  # look forward gain
Lfc = 2.0  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time tick
WB = 1.03  # [m] wheelbase of vehicle
MAX_STEER = 2000
MIN_STEER = -2000

class Visual:
    def __init__(self):
        self.goal_pub = rospy.Publisher("/goal_point", Point, queue_size=1)

    def pub_goal(self, x, y):
        gp = Point()
        gp.x = x
        gp.y = y
        gp.z = 0
        self.goal_pub.publish(gp)


class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw

        self.tx = 0.0
        self.ty = 0.0

    def update(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        # print("yaw: ")
        # print(yaw)


def proportional_control(target, current):  # 급발진 방지
    return Kp * (target - current)

def pure_pursuit_steer_control(state, goal, ld):
    global tx, ty

    v = Visual()
    index = int(round(ld * 2, -1))
    g_dis = 0.0

    while g_dis < ld:
        try:
            tx, ty = goal[0][index], goal[1][index]
        except IndexError:
            tx, ty = goal[0][-1], goal[1][-1]
            break
        g_dis = math.sqrt((tx - state.x) ** 2 + (ty - state.y) ** 2)
        index += 1
        
    v.pub_goal(tx, ty)

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw
    delta = math.atan2(2.0 * WB * math.sin(alpha) / g_dis, 1.0)

    return delta


class PurePursuit:
    def __init__(self):
        self.target_speed = 10.0 / 3.6  # [m/s]
        self.state = State()
        self.ind = 0

    def get_steer_state(self, x, y, heading, ld, goal):
        self.state.update(x, y, heading)
        
        steer = pure_pursuit_steer_control(self.state, goal, ld)

        target_steer = np.rad2deg(steer) * 71
        if target_steer > MAX_STEER:
            target_steer = MAX_STEER
        if target_steer < MIN_STEER:
            target_steer = MIN_STEER
            
        return -target_steer


class PurePursuitPID:
    def __init__(self, time):
        self.last_q = 0
        self.I_value = 0
        self.time = time

    def D_control(self, q):
        D_value = (q - self.last_q) / self.time

        self.last_q = q
        return D_value

    def I_control(self, q):
        if self.I_value * q <= 0 or abs(q) <= 0.3:
            self.I_value = 0
        self.I_value += q * self.time

        # I value 만땅 2로 제한
        if self.I_value >= 2.0:
            self.I_value = 2.0
        elif self.I_value <= -2.0:
            self.I_value = -2.0

        return self.I_value