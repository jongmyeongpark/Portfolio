#!/usr/bin/env python3
# -- coding: utf-8 --

import numpy as np
from math import pi, cos, sin, tan, sqrt, atan2
import cvxpy

# Parameters
DT = 0.2  # [s] time tick
WB = 3  # [m] wheelbase of vehicle

# Matrix parameters
NX = 4  # x = x, y, v, yaw
NU = 2  # a = [accel, steer]
T = 10  # horizon length

# MPC parameters
R = np.diag([0.01, 0.1])  # input cost matrix
Rd = np.diag([0.01, 0.1])  # input difference cost matrix
Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
Qf = Q  # state final matrix
MAX_TIME = 60.0  # max simulation time

# Iterative paramter
MAX_ITER = 3  # Max iteration
DU_TH = 0.1  # iteration finish param

N_IND_SEARCH = 10  # Search index number

MAX_STEER = 40 * pi /180
MIN_STEER = -40 * pi /180
MAX_DSTEER = np.deg2rad(5.0)  # maximum steering speed [rad/s]

MAX_SPEED = 30/3.6
MIN_SPEED = 0
MAX_ACCEL = 0.5

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

                 
def update_state(state, a, delta):

    if delta >= MAX_STEER:
        delta = MAX_STEER
    elif delta <= -MAX_STEER:
        delta = -MAX_STEER

    state.x = state.x + state.v * cos(state.yaw) * DT
    state.y = state.y + state.v* sin(state.yaw) * DT
    state.yaw = state.yaw + state.v / WB * tan(delta) * DT
    state.v = state.v + a * DT

    if state.v > MAX_SPEED:
        state.v = MAX_SPEED
    elif state.v < MIN_SPEED:
        state.v = MIN_SPEED

    return state

def angle_mod(x, zero_2_2pi=False, degree=False):

    if isinstance(x, float):
        is_float = True
    else:
        is_float = False

    x = np.asarray(x).flatten()
    if degree:
        x = np.deg2rad(x)

    if zero_2_2pi:
        mod_angle = x % (2 * np.pi)
    else:
        mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

    if degree:
        mod_angle = np.rad2deg(mod_angle)

    if is_float:
        return mod_angle.item()
    else:
        return mod_angle
    
def pi_2_pi(angle):
    return angle_mod(angle)

def get_linear_model_matrix(v, phi, delta):

    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = DT * cos(phi)
    A[0, 3] = - DT * v * sin(phi)
    A[1, 2] = DT * sin(phi)
    A[1, 3] = DT * v * cos(phi)
    A[3, 2] = DT * tan(delta) / WB

    B = np.zeros((NX, NU))
    B[2, 0] = DT
    B[3, 1] = DT * v / (WB * cos(delta) ** 2)

    C = np.zeros(NX)
    C[0] = DT * v * sin(phi) * phi
    C[1] = - DT * v * cos(phi) * phi
    C[3] = - DT * v * delta / (WB * cos(delta) ** 2)

    return A, B, C

def get_nparray_from_matrix(x):
    return np.array(x).flatten()

def calc_nearest_index(state, cx, cy, cyaw, pind):

    dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    min_d = min(d)
 
    ind = d.index(min_d) + pind

    min_d = sqrt(min_d)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - atan2(dyl, dxl))
    if angle < 0:
        min_d *= -1

    return ind, min_d

def predict_motion(x0, oa, od, xref):
    xbar = xref * 0.0
    for i, _ in enumerate(x0):
        xbar[i, 0] = x0[i]

    state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
    for (ai, di, i) in zip(oa, od, range(1, T + 1)):
        state = update_state(state, ai, di)
        xbar[0, i] = state.x
        xbar[1, i] = state.y
        xbar[2, i] = state.v
        xbar[3, i] = state.yaw

    return xbar

def linear_mpc_control(xref, xbar, x0, dref):
    """
    linear mpc control

    xref: reference point
    xbar: operational point
    x0: initial state
    dref: reference steer angle
    """

    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))

    cost = 0.0
    constraints = []

    for t in range(T):
        cost += cvxpy.quad_form(u[:, t], R)

        if t != 0:
            cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)

        A, B, C = get_linear_model_matrix(
            xbar[2, t], xbar[3, t], dref[0, t])
        constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

        if t < (T - 1):
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
            constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <=
                            MAX_DSTEER * DT]

    cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)

    constraints += [x[:, 0] == x0]
    constraints += [x[2, :] <= MAX_SPEED]
    constraints += [x[2, :] >= MIN_SPEED]
    constraints += [cvxpy.abs(u[0, :]) <= MAX_ACCEL]
    constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]
    
    # PID로 얻은 스티어로 직진 구간 판단 및 좌회전 우회전 판단 & 좌회전 또는 우회전 시 과도한 최적화 방지

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.ECOS, verbose=False, warm_start=True)

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = get_nparray_from_matrix(x.value[0, :])
        oy = get_nparray_from_matrix(x.value[1, :])
        ov = get_nparray_from_matrix(x.value[2, :])
        oyaw = get_nparray_from_matrix(x.value[3, :])
        oa = get_nparray_from_matrix(u.value[0, :])
        odelta = get_nparray_from_matrix(u.value[1, :])

    else:
        # print("Error: Cannot solve mpc..")
        oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

    return oa, odelta, ox, oy, oyaw, ov


def iterative_linear_mpc_control(xref, x0, dref, oa, od):
    """
    MPC control with updating operational point iteratively
    """
    ox, oy, oyaw, ov = None, None, None, None

    if oa is None or od is None:
        oa = [0.0] * T
        od = [0.0] * T

    for i in range(MAX_ITER):
        xbar = predict_motion(x0, oa, od, xref)
        poa, pod = oa[:], od[:]
        oa, od, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref)
        try:
            du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
            if du <= DU_TH:
                break
        except:
            oa, od = [0.0] * T, [0.0] * T
        

    return oa, od, ox, oy, oyaw, ov

def calc_ref_trajectory(state, cx, cy, cyaw, sp, dl, pind):
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)

    if pind >= ind:
        ind = pind
        
    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp
    # xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    dref[0, 0] = 0.0  # steer operational point should be 0

    travel = 0.0

    for i in range(1, T + 1):
        travel += abs(state.v) * DT
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            xref[2, i] = sp
            # xref[2, i] = sp[ind + dind]
            xref[3, i] = cyaw[ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = sp
            # xref[2, i] = sp[ncourse - 1]
            xref[3, i] = cyaw[ncourse - 1]
            dref[0, i] = 0.0

    return xref, ind, dref

def smooth_yaw(yaw):

    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= pi / 2.0:
            yaw[i + 1] -= pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -pi / 2.0:
            yaw[i + 1] += pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw

class MPC:
    def __init__(self):
        self.state = State()
        self.target_ind = 0
        self.odelta, self.oa = None, None
        
    def update_erp_state(self, x, y, v, yaw):
        self.state.x = x
        self.state.y = y
        self.state.v = v/3.6
        self.state.yaw = yaw
          
    def activate(self, cx, cy, cyaw, sp, dl=0.5):
             
        if self.state.yaw - cyaw[0] >= pi:
            self.state.yaw -= pi * 2.0
        elif self.state.yaw - cyaw[0] <= -pi:
            self.state.yaw += pi * 2.0

        cyaw = smooth_yaw(cyaw)
        
        odelta, oa = self.odelta, self.oa
        self.target_ind = 0
        
        xref, self.target_ind, dref = calc_ref_trajectory(
                self.state, cx, cy, cyaw, sp/3.6, dl, self.target_ind)

        x0 = [self.state.x, self.state.y, self.state.v, self.state.yaw]

        oa, odelta, ox, oy, oyaw, ov = iterative_linear_mpc_control(
                xref, x0, dref, oa, odelta)
        
        di, ai = 0.0, 0.0
        if oa is not None or odelta is not None:
            di, ai = odelta[0], oa[0]
            steer = di
            speed = self.state.v + ai
            self.odelta, self.oa = odelta, oa
            return speed*3.6, -steer, ox, oy
        
        else:
            speed = self.state.v + self.oa[1]
            steer = self.odelta[1]
            return speed*3.6, -steer, ox, oy