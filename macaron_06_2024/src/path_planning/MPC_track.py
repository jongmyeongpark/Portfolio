#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
import numpy as np
from math import pi, cos, sin, tan, sqrt, atan2
import cvxpy



# m/s 모델이므로 단위 확인 잘하기

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

class MPC:
    def __init__(self):
        self.state = State()
        self.target_ind = 0
        self.odelta, self.oa = None, None
        self.ox, self.oy, self.oyaw = None, None, None
        self.cx, self.cy, self.cyaw = None, None, None
        self.direction = 0

        # Parameters
        self.DT = 0.1  # [s] time tick
        self.WB = 1.03  # [m] wheelbase of vehicle

        # Matrix parameters
        self.NX = 4  # x = x, y, v, yaw
        self.NU = 2  # a = [accel, steer]
        self.T = 5  # horizon length

        # MPC parameters
        self.R = np.diag([0.01, 0.01])  # input cost matrix
        self.Rd = np.diag([0.01, 1.0])  # input difference cost matrix
        self.Q = np.diag([1.0, 1.0, 0.1, 0.5])  # state cost matrix
        self.Qf = self.Q  # state final matrix

        # Iterative paramter
        self.MAX_ITER = 3  # Max iteration
        self.DU_TH = 0.1  # iteration finish param

        self.N_IND_SEARCH = 10  # Search index number

        self.MAX_STEER = 27 * pi /180
        self.MIN_STEER = -27 * pi /180
        self.MAX_DSTEER = np.deg2rad(25.0)  # maximum steering speed [rad/s]

        self.MAX_SPEED = 20 / 3.6 # 5.55m/s
        self.MIN_SPEED = 0
        self.MAX_ACCEL = self.MAX_SPEED / 10

    def update_erp_state(self, x, y, v, yaw):
        self.state.x = x
        self.state.y = y
        self.state.v = v/3.6    
        self.state.yaw = yaw

    def smooth_yaw(self, yaw):
        for i in range(len(yaw) - 1):
            dyaw = yaw[i + 1] - yaw[i]

            while dyaw >= pi / 2.0:
                yaw[i + 1] -= pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]

            while dyaw <= -pi / 2.0:
                yaw[i + 1] += pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]
        return yaw
    
    def update_state(self, state, a, delta):

        if delta >= self.MAX_STEER:
            delta = self.MAX_STEER
        elif delta <= -self.MAX_STEER:
            delta = -self.MAX_STEER

        state.x = state.x + state.v * cos(state.yaw) * self.DT
        state.y = state.y + state.v* sin(state.yaw) * self.DT
        state.yaw = state.yaw + state.v / self.WB * tan(delta) * self.DT
        state.v = state.v + a * self.DT

        if state.v > self.MAX_SPEED:
            state.v = self.MAX_SPEED
        elif state.v < self.MIN_SPEED:
            state.v = self.MIN_SPEED

        return state
    
    def update_state_track(self, state, a, delta):

        if delta >= self.MAX_STEER:
            delta = self.MAX_STEER
        elif delta <= -self.MAX_STEER:
            delta = -self.MAX_STEER

        state.x = state.x + state.v * cos(state.yaw) * self.DT
        state.y = state.y + state.v* sin(state.yaw) * self.DT
        state.yaw = state.yaw + state.v / self.WB * tan(delta) * self.DT
        state.v = state.v + a * self.DT

        if state.v > self.MAX_SPEED:
            state.v = self.MAX_SPEED
        elif state.v < self.MIN_SPEED:
            state.v = self.MIN_SPEED

        return state

    def angle_mod(self, x, zero_2_2pi=False, degree=False):

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
    
    def pi_2_pi(self, angle):
        return self.angle_mod(angle)

    def det_direction_from_ck(self, ck, last_cy):
        """
         1 : 우회전
        -1 : 좌회전
         0 : 직진
        """
        curve_radius = 1 / np.average(ck)
        if abs(curve_radius) >= 50:
            self.direction = 0
        elif last_cy <= -1:
            self.direction = 1
        elif last_cy >= 1:
            self.direction = -1

    def det_direction_from_topic(self, direction):
        """
         1 : 우회전
        -1 : 좌회전
         0 : 직진
        """
        self.direction = direction

    def get_nparray_from_matrix(self, x):
        return np.array(x).flatten()
    
    def get_linear_model_matrix(self, v, phi, delta):
        DT = self.DT
        A = np.zeros((self.NX, self.NX))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = DT * cos(phi)
        A[0, 3] = - DT * v * sin(phi)
        A[1, 2] = DT * sin(phi)
        A[1, 3] = DT * v * cos(phi)
        A[3, 2] = DT * tan(delta) / self.WB

        B = np.zeros((self.NX, self.NU))
        B[2, 0] = DT
        B[3, 1] = DT * v / (self.WB * cos(delta) ** 2)

        C = np.zeros(self.NX)
        C[0] = DT * v * sin(phi) * phi
        C[1] = - DT * v * cos(phi) * phi
        C[3] = - DT * v * delta / (self.WB * cos(delta) ** 2)

        return A, B, C
    
    def calc_nearest_index(self, state, cx, cy, cyaw, pind):

        dx = [state.x - icx for icx in cx[pind:(pind + self.N_IND_SEARCH)]]
        dy = [state.y - icy for icy in cy[pind:(pind + self.N_IND_SEARCH)]]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        min_d = min(d)
    
        ind = d.index(min_d) + pind

        min_d = sqrt(min_d)

        dxl = cx[ind] - state.x
        dyl = cy[ind] - state.y

        angle = self.pi_2_pi(cyaw[ind] - atan2(dyl, dxl))
        if angle < 0:
            min_d *= -1

        return ind, min_d

    def calc_ref_trajectory(self, state, cx, cy, cyaw, sp, dl, pind):
        xref = np.zeros((self.NX, self.T + 1))
        dref = np.zeros((1, self.T + 1))
        ncourse = len(cx)

        ind, _ = self.calc_nearest_index(state, cx, cy, cyaw, pind)

        if pind >= ind:
            ind = pind
            
        xref[0, 0] = cx[ind]
        xref[1, 0] = cy[ind]
        xref[2, 0] = sp
        # xref[2, 0] = sp[ind]
        xref[3, 0] = cyaw[ind]
        dref[0, 0] = 0.0  # steer operational point should be 0

        travel = 0.0

        for i in range(1, self.T + 1):
            travel += abs(state.v) * self.DT
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
    
    def predict_motion(self, x0, oa, od, xref):
        xbar = xref * 0.0
        for i, _ in enumerate(x0):
            xbar[i, 0] = x0[i]

        state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
        for (ai, di, i) in zip(oa, od, range(1, self.T + 1)):
            state = self.update_state(state, ai, di)
            xbar[0, i] = state.x
            xbar[1, i] = state.y
            xbar[2, i] = state.v
            xbar[3, i] = state.yaw

        return xbar
    
    def linear_mpc_control(self, xref, xbar, x0, dref):
        """
        linear mpc control

        xref: reference point
        xbar: operational point
        x0: initial state
        dref: reference steer angle
        """
        T = self.T
        x = cvxpy.Variable((self.NX, T + 1))
        u = cvxpy.Variable((self.NU, T))

        cost = 0.0
        constraints = []

        for t in range(T):
            cost += cvxpy.quad_form(u[:, t], self.R)

            if t != 0:
                cost += cvxpy.quad_form(xref[:, t] - x[:, t], self.Q)

            A, B, C = self.get_linear_model_matrix(
                xbar[2, t], xbar[3, t], dref[0, t])
            constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

            if t < (T - 1):
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], self.Rd)
                constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <=
                                self.MAX_DSTEER * self.DT]

        cost += cvxpy.quad_form(xref[:, T] - x[:, T], self.Qf)

        constraints += [x[:, 0] == x0]
        constraints += [x[2, :] <= self.MAX_SPEED]
        constraints += [x[2, :] >= self.MIN_SPEED]
        constraints += [cvxpy.abs(u[0, :]) <= self.MAX_ACCEL]
        constraints += [cvxpy.abs(u[1, :]) <= self.MAX_STEER]
        # if self.direction == 0:
        #     constraints += [cvxpy.abs(u[1, :]) <= self.MAX_STEER * 0.2]
        # elif self.direction == -1:
        #     constraints += [u[1, 0] >= -self.MAX_STEER * 0.1]
        #     pass
        # elif self.direction == 1:
        #     constraints += [u[1, 0] <= self.MAX_STEER * 0.1]

        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        prob.solve(solver=cvxpy.OSQP)

        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            ox = self.get_nparray_from_matrix(x.value[0, :])
            oy = self.get_nparray_from_matrix(x.value[1, :])
            ov = self.get_nparray_from_matrix(x.value[2, :])
            oyaw = self.get_nparray_from_matrix(x.value[3, :])
            oa = self.get_nparray_from_matrix(u.value[0, :])
            odelta = self.get_nparray_from_matrix(u.value[1, :])

        else:
            print("Error: Cannot solve mpc..")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

        return oa, odelta, ox, oy, oyaw, ov
    
    def iterative_linear_mpc_control(self, xref, x0, dref, oa, od):
        """
        MPC control with updating operational point iteratively
        """
        ox, oy, oyaw, ov = None, None, None, None
        T = self.T
        if oa is None or od is None:
            oa = [0.0] * T
            od = [0.0] * T

        for i in range(self.MAX_ITER):
            xbar = self.predict_motion(x0, oa, od, xref)
            poa, pod = oa[:], od[:]
            oa, od, ox, oy, oyaw, ov = self.linear_mpc_control(xref, xbar, x0, dref)
            try:
                du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
                if du <= self.DU_TH:
                    break
            except:
                oa, od = [0.0] * T, [0.0] * T

        return oa, od, ox, oy, oyaw, ov

    def activate(self, cx, cy, cyaw, sp, dl=0.5):
        
        cyaw = self.smooth_yaw(cyaw)
        self.cx, self.cy, self.cyaw = cx, cy, cyaw

        if self.state.yaw - cyaw[0] >= pi:
            self.state.yaw -= pi * 2.0
        elif self.state.yaw - cyaw[0] <= -pi:
            self.state.yaw += pi * 2.0
            
        odelta, oa = self.odelta, self.oa

        sp = sp / 3.6
        xref, self.target_ind, dref = self.calc_ref_trajectory(
                self.state, cx, cy, cyaw, sp, dl, self.target_ind)

        x0 = [self.state.x, self.state.y, self.state.v, self.state.yaw]

        oa, odelta, ox, oy, oyaw, ov = self.iterative_linear_mpc_control(
                xref, x0, dref, oa, odelta)

        return oa, odelta, ox, oy, oyaw, ov
