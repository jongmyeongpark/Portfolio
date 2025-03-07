#!/usr/bin/env python3
# -- coding: utf-8 --

class Integral_control():
    def __init__(self, time):
        self.I_value = 0
        self.Ki = 0.25
        self.time = time
    
    def I_control(self, error):
        
        self.I_value += error * self.time
        if error <= -5:
            self.I_value = -5
        if self.I_value >= 10:
            self.I_value = 10

        return self.Ki * self.I_value

class Differential_control:
    def __init__(self, time):
        self.last_speed = 0
        self.Kd = 2
        self.time = time
        self.tau = 0.1
        self.last_d = 0
        
    def D_control(self, error):
        D = (error - self.last_speed) / self.time
        
        if D >= 40:
            D = 40
        elif D <= -40:
            D = -40
            
        D_value = (self.tau * self.last_d + self.time * D) / (self.tau + self.time)
        self.last_speed = error
        self.last_d = D_value
        
        return self.Kd * D
    
class PIDController:
    def __init__(self):
        self.PID_I = Integral_control(0.1)
        self.PID_D = Differential_control(0.1)
        self.Kp = 0.5
        
        
    def pid_control(self, target_speed, current_speed):
        if target_speed >= 0:
            P_speed = self.Kp * (target_speed - current_speed) + current_speed
            return P_speed
            # else:
            #     P_speed = self.Kp * (target_speed - current_speed)
            # I_speed = self.PID_I.I_control(target_speed - current_speed)
            # D_speed = self.PID_D.D_control(target_speed - current_speed)
            # speed = I_speed + P_speed + D_speed
            # print(I_speed,"I_speed")
            # 속도를 많이 줄여야 한다면
            # if target_speed - current_speed < -10:
            #     speed = 0
            #     brake = 1.0
            # return P_speed