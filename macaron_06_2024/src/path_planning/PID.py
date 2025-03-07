#!/usr/bin/env python3
# -- coding: utf-8 --

import time

# message file

class Integral_control(): 
    def __init__(self, time):
        self.I_value = 0
        self.Ki = 0.5
        self.time = time
    
    def I_control(self, error):
    
        self.I_value += error * self.time
        if error <= 0:
            self.I_value = 0
        if self.I_value >= 2:
            self.I_value = 2
        elif self.I_value <= -2:
            self.I_value = -2

        return self.Ki * self.I_value

class Differential_control:
    def __init__(self, time):
        self.last_speed = 0
        self.Kd = 2
        self.time = time
        self.tau = 0.1
        self.last_d = 0
        
    def D_control(self, error):
        if -7 < error  < 7:
            self.last_speed = error
            return 0
        
        D = (error - self.last_speed) / self.time
        
        if D >= 8:
            D = 8
        elif D <= -8:
            D = -8
            
        D_value = (self.tau * self.last_d + self.time * D) / (self.tau + self.time)
        self.last_speed = error
        self.last_d = D_value
        
        return self.Kd * D
    
class PID:
    def __init__(self):
        self.PID_I = Integral_control(0.1)
        self.PID_D = Differential_control(0.1)
        self.Kp = 0.95
        
        
    def PID_control(self, target_speed, current_speed):
        P_speed = self.Kp * (target_speed - current_speed) + current_speed        
        if target_speed - current_speed > 7:
            P_speed += 5
        
        if current_speed == 0 or abs(current_speed - target_speed) <= 7 :
            I_speed = self.PID_I.I_control(0)
            D_speed = self.PID_D.D_control(0)
        else:
            I_speed = self.PID_I.I_control(target_speed - current_speed)
            D_speed = self.PID_D.D_control(target_speed - current_speed)
            
        speed = P_speed + I_speed + D_speed

        return speed
           