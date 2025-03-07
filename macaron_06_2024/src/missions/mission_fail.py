#!/usr/bin/env python3
#-*-coding:utf-8-*-

import rospy
from math import sqrt

from geometry_msgs.msg import Point


class MissionFail():
    def __init__(self):

        self.fail_flag = 0
        self.fail_state = False

    
    def check(self, isEstop, isAuto):

        fail_flag = self.fail_flag
        if fail_flag == 0 and not isEstop and isAuto:
            self.fail_flag = 1
        elif fail_flag == 1 and isEstop and isAuto:
            self.fail_flag = 2
        elif fail_flag == 2 and isEstop and not isAuto:
            self.fail_flag = 3
        elif fail_flag == 3 and not isEstop and not isAuto:
            self.fail_flag = 4

        if self.fail_flag == 4:
            
            self.fail_state = True

    def simple_fail_check(self, isEstop, isAuto):
        if isAuto and isEstop:
            self.fail_flag = 1

        if self.fail_flag == 1:
            self.fail_state = True

    
    def check_fail(self):
        if self.fail_state:
            self.fail_state = False
            self.fail_flag = 0
            return True
        
        elif not self.fail_state:
            return False

            



    