#!/usr/bin/env python
# -*-coding:utf-8-*-

# Python packages
import rospy
import sys, os
import math
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from path_planning_tracking import Path_Tracking
from path_planning_tracking_rrtstar import Path_Tracking_RRTS
from path_planning_tracking_pure_pursuit import Path_Tracking_PP
from path_planning_tracking_dwa_PP import Path_Tracking_DWA
from path_planning.path_planning_tracking_dwa_PID_MPC import Path_Tracking_DWA_MPC
from path_planning_tracking_pure_pursuit_rrtstar import Path_Tracking_RRTS_Pure_Pursuit
# from sub_erp_state import sub_erp_state
from pure_pursuit_added_stanley import Path_Tracking_PP_Stanley

"""
mode 0 : TrajectoryPlanner pure pursuit
mode 1 : rrt* stanley method(PID control)
mode 2 : dwa pure pursuit(PID control)
mode 3 : rrt* pure pursuit(PID control)
mode 4 : dwa pure pursui(PID + MPC)
"""

# mode = 0
# mode = 1
# mode = 2
# mode = 3
mode = 4

# noinspection PyPep8Naming
class mission_cruising:
    def __init__(self, filename, file=0, gp=None):
        self.PT = None

        # ↓↓ original ↓↓
        # self.cr_PT = Path_Tracking(filename, file)

        # ↓↓   test   ↓↓
        self.cr_PT = Path_Tracking_PP_Stanley(filename, file, gp=gp)
        # self.cr_PT = Path_Tracking_PP(filename, file)

        if mode == 0:
            self.PT = Path_Tracking(filename, file)
            self.done = False
        elif mode == 1:
            self.PT = Path_Tracking_RRTS(filename, file)
            self.done = False
        elif mode == 2:
            self.PT = Path_Tracking_DWA(filename, file)
            self.done = False
        elif mode == 3:
            self.PT = Path_Tracking_RRTS_Pure_Pursuit(filename,file)
        elif mode == 4:
            self.PT = Path_Tracking_DWA_MPC(filename,file)
        else:
            pass

    def path_tracking(self, pose, heading):
        steer = self.cr_PT.gps_tracking(pose, heading)
        return steer

    def path_tracking_reverse(self, pose, heading):
        steer = self.cr_PT.gps_tracking_reverse(pose, heading)
        return steer

    def static_obstacle(self, pose, heading, obs, dwa_mode=0,s=0, q=0):
        steer = 0
        if mode == 0:
            steer = self.PT.gps_tracking(pose, heading, obs, path_len=4, ld=8, path_num=5)
        elif mode == 1:
            steer = self.PT.gps_tracking(pose, heading, obs)
        elif mode == 2:
            steer = self.PT.gps_tracking(pose, heading, obs, mode=dwa_mode)
        elif mode == 3:
            steer = self.PT.gps_tracking(pose, heading, obs)
        elif mode == 4:
            steer = self.PT.gps_tracking(pose, heading, obs, mode=dwa_mode, s=s, q=q)
        else:
            pass

        return steer

    def path_tracking_parking(self, pose, heading):
        steer = self.cr_PT.gps_tracking(pose, heading)
        
        # steer = self.cr_PT.gps_tracking_parking(pose, heading, path_num = 1)
        return steer