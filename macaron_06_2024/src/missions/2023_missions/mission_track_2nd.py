#!/usr/bin/env python
# -*-coding:utf-8-*-

import os, sys
import rospy

import numpy as np

sys.path.append(os.path.dirname(
    os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))) + "/src/sensor")
sys.path.append(os.path.dirname(
    os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))) + "/src/path_planning")

from scipy.spatial import distance

from global_path import GlobalPath
from macaron_5.msg import erp_write
from sub_erp_state import sub_erp_state
from path_planning_tracking_dwa_PP import Path_Tracking_DWA

Start = False
Middle = False
Finish = False

class Record_Map:
    def __init__(self):
        self.recent_pose = [0, 0]
        self.dot_distance = 0.25  # 0.5m
        self.trace = np.empty((1, 2))
        self.map = np.empty((1,2))

    def rec_pose(self, pose):
        if np.hypot(self.recent_pose[0] - pose[0], self.recent_pose[1] - pose[1]) >= self.dot_distance:
            trace = np.append(trace, np.array([[pose[0], pose[1]]]), axis=0)

            dst = distance.euclidean(trace[1], trace[-1])
            print('distance', dst)
            self.recent_pose = pose
            
            if not Start:
                Start = True
                trace = np.delete(trace, 0, axis=0)
                
    def update_map(self):
        pass

class Track_Cruising:
    def __init__(self, filename, file=0):
        self.PT = Path_Tracking_DWA(filename, file)


    def path_tracking(self, pose, heading):
        steer = self.cr_PT.gps_tracking(pose, heading)
        return steer

    def static_obstacle(self, pose, heading, obs):
        steer = self.PT.gps_tracking(pose, heading, obs)
        return steer

