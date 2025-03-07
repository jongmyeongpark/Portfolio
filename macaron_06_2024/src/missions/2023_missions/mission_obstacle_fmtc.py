#!/usr/bin/env python
# -*-coding:utf-8-*-

# Python packages
import rospy
import sys, os
import math
import numpy as np
from sensor_msgs.msg import PointCloud2, PointCloud

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))+"/path_planning")

from path_planning_tracking import Path_Tracking
from path_planning_tracking_rrtstar import Path_Tracking_RRTS
from path_planning_tracking_pure_pursuit import Path_Tracking_PP
from path_planning_tracking_dwa_PP import Path_Tracking_DWA
from path_planning_tracking_pure_pursuit_rrtstar import Path_Tracking_RRTS_Pure_Pursuit
# from sub_erp_state import sub_erp_state
from pure_pursuit_added_stanley import Path_Tracking_PP_Stanley

"""
mode 0 : TrajectoryPlanner pure pursuit
mode 1 : rrt* stanley method(PID control)
mode 2 : dwa pure pursuit(PID control)
mode 3 : rrt* pure pursuit(PID control)
"""

# mode = 0
# mode = 1
mode = 2
# mode = 3

RECORD_NUM = 20


# noinspection PyPep8Naming
class mission_obstacle_fmtc:
    def __init__(self, filename="bonseon.npy", file=0, gp=None):
        self.PT = None

        # ↓↓ original ↓↓
        # self.cr_PT = Path_Tracking(filename, file)

        # ↓↓   test   ↓↓
        self.cr_PT = Path_Tracking_PP_Stanley(filename, file, gp=gp)
        
        # self.cr_PT = Path_Tracking_PP(filename, file)
        
        self.moving_avg = np.zeros((RECORD_NUM,1))
        self.a = 0 
        self.mean_points_sub=rospy.Subscriber('/mean_points',PointCloud,self.mean_points_callback,queue_size=1)
        self.mean_points_sub2=rospy.Subscriber('/mean_points2',PointCloud,self.mean_points_callback,queue_size=1)


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
        else:
            pass
    
    def mean_points_callback(self, input_rosmsg):

        self.dynamic_obs = []
        self.static_obs = []

        for p in input_rosmsg.points:
            
            if p.z == 0:
                self.static_obs.append([p.x, p.y, 0])
            else:
                self.dynamic_obs.append([p.x, p.y, 0])
        
        dy = len(self.dynamic_obs)
        st = len(self.static_obs)
        
        print("dy : ",dy)
        print("st : ",st)

        for i in range(RECORD_NUM -1, -1, -1):
            self.moving_avg[i] = self.moving_avg[i-1]
        
        if dy == 0:
            self.moving_avg[0] = [0]
        else:
            self.moving_avg[0] = [1]

        self.a = np.mean(self.moving_avg, axis = 0)*2

        print("a: ",self.a)


    def path_tracking(self, pose, heading):
        steer = self.cr_PT.gps_tracking(pose, heading)
        return steer

    def path_tracking_reverse(self, pose, heading):
        steer = self.cr_PT.gps_tracking_reverse(pose, heading)
        return steer

    def static_obstacle(self, pose, heading, obs, dwa_mode=0):
        steer = 0
        if mode == 0:
            steer = self.PT.gps_tracking(pose, heading, obs, path_len=4, ld=8, path_num=5)
        elif mode == 1:
            steer = self.PT.gps_tracking(pose, heading, obs)
        elif mode == 2:
            steer = self.PT.gps_tracking(pose, heading, obs, mode=dwa_mode)
        elif mode == 3:
            steer = self.PT.gps_tracking(pose, heading, obs)
        else:
            pass
        
        if self.a > 0.6:
            speed = 0
        else:
            speed = 50

        return steer, speed
    

    def path_tracking_parking(self, pose, heading):
        steer = self.cr_PT.gps_tracking(pose, heading)
        
        # steer = self.cr_PT.gps_tracking_parking(pose, heading, path_num = 1)
        return steer
    
if __name__ == "__main__":
    rospy.init_node("sdf")
    mission = mission_obstacle_fmtc()
    rospy.spin()
    
# steer, speed = Mission_obstacle_fmtc.static_obstacle(erp.pose, erp.heading, erp.obs, dwa_mode=1)
