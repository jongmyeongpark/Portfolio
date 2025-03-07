#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy
import sys, os
import math
import numpy as np
import time

from sensor_msgs.msg import PointCloud

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/path_planning")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))+"/path/npy_file/path/")

from pure_pursuit_added_stanley import Path_Tracking_PP_Stanley
from path_planning_tracking import Path_Tracking
from path_planning_tracking_pure_pursuit import Path_Tracking_PP
from global_path import GlobalPath
from macaron_6.msg import erp_write
from sub_erp_state import sub_erp_state

### fmtc    ###
# parking_i = [[2, 10, 5],
#              [2, 11.5, 6],
#              [2, 8.5, 2],
#              [3, 10, 0]] # [[(주차시작s), (주차완료s), (후진완료s)]]

### kcity   ###
parking_i = [[3, 10, 5],
             [3, 11.5, 6],
             [3, 8.5, 2],
             [3, 10, 3],
             [3, 8.5, 2],
             [3, 10, 0]] # [[(주차시작s), (주차완료s), (후진완료s)]]

real_sq = [0,0]
min_offset = 0.5
offset = [0,0]

parking_speed = [60, 40, -40, 100] # 주차 위치 확인, 주차 진입, 후진 속도, 크루징 스피드
stay_time = 10

def two_dis(p1, p2):
    a = p1[0] - p2[0] + offset[0]
    b = p1[1] - p2[1] + offset[1]
    c = np.hypot(a, b)
    return c

def find_ind(pose, path):
    d_min = 1000.0
    d_ind = -1
    d_save = 0
    for p in path:
        d_ind += 1
        d = two_dis(p, pose)
        if d < d_min:
            d_save = d_ind
            d_min = d
    return d_save

class mission_parking:
    def __init__(self) -> None:
        self.area_width = 0.5
        self.PATH_ROOT = os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))+"/path/npy_file/path/"

        # self.parking_path = ["snu_add_parking1.npy",
        #                      "snu_add_parking2.npy",
        #                      "snu_add_parking3.npy",
        #                      "snu_add_parking4.npy"]
        self.parking_path = ["parking_KC1.npy",
                             "parking_KC2.npy",
                             "parking_KC3.npy",
                             "parking_KC4.npy",
                             "parking_KC5.npy",
                             "parking_KC6.npy"]
        
        self.map_correct = False

        self.parking_area = [False, False, False, False, False, False]
        self.false_cnt = [0, 0, 0, 0, 0, 0]
        self.parking_state = "Finding"
        self.start_ind = -1

        self.map_check_path = GlobalPath(self.PATH_ROOT + "kcity_parking.npy")
        self.PT_parking = Path_Tracking_PP_Stanley(self.parking_path[0])
        self.park_path = GlobalPath(self.PATH_ROOT + self.parking_path[0])
        
    def map_check(self, obs):
        obs_sq = []
        obs_xy = []
        min_s = 99999999
        
        for i in obs:
            s, q = self.map_check_path.xy2sl(i[0],i[1],mode=1)
            if s < min_s and q <= 0:
                min_s = s
                obs_sq = [s,q]
                obs_xy = i
        
        if obs_sq == []:
            print("No Obs!!!!!!!!!")
            self.map_correct = False
        else:
            if abs(obs_sq[0] - real_sq[0]) > min_offset:
                real_xy = self.map_check_path.sl2xy(real_sq[0],real_sq[1])
                offset = [real_xy[0]-obs_xy[0],real_xy[1]-obs_xy[1]]
            self.map_correct = True
         
    def find_area(self, obs):
        # print("obsssss ", obs)
        for i in range(len(self.parking_path)):
            print(i)
            mission_np = np.load(file = self.PATH_ROOT+self.parking_path[i])
            for p in obs:
                n = find_ind(p, mission_np) #원하는 P index  - - -  - - - mission  /  pose(erp) - - - mission
                if two_dis(p, mission_np[n]) < self.area_width or self.false_cnt[i] > 3:
                    print(two_dis(p, mission_np[n]))
                    self.parking_area[i] = False
                    self.false_cnt[i] += 1
                    break
                self.parking_area[i] = True

    def run(self, s, pose, heading, obs):
        # if self.map_correct is not True:
        #     self.map_check(obs)
        #     steer = "Finding"
        #     return [parking_speed[0], steer]
        
        # pose = [pose[0]+offset[0],pose[1]+offset[1]]
        
        self.find_area(obs)
        
        try:
            print(self.parking_area)
            mission_ind = self.parking_area.index(True)
        except:
            mission_ind = -1

        print("******************mission_ind ", mission_ind)
        print("******************start_ind ", self.start_ind)
        
            
        if mission_ind == -1:
            steer = "Finding"
            return [parking_speed[0], steer]
        else:
            s, _ = self.park_path.xy2sl(pose[0],pose[1])
            print("******************mission s : ",s)

            if parking_i[mission_ind][1] >= s >= parking_i[mission_ind][0] and self.parking_state == "Finding":
                print("=========주차 시작=========")

                self.start_flag = True
                self.start_ind = mission_ind
                self.park_path = GlobalPath(self.PATH_ROOT + self.parking_path[self.start_ind])
                self.PT_parking = Path_Tracking_PP_Stanley(self.parking_path[self.start_ind])
                self.parking_state = "Parking"
                speed, steer = self.parking(pose, heading)

            elif parking_i[mission_ind][1] >= s >= parking_i[mission_ind][0] and self.parking_state == "Parking":
                if mission_ind is not self.start_ind:
                    self.start_ind = mission_ind
                    self.park_path = GlobalPath(self.PATH_ROOT + self.parking_path[self.start_ind])
                    self.PT_parking = Path_Tracking_PP_Stanley(self.parking_path[self.start_ind])
                
                print("=========주차 중=========")
                speed, steer = self.parking(pose, heading)

            elif s >= parking_i[self.start_ind][1] and self.parking_state == "Parking":
                print("=========주차 완료=========")
                self.parking_state = "Waiting"
                return [0,0]

            elif s >= parking_i[self.start_ind][1] and self.parking_state == "Waiting":
                print("=========후진 시작=========")
                rospy.sleep(stay_time)
                parking_reversed = np.transpose(np.flip(np.load(file = self.PATH_ROOT+self.parking_path[self.start_ind]), axis=0))
                self.PT_parking_back = Path_Tracking_PP(parking_reversed, file=1)
                self.parking_state = "Back"
                speed, steer = self.back(pose, heading)
                
            elif s >= parking_i[self.start_ind][2] and self.parking_state == "Back":
                print("=========후진  중=========")
                speed, steer = self.back(pose, heading)
            
            elif s <= parking_i[self.start_ind][2] and self.parking_state == "Back":
                print("=========미션 완료=========")
                self.parking_state = "Done"
                return [-201, 0]
            
            else:
                print(mission_ind)
                print("=========예외 상황=========")
                steer = "Finding"
                return [parking_speed[0], steer]
            
            return speed, steer

    def parking(self, pose, heading): # 주차하기
        global parking_speed
        steer = self.PT_parking.gps_tracking(pose, heading)
        speed = parking_speed[1]
        return [speed, steer]
    
    def back(self, pose, heading): # 출차하기
        global parking_speed
        steer = self.PT_parking_back.gps_tracking(pose, heading, parking=1)
        speed = parking_speed[2]
        return [speed, steer]