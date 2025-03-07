#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy
import os, sys
import numpy as np
import time
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/path_planning")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/src/path_planning")

from sensor_msgs.msg import PointCloud
from macaron_6.msg import erp_write
from path_planning_tracking import Path_Tracking
from path_planning_tracking_pure_pursuit import Path_Tracking_PP
from pure_pursuit_added_stanley import Path_Tracking_PP_Stanley
from trajectory_planner import TrajectoryPlanner
from global_path import GlobalPath

from planning_parking_parallel import Parking_Parallel
from planning_parking_parallel_2nd import Parking_Parallel_2nd
from planning_parking_parallel_3arc import Parking_Parallel_3arc
from planning_parking_parallel_3arc_2nd import Parking_Parallel_3arc_2nd
from sub_erp_state import sub_erp_state
from mission_cruising import *
'''
MODE 설명
MODE0 = 원호2개로 만드는 경로, 미션 시작 점과 마지막 라바콘과 같은 s값을 기준으로 진행.
MODE1 = 원호3개로 만드는 경로. 미션 시작 점과 첫번째 라바콘과 같은 s값을 기준으로 진행.
MODE2 = 원호2개로 만드는 경로. 미션 시작 점과 마지막 주차위치와 같은 s값을 기준으로 진행.
MODE3 = 원호4개로 만드는 경로. 미션 시작 점과 마지막 라바콘과 같은 s값을 기준으로 하지만, 출차 경로가 후진경로 reverse가 아니라 4번째 원을 따라 출차.

MODE1으로 하고 반지름 1.8~2가 중앙선 안넘어가고 적당.

주차 공간 확정하기 전까지는 속도 높여서 달리다가 주차 공간 확정한다음 속도 줄이는 방향으로 생각해보자.

'''
MODE = 1
# DGU 경로이름
def path1_set_up(start_point, stop_point):
    if MODE == 0:
        parking_parallel = Parking_Parallel(start_point, stop_point, A=0.8, d=3)
    elif MODE == 1:
        parking_parallel = Parking_Parallel_3arc(start_point, stop_point, A=1.8, d=2.8)
    elif MODE == 2:
        parking_parallel = Parking_Parallel_2nd(start_point, stop_point, A=0.8, d=3)
    elif MODE == 3:
        parking_parallel = Parking_Parallel_3arc_2nd(start_point, stop_point, A=1.2, d=3)
    path1, path2, path3, path4 = parking_parallel.final_path()
    parking_parallel.save_path2npy(path1, 'parking_parallel_path1_1')    
    parking_parallel.save_path2npy(path2, 'parking_parallel_path1_2')
    parking_parallel.save_path2npy(path3, 'parking_parallel_path1_3')
    parking_parallel.save_path2npy(path4, 'parking_parallel_path1_4')

def path2_set_up(start_point, stop_point):
    if MODE == 0:
        parking_parallel = Parking_Parallel(start_point, stop_point, A=0.8, d=3)
    elif MODE == 1:
        parking_parallel = Parking_Parallel_3arc(start_point, stop_point, A=1.8, d=2.8)
    elif MODE == 2:
        parking_parallel = Parking_Parallel_2nd(start_point, stop_point, A=0.8, d=3)
    elif MODE == 3:
        parking_parallel = Parking_Parallel_3arc_2nd(start_point, stop_point, A=1.2, d=3)
    path1, path2, path3, path4 = parking_parallel.final_path()
    parking_parallel.save_path2npy(path1, 'parking_parallel_path2_1')
    parking_parallel.save_path2npy(path2, 'parking_parallel_path2_2')
    parking_parallel.save_path2npy(path3, 'parking_parallel_path2_3')
    parking_parallel.save_path2npy(path4, 'parking_parallel_path2_4')
    
def path3_set_up(start_point, stop_point):
    if MODE == 0:
        parking_parallel = Parking_Parallel(start_point, stop_point, A=0.8, d=3)
    elif MODE == 1:
        parking_parallel = Parking_Parallel_3arc(start_point, stop_point, A=1.8, d=2.8)
    elif MODE == 2:
        parking_parallel = Parking_Parallel_2nd(start_point, stop_point, A=0.8, d=3)
    elif MODE == 3:
        parking_parallel = Parking_Parallel_3arc_2nd(start_point, stop_point, A=1.2, d=3)
    path1, path2, path3, path4 = parking_parallel.final_path()
    parking_parallel.save_path2npy(path1, 'parking_parallel_path3_1')
    parking_parallel.save_path2npy(path2, 'parking_parallel_path3_2')
    parking_parallel.save_path2npy(path3, 'parking_parallel_path3_3')
    parking_parallel.save_path2npy(path4, 'parking_parallel_path3_4')

# fmtc 경로이름
# def path1_set_up(start_point, stop_point):
#     if MODE == 0:
#         parking_parallel = Parking_Parallel(start_point, stop_point, A=0.8, d=3)
#     elif MODE == 1:
#         parking_parallel = Parking_Parallel_3arc(start_point, stop_point, A=1.8, d=2.8)
#     elif MODE == 2:
#         parking_parallel = Parking_Parallel_2nd(start_point, stop_point, A=0.8, d=3)
#     elif MODE == 3:
#         parking_parallel = Parking_Parallel_3arc_2nd(start_point, stop_point, A=1.2, d=3)
#     path1, path2, path3, path4 = parking_parallel.final_path()
#     parking_parallel.save_path2npy(path1, 'parking_parallel_path1_1')    
#     parking_parallel.save_path2npy(path2, 'parking_parallel_path1_2')
#     parking_parallel.save_path2npy(path3, 'parking_parallel_path1_3')
#     parking_parallel.save_path2npy(path4, 'parking_parallel_path1_4')

# def path2_set_up(start_point, stop_point):
#     if MODE == 0:
#         parking_parallel = Parking_Parallel(start_point, stop_point, A=0.8, d=3)
#     elif MODE == 1:
#         parking_parallel = Parking_Parallel_3arc(start_point, stop_point, A=1.8, d=2.8)
#     elif MODE == 2:
#         parking_parallel = Parking_Parallel_2nd(start_point, stop_point, A=0.8, d=3)
#     elif MODE == 3:
#         parking_parallel = Parking_Parallel_3arc_2nd(start_point, stop_point, A=1.2, d=3)
#     path1, path2, path3, path4 = parking_parallel.final_path()
#     parking_parallel.save_path2npy(path1, 'parking_parallel_path2_1')
#     parking_parallel.save_path2npy(path2, 'parking_parallel_path2_2')
#     parking_parallel.save_path2npy(path3, 'parking_parallel_path2_3')
#     parking_parallel.save_path2npy(path4, 'parking_parallel_path2_4')
    
# def path3_set_up(start_point, stop_point):
#     if MODE == 0:
#         parking_parallel = Parking_Parallel(start_point, stop_point, A=0.8, d=3)
#     elif MODE == 1:
#         parking_parallel = Parking_Parallel_3arc(start_point, stop_point, A=1.8, d=2.8)
#     elif MODE == 2:
#         parking_parallel = Parking_Parallel_2nd(start_point, stop_point, A=0.8, d=3)
#     elif MODE == 3:
#         parking_parallel = Parking_Parallel_3arc_2nd(start_point, stop_point, A=1.2, d=3)
#     path1, path2, path3, path4 = parking_parallel.final_path()
#     parking_parallel.save_path2npy(path1, 'parking_parallel_path3_1')
#     parking_parallel.save_path2npy(path2, 'parking_parallel_path3_2')
#     parking_parallel.save_path2npy(path3, 'parking_parallel_path3_3')
#     parking_parallel.save_path2npy(path4, 'parking_parallel_path3_4')

# # k-city 경로이름
# def path1_set_up(start_point, stop_point):
#     if MODE == 0:
#         parking_parallel = Parking_Parallel(start_point, stop_point, A=0.8, d=3)
#     elif MODE == 1:
#         parking_parallel = Parking_Parallel_3arc(start_point, stop_point, A=1.8, d=2.8)
#     elif MODE == 2:
#         parking_parallel = Parking_Parallel_2nd(start_point, stop_point, A=0.8, d=3)
#     elif MODE == 3:
#         parking_parallel = Parking_Parallel_3arc_2nd(start_point, stop_point, A=1.2, d=3)
#     path1, path2, path3, path4 = parking_parallel.final_path()
#     parking_parallel.save_path2npy(path1, 'kcity_parking_parallel_path1_1')    
#     parking_parallel.save_path2npy(path2, 'kcity_parking_parallel_path1_2')
#     parking_parallel.save_path2npy(path3, 'kcity_parking_parallel_path1_3')
#     parking_parallel.save_path2npy(path4, 'kcity_parking_parallel_path1_4')

# def path2_set_up(start_point, stop_point):
#     if MODE == 0:
#         parking_parallel = Parking_Parallel(start_point, stop_point, A=0.8, d=3)
#     elif MODE == 1:
#         parking_parallel = Parking_Parallel_3arc(start_point, stop_point, A=1.8, d=2.8)
#     elif MODE == 2:
#         parking_parallel = Parking_Parallel_2nd(start_point, stop_point, A=0.8, d=3)
#     elif MODE == 3:
#         parking_parallel = Parking_Parallel_3arc_2nd(start_point, stop_point, A=1.2, d=3)
#     path1, path2, path3, path4 = parking_parallel.final_path()
#     parking_parallel.save_path2npy(path1, 'kcity_parking_parallel_path2_1')
#     parking_parallel.save_path2npy(path2, 'kcity_parking_parallel_path2_2')
#     parking_parallel.save_path2npy(path3, 'kcity_parking_parallel_path2_3')
#     parking_parallel.save_path2npy(path4, 'kcity_parking_parallel_path2_4')
    
# def path3_set_up(start_point, stop_point):
#     if MODE == 0:
#         parking_parallel = Parking_Parallel(start_point, stop_point, A=0.8, d=3)
#     elif MODE == 1:
#         parking_parallel = Parking_Parallel_3arc(start_point, stop_point, A=1.8, d=2.8)
#     elif MODE == 2:
#         parking_parallel = Parking_Parallel_2nd(start_point, stop_point, A=0.8, d=3)
#     elif MODE == 3:
#         parking_parallel = Parking_Parallel_3arc_2nd(start_point, stop_point, A=1.2, d=3)
#     path1, path2, path3, path4 = parking_parallel.final_path()
#     parking_parallel.save_path2npy(path1, 'kcity_parking_parallel_path3_1')
#     parking_parallel.save_path2npy(path2, 'kcity_parking_parallel_path3_2')
#     parking_parallel.save_path2npy(path3, 'kcity_parking_parallel_path3_3')
#     parking_parallel.save_path2npy(path4, 'kcity_parking_parallel_path3_4')

class mission_parking_parallel:
    def __init__(self):
        self.step = 0 # 미션 진행 단계
        self.s = 0
        self.end_s = 0
        self.current_time = 0 # 현재 시간
        self.parking_complete = False
        self.done = False
        self.Fail = False

        self.area_width = 0.8
        self.false_cnt = [0, 0, 0]
        
        self.time_start = 0
        self.end_time = 0

        self.parking_speed = [50, -30, 40] 

        # FMTC
        # self.start_point = (931373.4811928119, 1929856.211141192) 
        # self.stop_point = [(931368.8854115973, 1929850.8566882513), 
        #                    (931365.8344909106, 1929847.0650475544), 
        #                    (931362.9788351147, 1929843.327279783)]
        # fmtc 밖 도로 주차 경로
        # self.start_point = (931266.1315912452, 1929879.7824695776)
        # self.stop_point = [(931270.7810106126, 1929885.1919015388),
        #                    (931273.754546666, 1929889.2615204505),
        #                    (931276.771260584, 1929893.1976450197)]
        # K-CITY 원호 3개짜리 szero_point
        # start_point = (935541.5772585942, 1915871.6539429966)
        # stop_point = [(935539.8214537585, 1915868.301122867), 
        #             (935537.5355587602, 1915864.0328956733), 
        #             (935534.9655771592, 1915859.4453523539)]
        # DGU 좌표들.
        path1_set_up(start_point, stop_point[0])
        path2_set_up(start_point, stop_point[1])
        path3_set_up(start_point, stop_point[2])        

        self.PATH_ROOT = os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))+"/path/npy_file/path/"
        
        # FMTC에서 사용할 주차 경로 이름.
        self.path = [['parking_parallel_path1_1.npy', 'parking_parallel_path1_2.npy', 'parking_parallel_path1_3.npy'], 
                     ['parking_parallel_path2_1.npy', 'parking_parallel_path2_2.npy', 'parking_parallel_path2_3.npy'], 
                     ['parking_parallel_path3_1.npy', 'parking_parallel_path3_2.npy', 'parking_parallel_path3_3.npy']]
        
        # K-CITY에서 사용할 주차 경로 이름.
        # self.path = [['kcity_parking_parallel_path1_1.npy', 'kcity_parking_parallel_path1_2.npy', 'kcity_parking_parallel_path1_3.npy', 'kcity_parking_parallel_path1_4.npy'], 
        #              ['kcity_parking_parallel_path2_1.npy', 'kcity_parking_parallel_path2_2.npy', 'kcity_parking_parallel_path2_3.npy', 'kcity_parking_parallel_path2_4.npy'], 
        #              ['kcity_parking_parallel_path3_1.npy', 'kcity_parking_parallel_path3_2.npy', 'kcity_parking_parallel_path3_3.npy', 'kcity_parking_parallel_path3_4.npy']]
        
        self.straight_path = self.path[0][0]
        self.back_path = self.path[0][1]
        self.front_path = self.path[0][2]
        self.out_path = self.path[0][3]
        self.current_path = self.straight_path

        self.parking_area = [False, False, False]
        
        self.PT = Path_Tracking(self.current_path)
        self.PT_PP = Path_Tracking_PP(self.current_path)
        self.GP = GlobalPath(self.PATH_ROOT + self.current_path)
        self.PT_cr = mission_cruising(self.current_path)
        # self.PST = Path_Tracking_PP_Stanley(self.current_path)

    def speed_def(self):
        if self.step == 0:
            speed = self.parking_speed[0]
        elif self.step == 2:
            speed = self.parking_speed[1]
        elif self.step == 2.5:
            speed = 20
        elif self.step == 5:
            speed = self.parking_speed[2]
        else:
            speed = 0
        return speed

    def select_path(self, obs):
        def two_dis(p1, p2):
            a = p1[0] - p2[0]
            b = p1[1] - p2[1]
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
        
        for i in range(len(self.path)):
            path_np = np.load(file = self.PATH_ROOT + self.path[i][1])
            for j in obs:
                n = find_ind(j, path_np)
                if two_dis(j, path_np[n]) < self.area_width or self.false_cnt[i] > 3:
                    self.parking_area[i] = False
                    self.false_cnt[i] += 1
                    break
                self.parking_area[i] = True
            self.update_current_path()

    def update_current_path(self):
        if self.parking_area[0] == True:
            self.straight_path = self.path[0][0]
            self.back_path = self.path[0][1]
            self.front_path = self.path[0][2]
            self.out_path = self.path[0][3]
        elif self.parking_area[0] == False:
            if self.parking_area[1] == True:
                self.straight_path = self.path[1][0]
                self.back_path = self.path[1][1]
                self.front_path = self.path[1][2]
                self.out_path = self.path[1][3]
            elif self.parking_area[1] == False:
                if self.parking_area[2] == True:
                    self.straight_path = self.path[2][0]
                    self.back_path = self.path[2][1]
                    self.front_path = self.path[2][2]
                    self.out_path = self.path[2][3]

    def change_path(self):
        self.update_current_path()
        if self.step == 0 :
            self.current_path = self.straight_path
            self.PT = Path_Tracking(self.current_path)
        elif self.step == 2:
            self.current_path = self.back_path
            self.PT_PP = Path_Tracking_PP(self.current_path)
        elif self.step == 2.5:
            self.current_path = self.front_path
            self.PT_PP = Path_Tracking(self.current_path)
        elif self.step == 5:
            self.current_path = self.out_path
            # self.PT = Path_Tracking(self.current_path)
            # self.PT_PP = Path_Tracking_PP(self.current_path)
            self.PT_cr = mission_cruising(self.current_path)
        self.GP = GlobalPath(self.PATH_ROOT + self.current_path)

    def calculate_end_s(self):
        self.update_current_path()
        if self.step == 0:
            x, y = np.load(file = self.PATH_ROOT + self.straight_path)[-1]
            self.end_s, _ = self.GP.xy2sl(x, y)
        elif self.step == 2:
            x, y = np.load(file = self.PATH_ROOT + self.back_path)[-1]
            self.end_s, _ = self.GP.xy2sl(x, y)
        elif self.step == 2.5:
            x, y = np.load(file = self.PATH_ROOT + self.front_path)[-1]
            self.end_s, _ = self.GP.xy2sl(x, y)
        elif self.step == 5:
            x, y = np.load(file = self.PATH_ROOT + self.out_path)[-1]
            self.end_s, _ = self.GP.xy2sl(x, y)
        else:
            pass

    def update(self, pos, heading, obs):
        self.change_path()
        self.s, _ = self.GP.xy2sl(pos[0], pos[1])
        if self.step == 2:
            steer = self.PT_PP.gps_tracking(pos, heading, path_num=-1, parking=2)
            # steer = self.PT_PP.gps_tracking(pos, heading, parking=2)
        elif self.step == 5:
            # steer = self.PT_PP.gps_tracking(pos, heading, parking=1)
            steer = self.PT_cr.static_obstacle(pos, heading, obs)
        else:
            self.PT.stanley.k = 3
            steer = self.PT.gps_tracking(pos, heading)
            # steer = mission_cruising.path_tracking(pos, heading)
        return steer
        
    def step_finished(self, pos, heading, obs):
        steer = self.update(pos, heading, obs)
        speed = self.speed_def()
        self.calculate_end_s()
        if self.step == 0:
            print('주차 중...')
            print('s === ', self.s)
            print('길이 = ', self.end_s)
            if self.s >= self.end_s - 2:
                self.step += 1
        elif self.step == 1:
                self.step += 1
        elif self.step == 2:
            print('s === ', self.s)
            print('길이 = ', self.end_s)
            print('후진 중')
            if self.s >= self.end_s - 1.2:
                self.step += 0.5
        elif self.step == 2.5:
            if self.s >= self.end_s:
                self.step += 0.5
        elif self.step == 3:
            print('후진 완료')
            self.time_start = time.time()
            self.step += 1
            return [0, 0]
        elif self.step == 4:
            print('정지 중')
            self.end_time = time.time()
            if self.end_time - self.time_start > 5:
                self.step += 1
        elif self.step == 5:
            print('출차 중')
            if self.s >= self.end_s - 1.8:
                self.step += 1
        return speed, steer

    def run(self, pos, heading, obs):
        print(obs)
        self.select_path(obs)
        self.update_current_path()
        try:        
            print('주차공간 == ', self.parking_area)
            mission_ind = self.parking_area.index(True)
            print('최종 주차 공간 번호 == ', mission_ind + 1)
        except:
            print('주차 가능한 구역 없음')
            mission_ind = -1
            
        if mission_ind == -1:
            self.Fail == True
            speed = 50
            steer = 0
            return speed, steer
        else:
            print('step', self.step)
            # if self.step < 5:
            #     speed, steer = self.step_finished(pos, heading, obs)
            #     print('현재 경로 ===== ', self.current_path)
            # else:
            #     speed = self.speed_def()
            #     steer = 0
            #     self.done = True
            # return speed, steer
            if self.step < 6:
                speed, steer = self.step_finished(pos, heading, obs)
                print('현재 경로 ==== ', self.current_path)
            else:
                print('주차 미션 완료')
                speed = self.speed_def()
                steer = 0
                self.done = True
            return speed, steer