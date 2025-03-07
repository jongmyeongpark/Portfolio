#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy
import os, sys
import numpy as np
import time

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/path_planning")

from path_planning_tracking import Path_Tracking
from trajectory_planner import TrajectoryPlanner
from global_path import GlobalPath
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from data_hub import SensorDataHub


class mission_parallel:
    def __init__(self):
        
        self.step = 0  # step 초기값
        self.done = False #미션 완료됐는지 아닌지 확인
        self.complete = False
        
        # need mapping
        ### 밑에 직선 경로 + 후진 경로 수정
        self.path_straight_name = "kcity_parking_straight.npy" # path tracking 함수에만
        self.path_backward_name = "kcity_parking_backward.npy"  # path tracking 함수에만
        self.path_straight = "/home/macaron/catkin_ws/src/macaron_5/path/npy_file/path/kcity_parking_straight.npy" # global path 함수에만
        self.path_backward = "/home/macaron/catkin_ws/src/macaron_5/path/npy_file/path/kcity_parking_backward.npy"  # global path 함수에만
        self.PT = Path_Tracking(self.path_straight_name) # 일단 처음에는 직선경로
        self.GP = GlobalPath(self.path_straight)  # 일단 처음에는 직선경로
        

        self.parking_speed = [-30, 30]
        
        # 첫번째는 straight 경로 끝나는 s좌표 (직선 경로 기준 s좌표)
        # 두번째는 backward 경로 끝나는 s좌표 (후진 경로 기준 s좌표)
        # self.parking_location = [28, 10, 1000]
        
    # def parking(self):  # 주차하기
    #     self.speed = self.parking_speed[0]
        
    #     if self.s > self.parking_location[1]:
    #         self.step = 2

    def change_path(self): # 직선 경로 -> 후진 경로 변경
        self.PT = Path_Tracking(self.path_backward_name)  # 후진 경로!!
        self.GP = GlobalPath(self.path_backward)
        time.sleep(3)
        self.step = 2

    def check_path(self):
        # self.cord = PointCloud()
        real_len = SensorDataHub.rubber_len_check()
        if real_len < 0.5: #test 값
            self.step = 1
            time.sleep(4)
            self.change_path()
 
    # def path_finished(self): # straight 경로에서만
    #     if self.s > self.parking_location[0]:  # s좌표 기준은 kcity에서 최적화
    #         self.step = 1 # 주차 단계로 넘어감
    #         self.change_path() # 후진 경로로 변경
            
    def stop(self):
        if self.complete == False:
            self.current_time = time.time()
            self.complete = True
            
        if self.complete and time.time() - self.current_time < 12:
            # self.speed = -201 # 주차 완료 후 정지
            self.speed = 0

        if time.time() - self.current_time > 12:
            self.complete = False
            self.speed = self.parking_speed[1]
            self.done = True
                   

        
    def update(self, pose, heading):  # 현재 위치 정보들을 업데이트
        self.pose = pose
        self.heading = heading
        self.s, _ = self.GP.xy2sl(self.pose[0], self.pose[1])  # 현재 경로 기준 s 좌표
        self.steer = self.PT.gps_tracking(self.pose, self.heading)  # 현재 경로를 tracking
        
    def run(self, pose, heading):
        self.update(pose, heading)
        
        if self.step == 0:
            self.speed = 80
            self.check_path()
            print("Step 0 :", self.s)
        elif self.step == 1:
            # self.parking()
            self.change_path()
            print("Step 1 :", self.s)
        elif self.step == 2:
            print("Step 2 :", self.s)
            self.stop()
        
        return self.speed, self.steer    

'''
state.py 예시 코드
Mission_parking_parallel = mission_parking_parallel() # 객체 선언

.....(중간 생략).....

# (elif 문에서)
speed, steer = Mission_parking_parallel.run(erp.pose, erp.heading)

if Mission_parking_parallel.done:
    MS.mission_done()

'''