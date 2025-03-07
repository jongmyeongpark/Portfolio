#!/usr/bin/env python
#-*-coding:utf-8-*-

## 전체 알고리즙 ##
## 크게 두가지 기능이 필요
# 1. 빈 주차 공간을 특정
## -> 필요 없지 않을까?? 그냥 주차공간 1의 경로를 따라가라 하고 경로가 새로 생성(펼쳐지면) 다음 경로 (주차공간 2의 경로)로 경로를 덮어씌워버리는 방식은 어떨까? 
##    대신 장애물정보(obs 좌표)의 잔상을 남기는거지. 경로가 업데이트 될 때마다 초기화 해주고.
# 2. 주차공간에 정확히 들어감
# 3. 그대로 다시 나와서 미션 종료

# Python packages
import rospy
import sys, os
import math
import numpy as np

from sensor_msgs.msg import PointCloud

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/path_planning")

from path_planning_tracking import Path_Tracking

# 파라미터
parking_width = 2.0/2
# parking_width = 0.7
parking_i = [[10, 12, 19], [10, 12, 19]] # 레코딩 시작, 스캔 종료, 주차 종료값///앞은 팔정도, 뒤는 kcity
# parking_i = [[10, 15, 15], [10, 15, 15]] # 레코딩 시작, 스캔 종료, 주차 종료값
parking_speed = [50, -50] # 주차 진입, 후진 속도
stay_time = 12

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

class mission_parking():
    def __init__(self, Where):
        global parking_i
        self.WHERE = Where
        self.parking_file_ind = 1
        if self.WHERE == 1:
            self.PT_parking = Path_Tracking("parking_KC%d.npy"%(self.parking_file_ind))
            self.parking_np = np.load(file = (os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/parking_KC%d.npy"%(self.parking_file_ind))
            self.parking_index = parking_i[0]
        else:
            self.PT_parking = Path_Tracking("parking_KC%d.npy"%(self.parking_file_ind))
            self.parking_np = np.load(file = (os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/parking_KC%d.npy"%(self.parking_file_ind))
            self.parking_index = parking_i[1]

        self.enc_target = 0
        self.rec_st = {}
        self.step = 0
        self.ENC_target_state = False
        self.done = False

    def scan(self, pose, heading, obs): # 빈자리 찾아서 대강 들어가기 단계
        global parking_width, parking_speed
        for p in obs:
            n = find_ind(p, self.parking_np)
            if (two_dis(p, self.parking_np[n]) < parking_width) and (n > 12):
                self.parking_file_ind += 1
                try:
                    if self.WHERE == 1:
                        self.PT_parking = Path_Tracking("parking_KC%d.npy"%(self.parking_file_ind))
                        self.parking_np = np.load(file = (os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/parking_KC%d.npy"%(self.parking_file_ind))
                    else:
                        self.PT_parking = Path_Tracking("parking_KC%d.npy"%(self.parking_file_ind))
                        self.parking_np = np.load(file = (os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/parking_KC%d.npy"%(self.parking_file_ind))
                    print("주차 자리 갱신")
                    self.ENC_target_state = False
                except:
                    print("주차 자리 없음!!")
                    self.parking_file_ind -= 1
                    self.done = True

        steer = self.PT_parking.gps_tracking(pose, heading, path_len = 3, ld = 6)
        speed = parking_speed[0]
        return speed, steer

    def parking(self, pose, heading, lane): # 정확하게 주차하는 단계
        global parking_speed
        # 차선 좌우 맞출 수 있으면 맞추면서 주차
        # if stopline이 일정 조건 안에 오면 or gps로 원하는 좌표에 오면:
        #     self.ENC_start = ENC
        #     self.step = 2
        ## 일단 gps로만 해보자 ##
        steer = self.PT_parking.gps_tracking(pose, heading, path_len = 3, ld = 6)
        speed = parking_speed[0]
        return speed, steer
        
    def back(self, ENC): # 다시 빠져나오는 단계
        global parking_speed
        a = 0
        if ENC > self.enc_target:
            while ((ENC - a) in self.rec_st) is False:
                a += 1
            steer = self.rec_st[ENC - a]
            speed = parking_speed[1]
            return speed, steer
        else:
            self.done = True
            return 0, 0

    def rec_steer(self, steer, ENC):
        if ENC in self.rec_st:
            pass
        else:
            print("rec..."),
            self.rec_st[ENC] = steer

    def reset(self):
        self.parking_file_ind = 1
        if self.WHERE == 1:
            self.PT_parking = Path_Tracking("parking_KC%d.npy"%(self.parking_file_ind))
            self.parking_np = np.load(file = (os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/parking_KC%d.npy"%(self.parking_file_ind))
        else:
            self.PT_parking = Path_Tracking("parking_KC%d.npy"%(self.parking_file_ind))
            self.parking_np = np.load(file = (os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/parking_KC%d.npy"%(self.parking_file_ind))
        self.enc_target = 0
        self.rec_st = {}
        self.step = 0
        self.ENC_target_state = False
        self.done = False

    def run(self, pose, heading, obs, lane, erp_ENC, erp_steer):
        global stay_time

        ### 미션 인덱스 값 ###
        if find_ind(pose, self.parking_np) > self.parking_index[0] and self.ENC_target_state == False: #돌아올 위치 마킹점
            self.enc_target = erp_ENC
            self.ENC_target_state = True
        if find_ind(pose, self.parking_np) > self.parking_index[1] and self.step == 0: # 주차 공간 찾는 인덱스(저 인덱스까지만 공간 스캔) -> 이후로는 주차 공간 갱신 X
            self.step = 1
            print("주차 자리 확정")
        if find_ind(pose, self.parking_np) > self.parking_index[2] and self.step == 1: # 주차 완료 인덱스
            self.step = 2
            print("주차 완료")
            return -201, 0

        if self.step == 0:
            self.rec_steer(erp_steer, erp_ENC)
            speed, steer =  self.scan(pose, heading, obs)
        if self.step == 1:
            self.rec_steer(erp_steer, erp_ENC)
            speed, steer = self.parking(pose, heading, lane)
        if self.step == 2:
            rospy.sleep(stay_time)
            self.step = 3
            print("후진 시작")
        if self.step == 3:
            speed, steer = self.back(erp_ENC)

        return speed, steer