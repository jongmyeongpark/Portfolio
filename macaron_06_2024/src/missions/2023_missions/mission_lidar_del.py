#!/usr/bin/env python
#-*-coding:utf-8-*-

import rospy
import sys, os
import math
import numpy as np
import time
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/path_planning")

from path_planning_tracking import Path_Tracking
from mission_parking_old import two_dis, find_ind

# 파라미터
obs_check_len = 3.8
stop_dis = 0.5 # 주행코스와 내 차와의 거리
restart_time = 6

class mission_del_new():
    def __init__(self):
        self.mission_np = np.load("/home/macaron/catkin_ws/src/macaron_5/path/npy_file/path/mh_stair_in.npy")
        self.PT = Path_Tracking("mh_stair_in.npy")
        
        self.steer = 0
        
        self.state = "not recieved"
        self.done = False
        self.stop = False
        self.current_time = 0
        self.done = False

    def change(self, pose, heading): #change course
        steer = self.PT.gps_tracking(pose, heading, path_len = 3, ld = 6)
        speed = 40
        return speed, steer

    def check_sign(self, pose, obs):
        global stop_dis, restart_time
        # self.steer = self.PT.gps_tracking(pose, heading)
        
        for p in obs:
            # c 는 벽에 잡히는걸로 제한되는걸 방지한것
            c = math.sqrt((p[0]-pose[0]) ** 2 + (p[1]-pose[1]) ** 2)
            # print(c)
            # # p 는 라이다에 나오는 장애물
            if c < 3.8:
                print(c)
                print(self.state)
                n = find_ind(p, self.mission_np) # 라이다로 잡힌 장애물이 check line 에서 몇번째 index 인지 확인
                # n2 = find_ind(p, self.mission_np2)
                # n 은 check line 에서의 장애물 index
                print("1 : ", two_dis(p, self.mission_np[n]))
                print("2 : ",two_dis(pose, self.mission_np[n]))
                if (two_dis(p, self.mission_np[n]) < obs_check_len) and (two_dis(pose, self.mission_np[n]) < stop_dis):
                #if lidar change input become 0 then stop
                # if (two_dis(pose, self.mission_np2[n2]) < stop_dis):
                    # print(two_dis(pose, self.mission_np2[n2]))
                    # time.sleep(2)
                    self.state = "recieved"
                    print(self.state)
                    return self.state

        # print("for문 끝")



        if not self.stop:
            self.current_time = time.time()
            self.stop = True

        if self.stop and time.time() - self.current_time > restart_time:
            self.done = True
        self.state = "recieved"
        print(self.state)
        return self.state

        # if self.state == "recieved":
        #     #5초동안 멈춤 후 출발
        #     if self.stop == False:
        #         self.current_time = time.time()
        #         self.stop = True
        #         print("stop")
        #     if time.time() - self.current_time > restart_time:
        #         self.stop = False
        #         self.done = True
    
'''
state.py 예시 코드

# steer = Mission_cruising.path_tracking(erp.pose, erp.heading)
# speed = 40
# if Mission_pickup.check_sign(erp.pose, erp.obs) == "recieved":
#     speed = -201
#     MS.stop = True

# if Mission_pickup.done:
#     MS.mission_done()
'''
    
    