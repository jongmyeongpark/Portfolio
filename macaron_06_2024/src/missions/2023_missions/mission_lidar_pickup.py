#!/usr/bin/env python
#-*-coding:utf-8-*-

# Python packages
import rospy
import sys, os
import math
import time
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/path_planning")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/sensor")
from global_path import GlobalPath
from path_planning_tracking import Path_Tracking
from delivery_sign_loc import delivery_sign_loc as del_sign

# DV_PATH_NAME = "mission_delivery_path.npy"
# 갈아타게 되는 경로
DV_PATH_NAME = "kcity_pickup.npy"
# DV_PATH_NAME = "kcity_pickup.npy"

PATH_ROOT=(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/"
gv_name = PATH_ROOT + DV_PATH_NAME
DP = GlobalPath(gv_name) # find 와 check 에 사용되는 경로
DP2 = GlobalPath(gv_name)


class mission_pickup:

    def __init__(self):
        self.recive = False
        self.stop = False
        self.turn = False
        self.give = False
        self.speed_ok = False
        self.PT_change = Path_Tracking("kcity_pickup.npy")
        # self.DP_na = GlobalPath(self.PT_change)


        #[A표지판, A의 S좌표]
        self.info_A = [0, 0.0]
        self.info_B = [0, 0, 0, 0.0, 0.0, 0.0]

        self.count_A = [0, 0, 0]
        self.pre_s = 0
        self.time_rec = 0
        self.dv_path = Path_Tracking(DV_PATH_NAME)
        self.sign_xy = del_sign(2) ##
        
        

    def find_A(self, traffic_sign, current_s):
        print("traffic_sign ", traffic_sign)
        self.sign_xy.run_a()
        # sign_A = self.sign_xy.pickup_s
        # s = sign_A
        sign_A = self.sign_xy.pickup_loc[0]
        print("sign_A : ", sign_A)
        print("count_A :", self.count_A)
        s, q = DP.xy2sl(sign_A[0], sign_A[1], mode = 1)

        # if (1 in traffic_sign[0:3]):
        #     if (traffic_sign[0] == 1):
        #         self.count_A[0] += 1
        #     if(traffic_sign[1] == 1):
        #         self.count_A[1] += 1
        #     if(traffic_sign[2] == 1):
        #         self.count_A[2] += 1

        # if (s - self.info_A[1] >= -3): #-1,5
        self.info_A[0] = int(self.count_A.index(max(self.count_A))) + 1            

        #print("count_A: ", self.count_A)

        self.info_A[1] = s
        
        print("info_A: ",self.info_A)
        #self.info_A[0] 이 A1,2,3 인지 들어있는거임.


    def change_lane(self, pose, heading):
        steer = self.dv_path.gps_tracking(pose, heading, path_len = 3, ld = 6)
        speed = 60
        return speed, steer


    def run_a(self, pose, heading, traffic_sign, obs, current_s):
                          
        # speed = 60
        # steer = 0
        speed, steer = self.change_lane(pose, heading)
        st_del = 0
        s,q = DP2.xy2sl(pose[0], pose[1])
        print("curr s : ", s)
        self.sign_xy.update(obs)
        self.sign_xy.pub_vis_sign()
        
        # 아직 A표지판 인식 못함
        # print("not yet")
        if (not self.speed_ok and not self.recive):
            # steer = self.dv_path.gps_tracking(pose, heading, [[0.0, 0.0]], 4, 8)
            if not self.stop:
                print("break....")

                print("curr s : ", current_s)
                print("pre_s : ", self.pre_s)
                if (1 in traffic_sign[0:3]):
                    if (traffic_sign[0] == 1):
                        self.count_A[0] += 1
                    if(traffic_sign[1] == 1):
                        self.count_A[1] += 1
                    if(traffic_sign[2] == 1):
                        self.count_A[2] += 1

                print(self.count_A)
                self.find_A(traffic_sign, s)


            if self.info_A[1] > 0:
                # st_del = 1
                if (self.time_rec == 0):
                    self.pre_s = current_s
                    self.time_rec = 1
                if (current_s - self.pre_s > 4): #10
                    print(self.speed_ok)
                    self.speed_ok = True
                    self.time_rec = 0
                
                if(- 2.2 <= s - self.info_A[1] < 2.0 and not self.stop):
                    self.time_rec = time.time()
                    print("recive!!!!")
                    self.stop = True
                elif(time.time() - self.time_rec <= 8 and self.stop):
                    print(self.info_A)
                    speed = -201
                    print("stop !!!!")
                elif(time.time() - self.time_rec > 8 and self.stop):
                    self.time_rec = 0
                    self.stop = False
                    self.recive = True
                    self.speed_ok = False
                else:
                    print("reciving....")
        
        # A 표지판 인식했고 아직 정지안함
        elif (self.speed_ok and not self.recive):

            #배달경로로 갈아타기        
            
            # print("now change course")
            self.find_A(traffic_sign, s)
            
            #self.find_A(['a3'], obs)


            #2.2
            if(s - self.info_A[1] >= -2.2 and not self.stop):
                self.time_rec = time.time()
                print("recive!!!!")
                self.stop = True
            elif(time.time() - self.time_rec <= 8 and self.stop):
                print(self.info_A)
                speed = -201
                print("stop !!!!")
            elif(time.time() - self.time_rec > 8 and self.stop):
                self.time_rec = 0
                self.stop = False
                self.recive = True
                self.speed_ok = False
            else:
                print("reciving....")

        if (self.recive):
            st_del = 1
            print("pickup 미션 끝!!!")
            st_del = 2
            # 다시 전역경로로 복귀

        return speed, steer, st_del


    def toss_a(self):
        set_a = self.info_A[0]
        print(set_a)
        return set_a
