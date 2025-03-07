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
from delivery2_sign_loc import delivery_sign_loc as del_sign

B_plus_threshold = 13 # B_plus 이상 인지 누적 이후 급감 구역 탐색
B_minus_threshold = 5 # B_minus 이상 급감시 해당 s좌표 저장

# DV_PATH_NAME = "mission_delivery_path.npy"
DV_PATH_NAME = "kcity_del.npy"

PATH_ROOT=(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/"
gv_name = PATH_ROOT + DV_PATH_NAME
DP = GlobalPath(gv_name) # find 와 check 에 사용되는 경로
DP2 = GlobalPath(gv_name) # run 에 사용되는 경로

class mission_deliver:

    def __init__(self):
        self.recive = False
        self.stop = False
        self.turn = False
        self.give = False
        self.speed_ok = False

        self.info_A = [0, 0.0]
        self.info_B = [0, 0, 0, 0.0, 0.0, 0.0]

        self.loc_B = [0.0, 0.0, 0.0]
        self.order_B = [0, 0, 0]
        self.count_A = [0, 0, 0]
        self.count_B = [0, 0, 0]
        self.pre_s = 0
        self.time_rec = 0
        self.dv_path = Path_Tracking(DV_PATH_NAME)
        self.sign_xy = del_sign(2) ##
        
        self.bbox = [0, 0, 0]
        self.order = [False, # b1 < b2
                      False, # b1 > b2
                      False, # b2 < b3
                      False, # b2 > b3
                      False, # b1 < b3
                      False] # b1 > b3
        self.ordered = False
        self.decided = False
        self.sign_a = 0

    # def check_B_sign(self, traffic_sign):
        
    #     #if(1 in traffic_sign[3:6]):
            
    #     if(traffic_sign[3] == 1):
    #         self.info_B[0] += 1
    #         self.count_B[0] = 0
    #     elif(traffic_sign[4] == 1):
    #         self.info_B[1] += 1
    #         self.count_B[1] = 0
    #     elif(traffic_sign[5] == 1):
    #         self.info_B[2] += 1
    #         self.count_B[2] = 0

    #     if(self.info_B[0] > B_plus_threshold ):
    #         print("info_B[0] : ", self.info_B[0])
    #         if (traffic_sign[3] != 1):
    #             self.count_B[0] += 1
    #     if(self.info_B[1] > B_plus_threshold ):
    #         print("info_B[1] : ", self.info_B[1])
    #         if (traffic_sign[4] != 1):
    #             self.count_B[1] += 1
    #     if(self.info_B[2] > B_plus_threshold ):
    #         print("info_B[2] : ", self.info_B[2])
    #         if (traffic_sign[5] != 1):
    #             self.count_B[2] += 1
            
    #     if(self.count_B[0] > B_minus_threshold):
    #         if (self.order_B[1] == 0 and self.order_B[2] == 0 ):
    #             self.order_B[0] = 1
    #         elif (self.order_B[1] != 0 and self.order_B[2] != 0 ):
    #             self.order_B[0] = 3
    #         else:
    #             self.order_B[0] = 2
    #             self.order_B[self.order_B.index(0)] = 3
    #         self.info_B[0] = -999
    #         self.count_B[0] = 0
    #     if(self.count_B[1] > B_minus_threshold):
    #         if (self.order_B[0] == 0 and self.order_B[2] == 0 ):
    #             self.order_B[1] = 1
    #         elif (self.order_B[0] != 0 and self.order_B[2] != 0 ):
    #             self.order_B[1] = 3
    #         else:
    #             self.order_B[1] = 2
    #             self.order_B[self.order_B.index(0)] = 3
    #         self.info_B[1] = -999
    #         self.count_B[1] = 0
    #     if(self.count_B[2] > B_minus_threshold):
    #         if (self.order_B[0] == 0 and self.order_B[1] == 0 ):
    #             self.order_B[2] = 1
    #         elif (self.order_B[0] != 0 and self.order_B[1] != 0 ):
    #             self.order_B[2] = 3
    #         else:
    #             self.order_B[2] = 2
    #             self.order_B[self.order_B.index(0)] = 3
    #         # self.info_B[2] = -999
    #         self.count_B[2] = 0

    #     if (not 0 in self.order_B ):
    #         self.info_B[3] = self.loc_B[self.order_B[0]-1]
    #         self.info_B[4] = self.loc_B[self.order_B[1]-1]
    #         self.info_B[5] = self.loc_B[self.order_B[2]-1]

    #     print("info_B : ", self.info_B)            

    def check_B_sign(self, traffic_sign, bbox):
        if(traffic_sign[3] == 1):
            self.info_B[0] += 1
            self.count_B[0] = 0
            
        elif(traffic_sign[4] == 1):
            self.info_B[1] += 1
            self.count_B[1] = 0
            
        elif(traffic_sign[5] == 1):
            self.info_B[2] += 1
            self.count_B[2] = 0

        if not self.ordered:
            print(self.order)
            print(bbox)
                
            # self.order = [False,  # b1 < b2
            #               False,  # b1 > b2
            #               False,  # b2 < b3
            #               False,  # b2 > b3
            #               False,  # b1 < b3
            #               False]  # b1 > b3

            if self.order[0] and self.order[2]:  # b1 < b2 < b3
                self.order_B = [3, 2, 1]
                self.ordered = True
            if self.order[3] and self.order[4]:  # b1 < b3 < b2
                self.order_B = [2, 3, 1]
                self.ordered = True
            if self.order[1] and self.order[4]:  # b2 < b1 < b3
                self.order_B = [3, 1, 2]
                self.ordered = True
            if self.order[2] and self.order[5]:  # b2 < b3 < b1
                self.order_B = [1, 3, 2]
                self.ordered = True
            if self.order[0] and self.order[5]:  # b3 < b1 < b2
                self.order_B = [2, 1, 3]
                self.ordered = True
            if self.order[1] and self.order[3]:  # b3 < b2 < b1
                self.order_B = [1, 2, 3]
                self.ordered = True

            sign = traffic_sign[3:6]
            count = 0
            
            for i in range(3):
                if len(bbox[i]) > 1:
                    self.bbox[i] = sum(bbox[i]) / len(bbox[i])
                    print("avg ", sum(bbox[i]) / len(bbox[i]))
                    count += 1
            
            print("count ", count)
            if count < 2:
                self.bbox = [0, 0, 0]      
                return

            # order = [[], [], []]
            
            # for i, x in enumerate(self.bbox):
            #     order[i] = [x, i + 1]

            # order = sorted(order, reverse=True)
            # self.order_B = [order[0][1], order[1][1], order[2][1]]

            # self.ordered = True
            
            if not self.order[0] and not self.order[1] and not 0 in self.bbox[:2]:
                if self.bbox[0] < self.bbox[1]:
                    self.order[0] = True
                elif self.bbox[0] > self.bbox[1]:
                    self.order[1] = True
            
            if not self.order[2] and not self.order[3] and not 0 in self.bbox[1:3]:
                if self.bbox[1] < self.bbox[2]:
                    self.order[2] = True
                elif self.bbox[1] > self.bbox[2]:
                    self.order[3] = True
            
            if not self.order[4] and not self.order[5] and self.bbox[0] != 0 and self.bbox[2] != 0:
                if self.bbox[0] < self.bbox[2]:
                    self.order[4] = True
                elif self.bbox[0] > self.bbox[2]:
                    self.order[5] = True  

            self.bbox = [0, 0, 0]        

        if self.decided:
            print("모든 좌표 결정")
            
    def set_position(self):
        if not self.decided:
            targets = self.info_B[3:]
            s = []
            count = 0
            
            for x in targets:
                if x < 0.5:
                    count += 1


            # if count == 2:
            #     index = 0
            #     s = 0
            #     k = 5.0
            #     for i, x in enumerate(targets):
            #         if x > 0.5:
            #             index = i + 1
            #             s = x

            #     temp = self.order_B.index(index) # s가 정해진 표지판 순서 -> 0, 1, 2

            #     if temp == 0:
            #         self.info_B[4] = s + k
            #         self.info_B[5] = s + k * 2
            #     elif temp == 1:
            #         self.info_B[3] = s - k
            #         self.info_B[5] = s + k
            #     elif temp == 2:
            #         self.info_B[3] = s - k * 2
            #         self.info_B[4] = s - k

            #     self.decided = True
            

            if count == 1:
                for i, x in enumerate(targets):
                    if x < 0.5:
                        sign = i + 1 # b1, b2, b3
                    else:
                        s.append(x)

                if sign != self.sign_a:
                    self.decided = True
                else:
                    empty_index = self.order_B.index(sign) # 0, 1, 2번째

                    if sign == 1:
                        print("empty : b1")
                    elif sign == 2:
                        print("empty : b2")
                    elif sign == 3:
                        print("empty : b3")

                    if empty_index == 0:  # 첫번째 칸이 비었을 경우
                        gap = abs(s[0] - s[1])
                        self.info_B[sign + 2] = min(s) - gap
                    elif empty_index == 1:  # 두번째 칸이 비었을 경우
                        gap = abs(s[0] - s[1])
                        self.info_B[sign + 2] = min(s) + gap / 2
                    elif empty_index == 2:  # 세번째 칸이 비었을 경우
                        gap = abs(s[0] - s[1])
                        self.info_B[sign + 2] = max(s) + gap
                        
                    self.decided = True 
                

            if count == 0 and max(targets) - min(targets) > 15 and min(targets) > 0.5:
                for i, x in enumerate(targets):
                    if max(targets) - x > 20 and x > 0.5:
                        sign = i + 1
                    else:
                        s.append(x)
                        
                empty_index = self.order_B.index(sign)
                
                if empty_index == 0:  # 첫번째 칸이 비었을 경우
                    gap = abs(s[0] - s[1])
                    self.info_B[sign + 2] = min(s) - gap
                elif empty_index == 1:  # 두번째 칸이 비었을 경우
                    gap = abs(s[0] - s[1])
                    self.info_B[sign + 2] = min(s) + gap / 2
                elif empty_index == 2:  # 세번째 칸이 비었을 경우
                    gap = abs(s[0] - s[1])
                    self.info_B[sign + 2] = max(s) + gap
                    
                self.decided = True

            if count == 0:
                self.decided = True
                return                
               
                    
                    
        
    def find_B_loc(self, obs):
    
        self.sign_xy.run_b()
        sign_B1 = self.sign_xy.delivery_loc[0]
        sign_B2 = self.sign_xy.delivery_loc[1]
        sign_B3 = self.sign_xy.delivery_loc[2]
        s1, q = DP.xy2sl(sign_B1[0], sign_B1[1], mode = 1)
        s2, q = DP.xy2sl(sign_B2[0], sign_B2[1], mode = 1)
        s3, q = DP.xy2sl(sign_B3[0], sign_B3[1], mode = 1)

        print("s1 :", s1)
        print("s2 :", s2)
        print("s3 :", s3)
        
        if (sign_B1[0] != 0.0):
            self.loc_B[0] = s1
        if (sign_B2[0] != 0.0):
            self.loc_B[1] = s2
        if (sign_B3[0] != 0.0):
            self.loc_B[2] = s3

        if self.ordered and not self.decided: # 
            self.info_B[3] = self.loc_B[self.order_B.index(1)] # b1이 있는 곳의 순서
            self.info_B[4] = self.loc_B[self.order_B.index(2)] # b2이 있는 곳의 순서
            self.info_B[5] = self.loc_B[self.order_B.index(3)] # b3이 있는 곳의 순서
        #print("loc_B: ", self.loc_B)
   

################################################################################################

    def change_lane(self, pose, heading):
        steer = self.dv_path.gps_tracking(pose, heading, path_len = 3, ld = 6)
        speed = 60
        return speed, steer

    def run_b(self, pose, heading, traffic_sign, obs, current_s, del_a, bbox):

        # speed = 60
        # steer = 0
        speed, steer = self.change_lane(pose, heading)
        st_del = 0
        s,q = DP2.xy2sl(pose[0], pose[1])
        self.sign_xy.update(obs)
        self.sign_xy.pub_vis_sign()
        self.sign_a = del_a
        

        print("order :", self.order_B)
        print("info_b", self.info_B)

        # A 표지판 정지 완료했고 아직 B표지판 인식 못함
        if not self.decided: # (0.0 in self.info_B[3:]):
            print("searching_B....")
            # B 표지판 인식 할때까지 기존경로(배달경로) 따라 크루즈
            # st_del = 1 
            self.find_B_loc(obs)
            self.check_B_sign(traffic_sign, bbox)
            self.set_position()

        # B 표지판 인식 했고 아직 한바퀴 안돌았음
        else: # (not 0.0 in self.info_B[3:]):
            print("turning....")
            self.find_B_loc(obs)
            self.check_B_sign(traffic_sign, bbox)
            self.set_position()
            # 한바퀴 경로 넣고 따라서 크루즈
            self.speed_ok = True
            speed = 60 #200
            # st_del = 1
            # if (current_s - turn_end >= 0):
            # self.turn = True

        # 여기서 이게 들어갈 필요가 있나..?
        # elif (not self.speed_ok and not self.give):
        #     print("turnnig...break...")
        #     speed = 60
        #     st_del = 1
        #     if (self.time_rec == 0):
        #         self.pre_s = current_s
        #         self.time_rec = 1
        #     if (current_s - self.pre_s > 8):
        #         self.speed_ok = True
        #         self.time_rec = 0

        # B 표지판 인식했고 아직 정지안함
        if (self.speed_ok and not self.give):
 
            steer = self.dv_path.gps_tracking(pose, heading, [[0.0, 0.0]], 4, 8)
            self.info_A[0] = del_a
            goal = self.info_A[0] + 2
            print("s :", s)
            print("self.info_B[goal] :", self.info_B[goal])
            if(s - self.info_B[goal] > -2.1 and not self.stop): #-2.5
                self.time_rec = time.time()
                speed = -201
                print("stop !!!!")
                self.stop = True
            elif(time.time() - self.time_rec <= 8 and self.stop):
                speed = -201 #0
                print("delivery done !!!!")
            elif(time.time() - self.time_rec > 8 and self.stop):
                self.time_rec = 0
                self.give = True
            else:
                print("delivering....",self.info_A[0], self.info_B[goal])


        # B 표지판 정지 완료 했음 미션 끝
        elif (self.give):
            st_del = 1 
            print("배달 미션 끝!!!")
            st_del = 2
            # 다시 전역경로로 복귀

        return speed, steer, st_del