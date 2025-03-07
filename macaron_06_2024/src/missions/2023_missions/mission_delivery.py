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
from delivery_sign_loc_old import delivery_sign_loc as del_sign

turn_end = 150 # 돌아서 나오는거 끝나는 s 좌표 701.8
B_plus_threshold = 13 # B_plus 이상 인지 누적 이후 급감 구역 탐색
B_minus_threshold = 5 # B_minus 이상 급감시 해당 s좌표 저장

# DV_PATH_NAME = "mission_delivery_path.npy"
DV_PATH_NAME = "snu_del_path1.npy"

PATH_ROOT=(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/"
gv_name = PATH_ROOT + DV_PATH_NAME
DP = GlobalPath(gv_name) # find 와 check 에 사용되는 경로
DP2 = GlobalPath(gv_name) # run 에 사용되는 경로


# 1. 경로를 따라 주행하며 A 표지판 인지 -> 카메라
# 2. 인식한 A표지판의 숫자(A1, A2, A3)를 저장 -> 저장 및 이런걸 경로 DP 사용
# 3. 표지판 위치에 가서 5초 정지
# 4. B 표지판 무시하고 해당 블록 주행(경로 따로 만들어서 넣어야 함)
# 5. A 표지판 무시하고 B표지판 중 맞는 위치에 가서 5초 정지
# 6. 이후 원래 경로로 복귀

class mission_delivery:

    def __init__(self):
        self.recive = False
        self.stop = False
        self.turn = False
        self.give = False
        self.speed_ok = False

        #[A표지판, A의 S좌표]
        self.info_A = [0, 0.0]
        self.info_B = [0, 0, 0, 0.0, 0.0, 0.0]

        #[B위치]
        self.loc_B = [0.0, 0.0, 0.0]
        self.order_B = [0, 0, 0]
        self.count_A = [0, 0, 0]
        self.count_B = [0, 0, 0]
        self.pre_s = 0
        self.time_rec = 0
        self.dv_path = Path_Tracking(DV_PATH_NAME)
        self.sign_xy = del_sign(2) ##
        
        

    def find_A(self, traffic_sign, current_s):

        self.sign_xy.run()
        sign_A = self.sign_xy.pickup_loc[0]
        s, q = DP.xy2sl(sign_A[0], sign_A[1], mode = 1)

        if (1 in traffic_sign[0:3]):
            if (traffic_sign[0] == 1):
                self.count_A[0] += 1
            if(traffic_sign[1] == 1):
                self.count_A[1] += 1
            if(traffic_sign[2] == 1):
                self.count_A[2] += 1

        if (s - self.info_A[1] >= -1.5):
            self.info_A[0] = int(self.count_A.index(max(self.count_A))) + 1            
        else:
            self.info_A[0] = 0
        #print("count_A: ", self.count_A)

        self.info_A[1] = s
        
        print("info_A: ",self.info_A)

    def check_B_sign(self, traffic_sign):
        
        #if(1 in traffic_sign[3:6]):
            
        if(traffic_sign[3] == 1):
            self.info_B[0] += 1
            self.count_B[0] = 0
        elif(traffic_sign[4] == 1):
            self.info_B[1] += 1
            self.count_B[1] = 0
        elif(traffic_sign[5] == 1):
            self.info_B[2] += 1
            self.count_B[2] = 0

        if(self.info_B[0] > B_plus_threshold ):
            if (traffic_sign[3] != 1):
                self.count_B[0] += 1
        if(self.info_B[1] > B_plus_threshold ):
            if (traffic_sign[4] != 1):
                self.count_B[1] += 1
        if(self.info_B[2] > B_plus_threshold ):
            if (traffic_sign[5] != 1):
                self.count_B[2] += 1
            
        if(self.count_B[0] > B_minus_threshold):
            if (self.order_B[1] == 0 and self.order_B[2] == 0 ):
                self.order_B[0] = 1
            elif (self.order_B[1] != 0 and self.order_B[2] != 0 ):
                self.order_B[0] = 3
            else:
                self.order_B[0] = 2
                self.order_B[self.order_B.index(0)] = 3
            self.info_B[0] = -999
            self.count_B[0] = 0
        if(self.count_B[1] > B_minus_threshold):
            if (self.order_B[0] == 0 and self.order_B[2] == 0 ):
                self.order_B[1] = 1
            elif (self.order_B[0] != 0 and self.order_B[2] != 0 ):
                self.order_B[1] = 3
            else:
                self.order_B[1] = 2
                self.order_B[self.order_B.index(0)] = 3
            self.info_B[1] = -999
            self.count_B[1] = 0
        if(self.count_B[2] > B_minus_threshold):
            if (self.order_B[0] == 0 and self.order_B[1] == 0 ):
                self.order_B[2] = 1
            elif (self.order_B[0] != 0 and self.order_B[1] != 0 ):
                self.order_B[2] = 3
            else:
                self.order_B[2] = 2
                self.order_B[self.order_B.index(0)] = 3
            self.info_B[2] = -999
            self.count_B[2] = 0

        if (not 0 in self.order_B ):
            self.info_B[3] = self.loc_B[self.order_B[0]-1]
            self.info_B[4] = self.loc_B[self.order_B[1]-1]
            self.info_B[5] = self.loc_B[self.order_B[2]-1]

        print("info_B : ", self.info_B)            

            
    def find_B_loc(self, obs):
    
        self.sign_xy.run()
        sign_B1 = self.sign_xy.delivery_loc[0]
        sign_B2 = self.sign_xy.delivery_loc[1]
        sign_B3 = self.sign_xy.delivery_loc[2]
        s1, q = DP.xy2sl(sign_B1[0], sign_B1[1], mode = 1)
        s2, q = DP.xy2sl(sign_B2[0], sign_B2[1], mode = 1)
        s3, q = DP.xy2sl(sign_B3[0], sign_B3[1], mode = 1)

        if (sign_B1[0] != 0.0):
            self.loc_B[0] = s1
        if (sign_B2[0] != 0.0):
            self.loc_B[1] = s2
        if (sign_B3[0] != 0.0):
            self.loc_B[2] = s3
        #print("loc_B: ", self.loc_B)
   

################################################################################################

    def run(self, pose, heading, traffic_sign, obs, current_s):
                          
        speed = 60
        steer = 0
        st_del = 0
        s,q = DP2.xy2sl(pose[0], pose[1])
        self.sign_xy.update(obs)
        self.sign_xy.pub_vis_sign()
        
        # 아직 A표지판 인식 못함
        if (not self.speed_ok and not self.recive):
            print("break....")
            st_del = 1
            if (self.time_rec == 0):
                self.pre_s = current_s
                self.time_rec = 1
            if (current_s - self.pre_s > 10):
                self.speed_ok = True
                self.time_rec = 0
        
        # A 표지판 인식했고 아직 정지안함
        elif (self.speed_ok and not self.recive):

            #배달경로로 갈아타기        
            steer = self.dv_path.gps_tracking(pose, heading, [[0.0, 0.0]], 4, 8)
            self.find_A(traffic_sign, s)
            #self.find_A(['a3'], obs)

            if(s - self.info_A[1] >= -2.2 and not self.stop):
                self.time_rec = time.time()
                print("recive!!!!")
                self.stop = True
            elif(time.time() - self.time_rec <= 8 and self.stop):
                speed = 0 
                print("stop !!!!")
            elif(time.time() - self.time_rec > 8 and self.stop):
                self.time_rec = 0
                self.stop = False
                self.recive = True
                self.speed_ok = False
            else:
                print("reciving....")

        # A 표지판 정지 완료했고 아직 B표지판 인식 못함
        elif (self.recive and 0.0 in self.info_B[3:]):
            print("searching_B....")
            # B 표지판 인식 할때까지 기존경로(배달경로) 따라 크루즈
            st_del = 1 
            self.find_B_loc(obs)
            self.check_B_sign(traffic_sign)

        # B 표지판 인식 했고 아직 한바퀴 안돌았음
        elif (not self.turn and not 0.0 in self.info_B[3:]):
            print("turning....")
            self.find_B_loc(obs)
            self.check_B_sign(traffic_sign)
            # 한바퀴 경로 넣고 따라서 크루즈
            speed = 60 #200
            st_del = 1
            if (current_s - turn_end >= 0):
                self.turn = True

        elif (self.turn and not self.speed_ok and not self.give):
            print("turnnig...break...")
            speed = 60
            st_del = 1
            if (self.time_rec == 0):
                self.pre_s = current_s
                self.time_rec = 1
            if (current_s - self.pre_s > 10):
                self.speed_ok = True
                self.time_rec = 0
        

        # B 표지판 인식했고 아직 정지안함
        elif (self.turn and self.speed_ok and not self.give):
 
            steer = self.dv_path.gps_tracking(pose, heading, [[0.0, 0.0]], 4, 8)
            goal = self.info_A[0] + 2
            if(s - self.info_B[goal] >= -2.1 and not self.stop):
                self.time_rec = time.time()
                print("stop !!!!")
                self.stop = True
            elif(time.time() - self.time_rec <= 8 and self.stop):
                speed = 0 
                print("delivery done !!!!")
            elif(time.time() - self.time_rec > 8 and self.stop):
                self.time_rec = 0
                self.give = True
            else:
                print("delivering....",self.info_A[0], self.info_B[goal])


        # B 표지판 정지 완료 했음 미션 끝
        elif (self.give):
            print("배달 미션 끝!!!")
            st_del = 2
            # 다시 전역경로로 복귀

        return speed, steer, st_del