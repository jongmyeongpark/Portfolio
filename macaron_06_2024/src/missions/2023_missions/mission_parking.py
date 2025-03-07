#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy
import sys, os
import math
import numpy as np
import time

from sensor_msgs.msg import PointCloud

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/path_planning")

from path_planning_tracking import Path_Tracking
from trajectory_planner import TrajectoryPlanner
from global_path import GlobalPath
from macaron_5.msg import erp_write
from sub_erp_state import sub_erp_state

# 파라미터
parking_width = 2.0/2
# parking_i = [[13, 18, 21.5],[16, 19, 23],[20, 24, 27],[17, 27, 30]] # [[여기부터경로탐색, 경로정하고(주차시작s), 종료 자리(주차완료s)]]
parking_i = [[2, 5, 10.5],[2, 5, 10.5],[2, 5, 10.5],[2, 5, 10.5],[2, 5, 10.5],[2, 5, 10.5]]

# parking_i2 = [[16, 18, 25.5]]
# parking_i3 = [[20, 22, 28.5]]
# parking_i4 = [[17, 24, 30]]
#parking_i[0] 까지 경로를 정하라는 뜻임.
parking_speed = [40, -60] # 주차 진입, 후진 속도
stay_time = 10


# 서울대에서 테스트할때는 state 에서 기본경로 바꾼다음에
# s 좌표들 측정해서 state 랑 여기 parking_i 바꿔주고
# 아래 경로들 "parking_KC%d.npy" 들을 모두 "parking_SNU%d.npy" 로 바꿔주면 됨.
# 지금은 snu_add_parking1.npy 임.
# 원래는 parking_KC%d.npy
# KC 경로는 6개니까 k-city 가서 바꿔주기만 하면 됨.

class mission_parking():
    def __init__(self, Where):
        global parking_i
        self.erp = sub_erp_state()
        self.WHERE = Where
        self.parking_file_ind = 1 ##### 2
        self.PT_parking = Path_Tracking("parking_KC%d.npy"%(self.parking_file_ind))
        ### self.PT_parking = Path_Tracking("snu_add_parking%d.npy"%(self.parking_file_ind))
        self.cnt = 0 ##### 1
        self.current_time = 0

        ####
        # self.GB = GlobalPath(filename = "/home/macaron/catkin_ws/src/macaron_4_advanced/path/npy_file/path/snu_add_parking%d.npy"%(self.parking_file_ind))
        # self.s, self.q = self.GB.xy2sl(self.erp.pose[0], self.erp.pose[1])
        PATH_ROOT = os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))+"/path/npy_file/path/"

        self.gv_name1 = PATH_ROOT+"/parking_KC1.npy"
        self.gv_name2 = PATH_ROOT+"/parking_KC2.npy"
        self.gv_name3 = PATH_ROOT+"/parking_KC3.npy"
        self.gv_name4 = PATH_ROOT+"/parking_KC4.npy"
        self.gv_name5 = PATH_ROOT+"/parking_KC5.npy"
        self.gv_name6 = PATH_ROOT+"/parking_KC6.npy"
        ### 서울대 리허설용 경로
        # self.gv_name1 = "/home/macaron/catkin_ws/src/macaron_4_advanced/path/npy_file/path/snu_add_parking1.npy"
        # self.gv_name2 = "/home/macaron/catkin_ws/src/macaron_4_advanced/path/npy_file/path/snu_add_parking2.npy"
        # self.gv_name3 = "/home/macaron/catkin_ws/src/macaron_4_advanced/path/npy_file/path/snu_add_parking3.npy"
        # self.gv_name4 = "/home/macaron/catkin_ws/src/macaron_4_advanced/path/npy_file/path/snu_add_parking4.npy"

        self.gv_name = [self.gv_name1, self.gv_name2, self.gv_name3, self.gv_name4, self.gv_name5, self.gv_name6]

        self.DP = GlobalPath(self.gv_name[self.cnt])
        
        ####
        #npy 배열 불러오기 위에서는 따로 하고 배열에 넣어서 인덱스로 꺼내쓰고 여기서는 %d 로 1씩 올려주는거
        self.parking_np = np.load(file = (os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/parking_KC%d.npy"%(self.parking_file_ind))
        self.parking_index = parking_i[self.cnt]
        #TrajectoryPlanner 로 경로 확인
        self.path_planner_parking = TrajectoryPlanner(GlobalPath(self.gv_name[self.cnt]))

        self.stop = False
        self.current = 0
        self.enc_target = 0 #encoder 초기값
        self.rec_st = {} #steer record 해서 저장
        self.step = 0 # step 초기값
        self.ENC_target_state = False #encoder target 상태
        self.done = False #미션 완료됐는지 아닌지 확인
    
    #loop 를 위한 update 함수
    def update_DP(self):
        self.parking_index = parking_i[self.cnt]
        self.DP = GlobalPath(self.gv_name[self.cnt])
        self.path_planner_parking = TrajectoryPlanner(GlobalPath(self.gv_name[self.cnt]))

    def check_location(self, pose, heading, obs, s): #들어갈 경로 정하기
        global parking_speed

        # try:
        print(self.parking_file_ind)
        
        if s > self.parking_index[0]:
            _, collision_count = self.path_planner_parking.optimal_trajectory_parking(pose[0], pose[1], heading, obs)
            print("collision : ", collision_count)
            if collision_count == False:
            #if 
                self.PT_parking = Path_Tracking("parking_KC%d.npy"%(self.parking_file_ind))
                ### self.PT_parking = Path_Tracking("snu_add_parking%d.npy"%(self.parking_file_ind))
                self.parking_np = np.load(file = (os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/parking_KC%d.npy"%(self.parking_file_ind))
                # print("주차 자리 정함")
                self.ENC_target_state = False
                # s, q = self.DP.xy2sl(pose[0], pose[1])

            else:
                if self.parking_file_ind < 6 and time.time() - self.current_time > 0.5:
                    self.current_time = time.time()
                    self.parking_file_ind += 1
                    self.cnt += 1
                    self.update_DP()
                    self.PT_parking = Path_Tracking("parking_KC%d.npy"%(self.parking_file_ind))
                    ### self.PT_parking = Path_Tracking("snu_add_parking%d.npy"%(self.parking_file_ind))
                    self.parking_np = np.load(file = (os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/parking_KC%d.npy"%(self.parking_file_ind))
                    self.ENC_target_state = False


        # except:
        #     # print("주차 못했음")
        #     # self.parking_file_ind -= 1
        #     # self.done = True
            # print("aaaa")

        steer = self.PT_parking.gps_tracking_parking(pose, heading, path_len = 3, ld = 6)
        speed = parking_speed[0]
        return speed, steer

    def parking(self, pose, heading, lane): #주차하기
        global parking_speed
        steer = self.PT_parking.gps_tracking_parking(pose, heading, path_len = 3, ld = 6)
        speed = parking_speed[0]
        return speed, steer
        
    def back(self, ENC, s): #후진
        global parking_speed
        a = 0
        if ENC > self.enc_target and s > self.parking_index[1] + 2:
            while ((ENC - a) in self.rec_st) is False:
                a += 1
            steer = self.rec_st[ENC - a]
            speed = parking_speed[1]
            print(self.done)
            return speed, steer
        else:
            self.done = True
            return 0, 0

    def rec_steer(self, steer, ENC): #steer record
        if ENC in self.rec_st:
            pass
        else:
            print("rec..."),
            self.rec_st[ENC] = steer

    def reset(self): #리셋
        self.parking_file_ind = 1
        if self.WHERE == 1:
            self.PT_parking = Path_Tracking("parking_KC%d.npy"%(self.parking_file_ind))
            ### self.PT_parking = Path_Tracking("snu_add_parking%d.npy"%(self.parking_file_ind))
            self.parking_np = np.load(file = (os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/parking_KC%d.npy"%(self.parking_file_ind))
        else:
            self.PT_parking = Path_Tracking("parking_KC%d.npy"%(self.parking_file_ind))
            ### self.PT_parking = Path_Tracking("snu_add_parking%d.npy"%(self.parking_file_ind))
            self.parking_np = np.load(file = (os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/parking_KC%d.npy"%(self.parking_file_ind))
        self.enc_target = 0
        self.rec_st = {}
        self.step = 0
        self.ENC_target_state = False
        self.done = False

    def run(self, pose, heading, obs, lane, erp_ENC, erp_steer): #여기가 메인 함수
        global stay_time
        s, q = self.DP.xy2sl(pose[0], pose[1])
        self.path_planner_parking.optimal_trajectory_parking(pose[0], pose[1], heading, obs)
        ### 미션 인덱스 값 ###
        print("sss : " ,s)
        if s > self.parking_index[0] and self.ENC_target_state == False:
            self.enc_target = erp_ENC
            self.ENC_target_state = True
        if s > self.parking_index[1] and self.step == 0:
            self.step = 1
            print("주차 자리 확정")
        if s > self.parking_index[2] and self.step == 1:
            self.step = 2
            print("주차 완료")
            return -201, 0

        ##################################################

        if self.step == 0:
            speed = 60
            # self.rec_steer(erp_steer, erp_ENC)
            speed, steer =  self.check_location(pose, heading, obs, s)
        if self.step == 1:
            self.rec_steer(erp_steer, erp_ENC)
            speed, steer = self.parking(pose, heading, lane)
        if self.step == 2:
            rospy.sleep(stay_time)
            self.step = 3
            print("후진 시작")
        if self.step == 3:
            speed, steer = self.back(erp_ENC,s)

        return speed, steer