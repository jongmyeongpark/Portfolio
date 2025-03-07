#!/usr/bin/env python
#-*-coding:utf-8-*-

# 1. 라이다로 clean 된 pointcloud 받아오기(거리값만?)
# 2. 들어온 점들 중 우측 점들만 새로운 리스트에 추가
# 3. 기준 거리 p 선정해주기, 기준 갯수 q
# 4. 평행주차 구간이 시작되고 기준 k 값 이하 값이 들어오면 미션 알고리즘 시작
# 5. target position 에 대한 거리인 m 값 만큼의 값이 q 번 들어오면 target position 임을 체크하고 다음 step 으로 넘어감
# 6. 이러다 다시 기준거리 p 점들이 q 만큼 잡히면 여기가 정지해야 할 구간이구나 하고 speed = 0
# 7. 진입을 위해 후진 오른쪽으로 풀 스티어 3초
# 8. 수평을 위해 후진 왼쪽으로 풀 스티어 2초
# 9. 후진 스티어 0 로 1초
# 10. 정지 3초
# 11. 전진 스티어 0 로 1초
# 12. 전진 왼쪽으로 풀 스티어 2초
# 13. 전진 오른쪽으로 풀 스티어 3초
# 14. 끝, 원래 경로 복귀 후 달리기

from ctypes import pointer
from turtle import st
import rospy
import time
import math
import numpy as np

from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from macaron_6.msg import erp_write

P = 2.9 # 들어가면 안되는 곳 기준거리
Q = 10 # 넘어야 하는 기준 점 개수
K_N = 1.5 # 미션 시작 알리는 기준값(보이는 첫 라바콘 거리)
K_M = 2.9 #2.9
M_M = 5.8 # 들어가야 하는 공간 거리
M_N = 5.0
# PP = 10

class publish_erp():
    def __init__(self):
        self.erp_pub = rospy.Publisher("speed_planner", erp_write, queue_size=1)
        self.erp = erp_write()

    def pub_erp(self, speed, steer):
        self.erp.write_speed = speed
        self.erp.write_steer = steer
        self.erp_pub.publish(self.erp)
# pub = publish_erp()



class scan_lidar:
    def __init__(self):
        #구독자 선언
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size = 1)

        #flag 선언
        # self.lidar_flag = False

        #Sub 받은 데이터 저장 공간
        self.sub_scan = []
        for i in range(810):
            self.sub_scan.append(0.0)
        self.not_inf = []
        self.right_scan = []
        self.cleaned_scan = []
        self.step = 0
        self.current_time = 0
        self.check_time = 0
        self.first_count = 0
        self.stop_start = 0
        self.stop = False
        self.check_start = False
        self.check_complete = False
        self.tracking = False
        self.done = False
        self.a = 0
        self.b = 0
        self.c = 0

    ##########callback##########
    # 각 센서에서 데이터가 들어오면 객체 내부의 데이터 저장공간에 저장
    def scan_callback(self, scan):
        self.sub_scan = list(scan.ranges[120:400])
        self.right_scan = self.sub_scan
        # for i in self.sub_scan:            
        #     if i < 50: # -90~90
        #         self.not_inf.append(i)
                
            
        # self.lidar_flag = True

    # gps, imu, lidar 가 모두 작동해서 데이터가 들어왔는지 확인
    # def senser_check(self):
    #     return self.lidar_flag

    ############################
    # def clean_scan_update(self, scan_list):
    #     for i in scan_list:
    #         if i < PP:
    #             self.not_inf.append(i)

    #         else:
    #             pass

    # def clus_right_update(self, scan_list2):
    #     for i in range(len(scan_list2)):
    #         if i < int((len(scan_list2))/8):
    #             self.right_scan.append(i)

    #         else:
    #             pass


    def mission_start(self):
        a = 0
        # count = 0
        if not self.tracking:
            for i in self.right_scan:
                if i > K_N and i < K_M:
                    a += 1
                    
                print(a)
                # if a > 9:
                    
                if a > 5:
                    self.current_time = time.time()
                    self.tracking = True
                    # self.step = 1
                    print("미션 시작 첫 라바콘 감지")
            
        elif time.time() - self.current_time > 0.5:
            self.step = 1



    def check_target(self):
        b = 0
        if not self.check_start:
            self.check_time = time.time()
            self.check_start = True
        elif time.time() - self.check_time < 1:
            for i in self.right_scan:
                if K_N < i < K_M:
                    self.first_count += 1
        elif time.time() - self.check_time > 1:
            if self.first_count < 10:
                self.step = 2
                return


            for i in self.right_scan:
                if K_N < i < K_M:
                    return
                    
            for i in self.right_scan:
                if i > M_N and i < M_M:
                    b += 1
                
                if b > 4:
                    self.step = 2
                    print("주차해야 할 위치 확인")

    # def distance(self):
    #     count = 0
    #     for i in self.right_scan:
    #         if i < 3:
    #             count += 1

    #     if count > 10:
    #         return True
    #     else:
    #         return False

    def ready_to_stop(self):
        c = 0
        points = []
        # for i in self.right_scan:
        #     if K_N < i < K_M:
        #         return
        
        for i in self.right_scan:
            if 1 < i < P:
                c += 1
                points.append(i)
            if c > 0:
                avg = sum(points) / c
                print("c :", c)
                print("avg :", avg)
                if c > 5 and avg < 2.8:
                    self.step = 3
                    print("이제 멈춰야됨")

    def finish(self):
        self.done = True



    def run(self):
        pub = publish_erp()
        # self.clean_scan_update(self.not_inf)
        # self.clus_right_update(self.not_inf)
        if self.step == 0:
            # self.clean_scan_update(self.not_inf)
            # self.clus_right_update(self.not_inf)
            self.mission_start()
            print("now step 0")
            # print(self.not_inf)
        
        elif self.step == 1:
            print("step1 start")
            # self.clean_scan_update(self.not_inf)
            # self.clus_right_update(self.not_inf)
            self.check_target()
            print("now step 1")

        elif self.step == 2:
            # self.clean_scan_update(self.not_inf)
            # self.clus_right_update(self.not_inf)
            self.ready_to_stop()
            print("now step 2")

        elif self.step == 3:
            print("now step 3")
            time.sleep(2.5)
            pub.pub_erp(-60,2000)
            time.sleep(3.2)
            pub.pub_erp(-201,0)
            time.sleep(1)
            pub.pub_erp(0,-2000)
            time.sleep(1)
            pub.pub_erp(-60,-2000)
            time.sleep(2.5)
            pub.pub_erp(-30,0)
            # time.sleep(0.5)
            pub.pub_erp(-201,0)
            time.sleep(11)
            self.step = 4
            # if not self.stop:
            #     self.stop_start = time.time()
            #     self.stop = True
            #     pub.pub_erp(0,0)
            # elif time.time() - self.stop_start > 11:
            #     self.step = 4

        elif self.step == 4:
            print("now step 4")
            # pub.pub_erp(30,0)
            # time.sleep(1.5)
            pub.pub_erp(0,-2000)
            time.sleep(1)
            pub.pub_erp(80,-2000)
            time.sleep(3)
            pub.pub_erp(30,2000)
            time.sleep(2)
            self.step = 5

        elif self.step == 5:
            print("now step 5 finished")
            self.finish()
