#!/usr/bin/env python
# -*-coding:utf-8-*-

# Python Package

import rospy
import sys, os
import time
import numpy as np

from macaron_6.msg import Traffic, obj_info

'''
<주행 전 무조건 바꿔야하는 변수>

1. line 25 (직진:0 , 좌회전:1)
2. line 51 (직진 정지선s값)
3. line 52 (좌회전 정지선 s값)
4. line 57 (직진 or 좌회전 정지선 s값) 
5. line 104 (4와 동일)

'''

# Parameter
MODE = 0  # 미션에 따라 모드 설정해주기(0:직진 / 1:좌회전)
BEFORE_STOP_LINE_SPEED = 70
AFTER_STOP_LINE_SPEED = 110 - 15

between_erp_stop_line = 15.0  # 정지선~erp 사이 정                                              
traffic_list = ['red_3',
                "orange_3",
                "green_3", 
                'red_4', 
                "orange_4",
                "left_green_4", 
                "all_green_4",
                "straight_green_4"]

# traffic_list = ['green', 
#                 'yellow', 
#                 'red', 
#                 'all_green', 
#                 'left']


# 인지 판단 구간 계산(속도에 따라 다른 값 , 그냥 고정값 써도 무관)
def stop_distance(erp_speed):
    erp_speed = np.round((erp_speed - 1) / 10) * 10

    stop_dist = (0.4 / 10) * (erp_speed - 50) + 1.5

    return 2.0  # 고정으로 쓰고 싶다면
    # return stop_line + stop_dist


class mission_traffic_straight:
    def __init__(self, WHERE=1):
        # 정지선 좌표, 사실 WHERE 값 없애고 초기화 해줘도 상관없을 듯
        if WHERE == 1:
            self.STRAIGHT_STOP_LINE = [260]  # 직진 신호등 정지선 s좌표
            self.LEFT_STOP_LINE = [999999]  # 좌회전 신호등 정지선 s좌표
        if WHERE == 2 or WHERE == 3:  # k-city
            # 직진 신호등 정지선 s좌표
            self.STRAIGHT_STOP_LINE = [280 - 2, 416, 899 - 2, 947 - 2]
            self.LEFT_STOP_LINE = [229.5 - 2, 522, 603- 2]  # 좌회전 신호등 정지선 s좌표

        self.sub_sign = rospy.Subscriber('/traffic_obj', Traffic, self.obj_callback, queue_size=1)
        self.traffic_flag = 0
        self.time_rec = 0
        self.traffic_s = 30.3
        self.done = False
        self.stop = True
        self.start_count = 0
        self.green_count = 0
        self.red_count = 0
        self.left_green_count = 0
        self.traffic = [0, 0, 0, 0, 0]
        self.mode = 0
        self.end_time = None
        self.interval_time = 120
        self.interval_time_left = 120
        
        # self.start_red_time = 99999999999999999999

    def traffic_update(self, ns):

        if ns == "red_3" or ns == "red_4":
            self.traffic[0] += 1
            self.traffic[1] = 0
            self.traffic[2] = 0
            self.traffic[3] = 0
            self.traffic[4] = 0
        elif ns == "orange_3" or ns == "orange_4":
            self.traffic[0] = 0
            self.traffic[1] += 1
            self.traffic[2] = 0
            self.traffic[3] = 0
            self.traffic[4] = 0
        elif ns == "left_green_4":
            self.traffic[0] = 0
            self.traffic[1] = 0
            self.traffic[2] += 1
            self.traffic[3] = 0
            self.traffic[4] = 0
        elif ns == "straight_green_4" or ns == "green_3":
            self.traffic[0] = 0
            self.traffic[1] = 0
            self.traffic[2] = 0
            self.traffic[3] += 1
            self.traffic[4] = 0
        elif ns == "all_green_4":
            self.traffic[0] = 0
            self.traffic[1] = 0
            self.traffic[2] = 0
            self.traffic[3] = 0
            self.traffic[4] += 1
        else:
            pass
        print("self.traffic : ",self.traffic)

        # if ns == "red":
        #     self.traffic[0] += 1
        #     self.traffic[1] = 0
        #     self.traffic[2] = 0
        #     self.traffic[3] = 0
        #     self.traffic[4] = 0
        # elif ns == "orange":
        #     self.traffic[0] = 0
        #     self.traffic[1] += 1
        #     self.traffic[2] = 0
        #     self.traffic[3] = 0
        #     self.traffic[4] = 0
        # elif ns == "left":
        #     self.traffic[0] = 0
        #     self.traffic[1] = 0
        #     self.traffic[2] += 1
        #     self.traffic[3] = 0
        #     self.traffic[4] = 0
        # elif ns == "green":
        #     self.traffic[0] = 0
        #     self.traffic[1] = 0
        #     self.traffic[2] = 0
        #     self.traffic[3] += 1
        #     self.traffic[4] = 0
        # elif ns == "all_green":
        #     self.traffic[0] = 0
        #     self.traffic[1] = 0
        #     self.traffic[2] = 0
        #     self.traffic[3] = 0
        #     self.traffic[4] += 1
        #
        # else:
        #     pass

    def obj_callback(self, data):

        boxes_3_phase = list()
        boxes_4_phase = list()

        for sign in data.obj:

            if sign.ns in traffic_list:  # 표지판인 경우만 detect

                if sign.ns == "red_3" or sign.ns == "green_3" or sign.ns == "orange_3":
                    # 넓이와 class 이름 tuple 형태로 추가
                    boxes_3_phase.append([sign.xmin, sign.ymin, sign.xmax, sign.ymax, sign.ns])
                
                else:
                    # 넓이와 class 이름 tuple 형태로 추가
                    boxes_4_phase.append([sign.xmin, sign.ymin, sign.xmax, sign.ymax, sign.ns])
        
        # 3,4구 동시검출
        if len(boxes_3_phase) >= 1 and len(boxes_4_phase) >= 1:

            boxes_3_phase.sort(key=lambda x: x[3]) 
            boxes_4_phase.sort(key=lambda x: x[3]) 

            # 4구 우선으로 두고, 그다음 3구 append
            for i in boxes_3_phase:
                boxes_4_phase.append(i)

            if len(boxes_4_phase) >= 4:
                self.traffic_update(boxes_4_phase[0][4])
                self.traffic_update(boxes_4_phase[1][4])
            else:
                self.traffic_update(boxes_4_phase[0][4])
        
        # 4구만 검출
        elif len(boxes_4_phase) >= 1 and len(boxes_3_phase) == 0:

            boxes_4_phase.sort(key=lambda x: x[3]) 

            if len(boxes_4_phase) >= 4:
                self.traffic_update(boxes_4_phase[0][4])
                self.traffic_update(boxes_4_phase[1][4])
            else:
                self.traffic_update(boxes_4_phase[0][4])

        # 3구만 검출
        elif len(boxes_3_phase) >= 1 and len(boxes_4_phase) == 0:

            boxes_3_phase.sort(key=lambda x: x[3]) 

            if len(boxes_3_phase) >= 4:
                self.traffic_update(boxes_3_phase[0][4])
                self.traffic_update(boxes_3_phase[1][4])
            else:
                self.traffic_update(boxes_3_phase[0][4])

    # 3,4구 구분 없는 pt
    def obj_callback2(self, data):

        box_l = list()
        rest_boxes = list()

        for sign in data.obj:

            if sign.ns in traffic_list:  # 표지판인 경우만 detect

                if sign.ns == "left":
                    # 넓이와 class 이름 tuple 형태로 추가
                    box_l.append([sign.xmin, sign.ymin, sign.xmax, sign.ymax, sign.ns])
                
                else:
                    # 넓이와 class 이름 tuple 형태로 추가
                    rest_boxes.append([sign.xmin, sign.ymin, sign.xmax, sign.ymax, sign.ns])
        
        # 좌회전검출시 좌회전만
        if len(box_l) >= 1:

            box_l.sort(key=lambda x: x[3]) 

            if len(box_l) >= 4:
                self.traffic_update(box_l[0][4])
                self.traffic_update(box_l[1][4])
            else:
                self.traffic_update(box_l[0][4])
        
        # 검출 안되면 나머지 박스들끼리만
        else:

            rest_boxes.sort(key=lambda x: x[3]) 

            if len(rest_boxes) >= 4:
                self.traffic_update(rest_boxes[0][4])
                self.traffic_update(rest_boxes[1][4])
            else:
                self.traffic_update(rest_boxes[0][4])
    

    # def obj_callback(self, data):

    #     boxes = list()

    #     for sign in data.obj:

    #         if sign.ns in traffic_list:  # 표지판인 경우만 detect
    #             # 넓이와 class 이름 tuple 형태로 추가
    #             boxes.append([sign.xmin, sign.ymin, sign.xmax, sign.ymax, sign.ns])
        
    #     if len(boxes) >= 1:
    #         boxes.sort(key=lambda x: x[3]) # x[1]은 ymin, x[3]은 ymax

    #         if len(boxes) >= 4:
    #             self.traffic_update(boxes[0][4])
    #             self.traffic_update(boxes[1][4])
    #         else:
    #             self.traffic_update(boxes[0][4])


    def reset(self):
        self.traffic_flag = 0
        self.done = False
        self.stop = True
        self.time_rec = 0
        self.start_count = 0
        self.green_count = 0
        self.red_count = 0
        self.left_green_count = 0
        # self.start_red_time = 99999999999999999999
        self.traffic_s = 30.3  # 정지선 s값
        self.end_time = None
        self.interval_time = 120
        self.interval_time_left = 120

    def run(self, s, erp_speed, mode):
        self.mode = mode
        
        speed = 0.0

        # erp와 정지선 사이의 거리를 토대로 인지판단 구간 갱신
        if self.mode == 0:
            for i in self.STRAIGHT_STOP_LINE:
                if i - 16 <= s <= i + 10:
                    self.traffic_s = i
            # speed = 0

        if self.mode == 1:
            for i in self.LEFT_STOP_LINE:
                if i - 16 <= s <= i + 10:
                    self.traffic_s = i

            # speed = 0

        # 신호등 인식 시작 구간
        if self.traffic_flag == 0:
            speed = 90

            # 인지판단 구간 진입
            # 만약 정지선(traffic_s)을 지났을때도 멈추도록 and 값
            if (self.traffic_s - s <= between_erp_stop_line) and (self.traffic_s - 3 > s):
                print('정지에 대비하기 위해 감속 합니다!')
                self.traffic_flag = 1

        elif self.traffic_flag == 1:
            speed = BEFORE_STOP_LINE_SPEED

            # 직진인 경우
            if self.mode == 0:

                # 초록불, 직좌신호
                if self.traffic[3] > 10 or self.traffic[4] > 10:
                    self.green_count += 1
                    self.left_green_count = 0
                    self.red_count = 0

                # 빨간불 or 주황불 or 좌회전이면
                elif self.traffic[0] > 10 or self.traffic[1] > 10 or self.traffic[2] > 10:
                    self.green_count = 0
                    self.left_green_count = 0
                    self.red_count += 1

                if self.green_count >= 15:
                    self.stop = False
                # elif self.red_count >= 15:
                #     self.stop = True
                # elif self.left_green_count >= 15:
                #     self.stop = True

                # 비율로 stop 판단하기
                # if self.green_count / self.red_count > 0:
                #     self.stop = False
                #
                # elif self.green_count / self.red_count < 0:
                #     self.stop = True
                #
                # else:
                #     print("detect nothing....")

            # 좌회전 미션인 경우
            elif self.mode == 1:

                # 빨좌, 직좌 라면
                if self.traffic[2] > 10 or self.traffic[4] > 10:
                    self.green_count = 0
                    self.left_green_count += 1
                    self.red_count = 0

                # 빨간불, 주황불,  초록불이면
                elif self.traffic[0] > 10 or self.traffic[1] > 10 or self.traffic[3] > 10:
                    self.green_count = 0
                    self.left_green_count = 0
                    self.red_count += 1

                if self.left_green_count >= 15:
                    self.stop = False
                # elif self.red_count >= 15:
                #     self.stop = True
                # elif self.green_count >= 2:
                #     self.stop = True

                # 비율로 stop 판단하기
                # if self.left_green_count / self.red_count > 0:
                #     self.stop = False
                #
                # elif self.left_green_count / self.red_count < 0:
                #     self.stop = True
                #
                # else:
                #     print("detect nothing....")

            over_stop_line = stop_distance(erp_speed)

            if self.traffic_s - s <= over_stop_line:  # 인지판단 구간을 넘을 시

                self.traffic_flag = 2

                if self.stop is True:
                    print("빨간불 인식 정지")

                else:
                    print("파란불 인식 중")

        elif self.traffic_flag == 2:
            if self.end_time is None:
                self.end_time = time.time()

            # if self.stop is True and time.time() < self.end_time + self.interval_time :
            #     speed = 0

            if self.mode == 0:
                if self.stop is True and time.time() < self.end_time + self.interval_time :
                    speed = 0
                
                    if self.traffic[3] > 1 or self.traffic[4] > 1:
                        self.start_count += 1
                    else:
                        self.start_count = 0

                    if self.start_count >= 3:
                        self.traffic_flag = 3
                else:
                    speed = AFTER_STOP_LINE_SPEED
                    self.traffic_flag = 3                
                    
            elif self.mode == 1:
                if self.stop is True and time.time() < self.end_time + self.interval_time_left :
                    speed = 0
                    if self.traffic[4] > 1 or self.traffic[2] > 1:
                        self.start_count += 1
                    else:
                        self.start_count = 0

                    if self.start_count >= 3:
                        self.traffic_flag = 3
                                        
                else:
                    speed = 90
                    self.traffic_flag = 3

            self.time_rec = time.time()

        # 초록불 판단 후
        elif self.traffic_flag == 3:
            speed = 90
            t = time.time()
            while(time.time()-t < 0.3):
                print("done!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            self.done = True
            self.reset()
            # if s - self.traffic_s > 2:
            #     self.done = True
            #     self.reset()
            #     for i in range(1, 10, 1):
            print('신호등 미션 끝!\n')
        return speed
