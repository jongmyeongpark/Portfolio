#!/usr/bin/env python
# -*-coding:utf-8-*-

# 1. object_detection_jjj 에서 쏴준 정보를 ros 로 받음
# 2. 넓이를 기준 으로 처리 해줌 -> 원래는 xmin 을 고려 하려 했으나 양쪽에서 잘 동작할 수 있게끔 넓이로 수정
# 3. 가중치를 두어서 정확하게 처리해 줌
# 4. 멈추는 구간을 return 함 -> state_jjj 에서 구간을 받아 미션 처리

# Python packages
import rospy
import sys, os
import numpy as np

from macaron_5.msg import Traffic, obj_info
# from path_planning_tracking import Path_Tracking
# from path_planning_tracking_dwa_PP import Path_Tracking_DWA
from pure_pursuit_added_stanley import Path_Tracking_PP_Stanley

# sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

# 멈춰야 하는 표지판 => a 표지판과 매칭 시키는 부분
# want_sign = mission_jjj_pickup
# want_sign.target_sign()

sign_dictionary = {
    'delivery_a1': 0,
    'delivery_a2': 1,
    'delivery_a3': 2,
    'delivery_b1': 3,
    'delivery_b2': 4,
    'delivery_b3': 5,
}


class mission_jjj:
    def __init__(self):
        self.stop_section = None
        self.sub_sign = rospy.Subscriber('/traffic_obj', Traffic, self.obj_callback, queue_size=1)
        self.boxes = list()  # 아무 것도 없는 경우 방지
        self.sign_list = ['delivery_a1', 'delivery_a2', 'delivery_a3', 'delivery_b1', 'delivery_b2', 'delivery_b3']

        self.sign_weight_a = [0, 0, 0, 0, 0, 0]  # 순서 대로 a1~a3이며 가까운 순서 대로 배열

        self.sign_weight_b = [[0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0]]  # 순서 대로 b1~b3이며 가까운 순서 대로 배열

        self.target_sign = ''
        self.flag_b = False
        self.sign = 'a'
        self.step = 0
        self.pick_up_path = "snu_pickup_0725.npy"
        self.delivery_path = "snu_deliver_0725.npy"
        self.pick_up_PT = Path_Tracking_PP_Stanley(self.pick_up_path)
        self.delivery_PT = Path_Tracking_PP_Stanley(self.delivery_path)

    def obj_callback(self, data):
        self.boxes = list()
        for sign in data.obj:
            if sign.ns in self.sign_list:  # 표지판인 경우만 detect
                # 넓이와 class 이름 tuple 형태로 추가
                self.boxes.append(((abs(sign.xmin - sign.xmax)) * (abs(sign.ymin - sign.ymax)), sign.ns))
        # [(넓이, 클래스 이름)] 저장

    def process_sign_a(self):
        global sign_dictionary
        print("process_sign_a....")

        if len(self.boxes) == 1:  # 3개만 인식 했을 때 실행 하게 끔 -> 표지판 구간에 들어 가기 전에 인지/판단 마치기            
            self.sign_weight_a[sign_dictionary[self.boxes[0][1]]] += 1  # a 표지판 가중치 증가
            for key, value in sign_dictionary.items():  # key : delivery_, value: number
                if value == (self.sign_weight_a.index(max(self.sign_weight_a)) + 3):  # a에서 3씩 증가해서 b 표지판으로 이동
                    self.target_sign = key  # 목표 표지판 저장

            print("delivery_a: ", self.target_sign)  # 목표 표지판(b) 출력
            return True, self.boxes[0][0]  # True, 넓이 return
        else:
            return False, 0  # False, 0 return

    def process_sign_b(self):
        global sign_dictionary
        print("process_sign_b....")

        if len(self.boxes) == 3:  # 3개만 인식 했을 때 실행 하게 끔 -> 표지판 구간에 들어 가기 전에 인지/판단 마치기
            self.boxes = sorted(self.boxes, key=lambda x: x[0], reverse=True)  # 크기가 큰 순서 대로 출력

            for i in range(3):
                self.sign_weight_b[i][sign_dictionary[self.boxes[i][1]]] += 1  # 가중치 증가

            return True, self.boxes[0][0]  # 가장 큰 표지판만 return
        else:
            return False, 0  # False, 0 return

    def stop_mission(self):
        global sign_dictionary
        self.flag_b = True

        # a 표지판에서 멈춰야하는 b 표지판 넘겨받음 (target)
        want_sign = self.target_sign
        # 멈추는 구간과 class 같이 저장 (5m 단위)

        signs_list = list()

        for i in range(3):
            for key, value in sign_dictionary.items():  # key : delivery_, value: number
                if value == self.sign_weight_b[i].index(max(self.sign_weight_b[i])):
                    signs_list.append(key)

        print(signs_list)  # 표지판 순서 대로 출력

        print("======================")
        print('sign_list: ', signs_list)
        print('want sign: ', self.target_sign)
        print("======================")

        # 인식을 한 경우 에만 출력
        if signs_list:
            # 5미터, 10미터, 15미터 앞에서 멈추기
            self.stop_section = [(68.9, signs_list[0]), (72.4, signs_list[1]), (75.2, signs_list[2])]

            for state, sign in self.stop_section:
                if sign == want_sign:  # 만약 원하는 표지판 이라면
                    return state, sign  # 멈추는 구간 알려 주기

        else:  # 인식에 실패한 경우
            return 9999, False

    def update(self, pose, heading, sign):
        if sign == 'a':
            self.steer = self.pick_up_PT.gps_tracking(pose, heading)
        elif sign == 'b':
            self.steer = self.delivery_PT.gps_tracking(pose, heading)
        else:
            pass

    def run_a(self, pose, heading):
        self.sign = 'a'
        self.update(pose, heading, self.sign)

    def run_b(self, pose, heading):
        self.sign = 'b'
        self.update(pose, heading, self.sign)
