#!/usr/bin/env python
# -*-coding:utf-8-*-

# Python Package
import rospy
import sys, os
import time
import cv2
import numpy as np

from std_msgs.msg import Int32
from macaron_6.msg import Traffic, obj_info


## 정지선을 검출한다 -> 수평선
## 수평선의 갯수가 일정 이상이면 speed = 0을 주고 멈추게 한다
## speed = 0 인 순간부터 신호등 검출을 한다.

## 정지선을 검출한다 -> 수평선
## 수평선의 개수가 일정 이상이면 그때부터 인지판단 구간으로 한다. -> 아마 화살표부터 검출할거니까, 인지판단 구간으로 줘도 괜찮을듯

class mission_traffic:
    def __init__(self):

        self.sub_sign = rospy.Subscriber('/traffic_obj', Traffic, self.obj_callback, queue_size=1)
        self.pub_speed = rospy.Publisher('speed', Int32, queue_size=1)
        self.traffic_flag = 0
        self.done = False
        self.stop = True

        # 정지선 검출할 카메라 설정
        self.cap = cv2.VideoCapture(4) # 10
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)  # 해상도 조절해주기,웹캠사용시 필요
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

        # 최초 인지 카운트
        self.left_green_count = 0
        self.red_count = 0

        # 빨간불 인지 후 초록불 인지 카운트
        self.start_count = 0

        self.traffic = [0, 0, 0, 0, 0]

        self.interval_time = 60
        self.start_time = None

    def obj_callback(self, data):
        for cl in data.obj:
            if cl.ns == "red_3" or cl.ns == "red_4":
                self.traffic[0] += 1
                self.traffic[1] = 0
                self.traffic[2] = 0
                self.traffic[3] = 0
                self.traffic[4] = 0
            elif cl.ns == "orange_3" or cl.ns == "orange_4":
                self.traffic[0] = 0
                self.traffic[1] += 1
                self.traffic[2] = 0
                self.traffic[3] = 0
                self.traffic[4] = 0
            elif cl.ns == "left_green_4":
                self.traffic[0] = 0
                self.traffic[1] = 0
                self.traffic[2] += 1
                self.traffic[3] = 0
                self.traffic[4] = 0
            elif cl.ns == "straight_green_4" or cl.ns == "green_3":
                self.traffic[0] = 0
                self.traffic[1] = 0
                self.traffic[2] = 0
                self.traffic[3] += 1
                self.traffic[4] = 0
            elif cl.ns == "all_green_4":
                self.traffic[0] = 0
                self.traffic[1] = 0
                self.traffic[2] = 0
                self.traffic[3] = 0
                self.traffic[4] += 1
            else:
                pass
            # print("self.traffic : ",self.traffic)

    # 필요없지만 예의상.... ...
    def reset(self):

        self.traffic_flag = 0
        self.done = False
        self.stop = True

        # 최초 인지 카운트
        self.left_green_count = 0
        self.red_count = 0

        # 빨간불 인지 후 초록불 인지 카운트
        self.start_count = 0

        self.traffic = [0, 0, 0, 0, 0]

        self.interval_time = 60
    
    # 정지선 인지하는 함수(카메라 인덱스 설정은 init에 있음)
    def stop_line(self):
        
        if self.start_time is None:
            self.start_time = time.time()
            self.horizontal_lines = 0
            
        if time.time() - self.start_time < 1:
            ret, frame = self.cap.read()
            
            # 흰색 부분만 선택
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_white = np.array([0, 0, 200])
            upper_white = np.array([180, 255, 255])
            mask = cv2.inRange(hsv, lower_white, upper_white)
            result = cv2.bitwise_and(frame, frame, mask=mask)
            
            # 선택된 흰색 부분은 흰색으로, 나머지는 검은색으로 만들기 
            result[mask == 0] = [0, 0, 0]
            result[mask != 0] = [255, 255, 255]
            
            gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150, apertureSize=3)
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=100, maxLineGap=10)
            
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    if abs(y2 - y1) < 0.1*(x2 - x1):  # 기울기의 절대값이 0.1보다 작은 경우만 수평선으로 간주
                        if 50 < y1 < 480 and 50 < y2 < 480:  

                            cv2.line(result, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            self.horizontal_lines += 1
            if cv2.waitKey(1):
                cv2.imshow('frame', result)
        else:
            self.start_time = time.time()
            print("Detected horizontal lines: ", self.horizontal_lines)
            self.horizontal_lines = 0

        # if time.time() - self.start_time >= 1:  # 2초마다 수평선의 개수 출력
            # print("Detected horizontal lines: ", self.horizontal_lines)

            #################################
            # 테스트 하면서 바꿔야하는 부분, 화살표에서의 수평선의 개수 범위를 어떻게 줄건지
            #
            # 일정 이상 수평선이 그어지면 인지판단 시작, 즉 인지판단이 시작될 th값을 구해야함
            #
            # 토요일에 테스트했을땐, 정지선이 카메라에 보이는 시점부터 유의미하게 값이 차이가 났음!
            # 그래서 th 개념으로 접근해도 알고리즘 활용 가능할거같아
            #################################
        
        
        if self.horizontal_lines > 15:
            return 1
        else:
            return 0
            # else:
            #     self.start_time = time.time()
            #     self.horizontal_lines = 0

    def run(self):
        speed = 70
        print('traffic_flag: ', self.traffic_flag)
        # 인지판단 구간 돌입 확인 구간
        if self.traffic_flag == 0:
            
            speed = 70
            
            # t = time.time()
            # while time.time() - t < 0.3:
            #     print("FLAG 0")
            #     print(speed)

            decision = self.stop_line()
            # t = time.time()
            # while time.time() - t < 0.3:
            #     print(type(decision))

            if decision == 1:
                self.traffic_flag = 1

        # 인지판단 중
        elif self.traffic_flag == 1:
            speed = 40

            print("FLAG 1")
            # print("self.traffic : ",self.traffic)

            # 2번: 좌회전, 4번: 직좌 신호
            if self.traffic[2] > 5 or self.traffic[4] > 5:
                # print("가는거니 가는거니 가는거니")
                self.left_green_count += 1
                self.red_count = 0

                # 해당 변수값 조정을 통해 새로 짠 GPS 없는 신호등 잘 작동되도록 해주어야함
                # 멈추면 안되는데 멈췄다 -> 10보다 작은값주기
                if self.left_green_count >= 10:
                    # print("고고고고고고고고곡고고고고고고고고고고고고")
                    self.stop = False
                    self.traffic_flag = 2

            # 빨간불이거나 직진일 때 멈추기
            elif self.traffic[0] > 10 or self.traffic[1] >10 or self.traffic[3] > 10:
                self.left_green_count = 0
                self.red_count += 1

                # 해당 변수값 조정을 통해 새로 짠 GPS 없는 신호등 잘 작동되도록 해주어야함
                # 멈춰야되는데 그냥 지나가버린다 -> 15보다 작은 값 주기
                if self.red_count >= 15:
                    print("난 멈춘다 멈춘다 멈춘다 나도 멈춰조...")
                    self.traffic_flag = 2

        # 첫번째 인지판단 끝남
        elif self.traffic_flag == 2:
            print("FLAG 2")
            end_time = time.time()
            #print("self.traffic : ", self.traffic)

            # 정지선에 멈춰 있는 상황
            if self.stop is True and time.time() < end_time + self.interval_time:
                speed = 0
                # 좌회전 신호 보는 중
                if self.traffic[2] > 1 or self.traffic[4] > 1:
                    self.start_count += 1

                else:
                    self.start_count = 0

                # 좌회전 신호를 너무 많이보고 출발 시 3보다 낮은 값 주기
                if self.start_count >= 3:
                    speed = 90
                    self.traffic_flag = 3

            # Flag 1 부분에서 stop == False로 들어오면 speed = 0 안주고 바로 90으로 달리기
            else:
                speed = 90
                self.traffic_flag = 3

        # 신호등 미션 끝 (정지선 이후 상황)
        elif self.traffic_flag == 3:
            speed = 90
            t = time.time()
            while time.time() - t < 0.3:
                print("done!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            self.done = True
            self.reset()
            print('신호등 미션 끝!\n')
            
        print("speed:", speed)
        self.pub_speed.publish(speed)
        return speed


# if __name__ == "__main__":
#     rospy.init_node("traffic_light")
#     ms = mission_traffic()
#     while True:
#         ms.run()
