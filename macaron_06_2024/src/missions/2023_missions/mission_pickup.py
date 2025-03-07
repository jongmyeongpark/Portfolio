#!/usr/bin/env python
#-*-coding:utf-8-*-

import rospy
import os
import sys
import numpy as np
import time
import math

from macaron_5.msg import Traffic
from sensor_msgs.msg import LaserScan
from path_planning_tracking import Path_Tracking
from trajectory_planner import TrajectoryPlanner
from global_path import GlobalPath
from sub_erp_state import sub_erp_state


# state 경로는 차선 중앙의 경로이고 pickup 경로는 오른쪽 차선을 밟는 경로
# done이 True로 변하면 cruising으로 넘어가므로 state의 경로를 따라감

# TODO : self.pickup_loc 좌표 최적화, self.distance의 기준 좌표 최적화
#        self.pickup_path 경로 하나 mapping

class mission_pickup:
    def __init__(self):
        # tracking
        self.pickup_path_name = "kcity_pickup.npy"
        self.PT = Path_Tracking(self.pickup_path_name)
        
        # control
        self.steer = 0
        self.speed = 60
        
        # position
        self.pose = [0, 0]
        self.heading = 0
        
        # bounding box
        self.bbox_size = [[], [], []]
        self.bbox = [[], [], []]  # [xmin, xmax] of a1, a2, a3
        
        # lidar
        self.points = []
        self._points = []
        self._min = 0.4
        self._max = 4.5
        self._stop = 3.8
        self.empty = 0
        
        # sign
        self.delivery = [0, 0, 0]  # count of a1, a2, a3
        self.delivery_zone = ""
        self.delivery_index = -1
        
        # mission
        self.step = 0
        self.done = False
        
        # time estimation
        self.stop = False
        self.scanned = False
        self.ready_to_stop = 0
        self.stop_start = 0
        self.stop_time = 0
        
        self.sub_sign = rospy.Subscriber('/traffic_obj', Traffic, self.obj_callback, queue_size=1)
        self.sub_lidar = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size = 1)
        
    def obj_callback(self, data):
        for cl in data.obj:
            if cl.ns[0:3] == "del":
                if cl.ns[9:11] == "a1":
                    self.delivery[0] += 1
                    if self.step == 1 and self.delivery_index == 0:
                        self.bbox[self.delivery_index].append(cl.xmin)

                if cl.ns[9:11] == "a2":
                    self.delivery[1] += 1
                    if self.step == 1 and self.delivery_index == 1:
                        self.bbox[self.delivery_index].append(cl.xmin)

                if cl.ns[9:11] == "a3":
                    self.delivery[2] += 1
                    if self.step == 1 and self.delivery_index == 2:
                        self.bbox[self.delivery_index].append(cl.xmin)
                    
    def scan_callback(self, data):
        self._points = []

        for p in list(data.ranges[120:320]):
            if self._min < p < self._max:
                self._points.append(p)

        if len(self._points) > 3:
            self.points = self._points 
        elif len(self._points) < 1:
            self.empty += 1
        
        if self.empty > 2:
            self.points = []
            self.empty = 0
                
    def get_sign(self):
        for i, x in enumerate(self.delivery):
            if x > 10:
                if i == 0:
                    self.delivery_zone = "a1"
                elif i == 1:
                    self.delivery_zone = "a2"
                elif i == 2:
                    self.delivery_zone = "a3"
                    
                self.delivery_index = i
                self.step = 1
                break
            
    def wait_for_scan(self):
        if len(self.bbox[self.delivery_index]) > 10:
            count = 0
            for xmin in self.bbox[self.delivery_index]:
                if xmin > 190:
                    count += 1
                    
            print("bbox :", count)

            if count > 3:
                self.step = 2 
            else:
                self.bbox = [[], [], []]  # reset

    def find_target(self):
        self.speed = 40

        if len(self.points) > 0:
            targets = self.points
            num = len(targets)
            avg = sum(targets) / num
            
            # print(targets)
            print("count :", num)
            print("avg :", avg)

            ########################
            if num > 2 and self._min < avg < self._stop:
                # time.sleep(2) ###########################
                # self.speed = -201
                
                self.step = 3
        else:
            print("scan nothing..........")
        
    def stop_to_pickup(self):
        if not self.scanned:
            self.ready_to_stop = time.time()
            self.scanned = True
        elif time.time() - self.ready_to_stop > 2.5:
            if not self.stop:
                self.stop_start = time.time()
                self.stop = True
                
            if self.stop and time.time() - self.stop_start < 6:
                self.speed = -201
            
            if time.time() - self.stop_start > 6:
                self.speed = 100
                self.done = True
                
    def update(self, pose, heading):
        self.pose = pose
        self.heading = heading
        self.steer = self.PT.gps_tracking(pose, heading)
        # self.step = 2 # 라이다 테스트

    def run(self, pose, heading):
        self.update(pose, heading)
        
        if self.step == 0:
            print("step 0")
            self.get_sign()
        elif self.step == 1:
            print("step 1")
            self.wait_for_scan()
        elif self.step == 2:
            print("step 2")
            self.find_target()
        elif self.step == 3:
            print("stop!!!!!!!!")
            self.stop_to_pickup()
            
        return self.speed, self.steer
            

'''
# 카메라만
class mission_pickup:
    def __init__(self):
        self.sub_sign = rospy.Subscriber('/traffic_obj', Traffic, self.obj_callback, queue_size=1)
        
        ### 여기 경로 수정
        self.pickup_path_name = "kcity_pickup.npy"
        self.pickup_path = "/home/macaron/catkin_ws/src/macaron_4_advanced/path/npy_file/path/kcity_pickup.npy"
        self.pickup_loc = [935658.5279256748, 1916154.6391563918] ### 정차 구간 수정
        self.pickup_coord = np.load(self.pickup_path)
        self.PT = Path_Tracking(self.pickup_path_name)
        self.DP = GlobalPath(self.pickup_path)
        self.parking_s = 0 # 주차 구역의 s좌표 -> 정차해야하는 s좌표
        
        # bounding box의 좌표
        self.bbox_size = [0, 0, 0]
        self.bbox = [[0, 0], [0, 0], [0, 0]]
        
        self.delivery = [0, 0, 0]
        self.delivery_zone = ""
        self.index = -1
        
        self.speed = 0
        self.steer = 0
        
        self.pose = []
        self.heading = 0
        self.s = 0 # 현재 세부 경로 기준 (state 경로 x) s좌표 -> update 함수에서 계속 갱신됨
        
        self.step = 0
        self.location_decision = False
        self.done = False
        self.stop = False
        self.stop_time = 0 # 일단 1초로 설정
        self.ready_to_stop = 0
        self.stop_delay = 2
        self.current_time = 0

    def obj_callback(self, data):
        for cl in data.obj:
            if cl.ns[0:3] == "del":
                if cl.ns[9:11] == "a1":
                    self.delivery[0] += 1
                    self.bbox[0][0] = cl.xmin
                    self.bbox[0][1] = cl.xmax
                    self.bbox_size[0] = abs(cl.ymax - cl.ymin)

                if cl.ns[9:11] == "a2":
                    self.delivery[1] += 1
                    self.bbox[1][0] = cl.xmin
                    self.bbox[1][1] = cl.xmax
                    self.bbox_size[1] = abs(cl.ymax - cl.ymin)

                if cl.ns[9:11] == "a3":
                    self.delivery[2] += 1
                    self.bbox[2][0] = cl.xmin
                    self.bbox[2][1] = cl.xmax
                    self.bbox_size[2] = abs(cl.ymax - cl.ymin)

    def check(self): # 배달 표지판이 어느 표지판인지 판단하는 함수
        for i in range(3):
            if self.delivery[i] > 10: # a1, a2, a3 중 30번 이상 받았으면 확실하다고 판단
                if i == 0:
                    self.delivery_zone = "a1"
                    self.index = 0
                elif i == 1:
                    self.delivery_zone = "a2"
                    self.index = 1
                elif i == 2:
                    self.delivery_zone = "a3"
                    self.index = 2
                    
                self.step = 1 # 다음 단계로 넘어감
                
                break
    
    def distance(self, mission_loc):
        d = math.sqrt(((self.pose[0] - mission_loc[0])**2) + ((self.pose[1] - mission_loc[1])**2))*10
        print("distance :", d)
        if d < 5:
            return True
        else:
            return False
    
    def set_location(self):
        print("self.bbox[self.delivery_index][0] :", self.bbox[self.index][0])
        print("self.bbox_size[self.index] :", self.bbox_size[self.index])
        if self.bbox_size[self.index] > 20 and self.bbox[self.index][0] > 200 and not self.location_decision:
            self.ready_to_stop = time.time()
            self.location_decision = True

        print("time :", time.time() - self.ready_to_stop)
        if self.location_decision and time.time() - self.ready_to_stop > self.stop_delay:
            self.step = 2
            
    def parking(self):  # 원하는 배달 구역에 정차
        if self.stop == False:
            self.current_time = time.time()
            self.stop = True

        if time.time() - self.current_time < 12:
            print("stop!!")
            self.speed = 0  # 원하는 배달 구역에 도착했을 경우 정지
        else:
            self.stop = False
            self.step = 3
            
        
    def update(self, pose, heading): # 현재 s좌표 업데이트 + 배달 경로 트래킹해서 steer 업데이트
        self.pose = pose
        self.heading = heading
        self.s, _ = self.DP.xy2sl(pose[0], pose[1])
        self.steer = self.PT.gps_tracking(pose, heading)
        self.speed = 100  # 속도는 조절 가능
        
    def run(self, pose, heading): 
        self.update(pose, heading)
        
        if self.step == 0:
            self.check()
        elif self.step == 1:
            self.set_location()
        elif self.step == 2:
            print("Delivery :", self.delivery_zone)
            print(self.delivery)
            self.parking() # 원하는 s좌표에 도달할 때까지 직진 -> 직진은 update함수에서 
        elif self.step == 3:
            self.done = True
            self.speed = 70
            print("픽업 미션 완료...")
            
        return self.speed, self.steer

'''

'''
# 카메라 + 좌표
class mission_pickup:
    def __init__(self):
        self.sub_sign = rospy.Subscriber(
            '/traffic_obj', Traffic, self.obj_callback, queue_size=1)

        ### 여기 경로 수정
        self.pickup_path_name = "kcity_pickup.npy"
        self.pickup_path = "/home/macaron/catkin_ws/src/macaron_4_advanced/path/npy_file/path/kcity_pickup.npy"
        self.pickup_loc = [[935658.5279256748, 1916154.6391563918]]  # 정차 구간 수정
        self.pickup_coord = np.load(self.pickup_path)
        self.pickup_loc[0] = self.pickup_coord[0]

        self.PT = Path_Tracking(self.pickup_path_name)
        self.DP = GlobalPath(self.pickup_path)
        self.parking_s = 0  # 주차 구역의 s좌표 -> 정차해야하는 s좌표

        # bounding box의 좌표
        # a1, a2, a3, b1, b2, b3 순으로 좌표들을 저장
        self.xmin = [0, 0, 0]
        self.xmax = [0, 0, 0]
        self.ymin = [0, 0, 0]
        self.ymax = [0, 0, 0]
        self.bbox = [self.xmin, self.xmax, self.ymin, self.ymax]
        self.x_mean = [0, 0, 0]

        self.delivery = [0, 0, 0]
        self.delivery_zone = ""

        self.speed = 0
        self.steer = 0

        self.pose = []
        self.heading = 0
        self.s = 0  # 현재 세부 경로 기준 (state 경로 x) s좌표 -> update 함수에서 계속 갱신됨

        self.step = 0
        self.done = False
        self.stop = False
        self.current_time = 0

    def obj_callback(self, data):
        for cl in data.obj:
            if cl.ns[0:3] == "del":
                if cl.ns[9:11] == "a1":
                    self.delivery[0] += 1
                else:
                    pass

                if cl.ns[9:11] == "a2":
                    self.delivery[1] += 1
                else:
                    pass

                if cl.ns[9:11] == "a3":
                    self.delivery[2] += 1
                else:
                    pass

    def check(self):  # 배달 표지판이 어느 표지판인지 판단하는 함수
        for i in range(3):
            if self.delivery[i] > 20:  # a1, a2, a3 중 30번 이상 받았으면 확실하다고 판단
                if i == 0:
                    self.delivery_zone = "a1"
                elif i == 1:
                    self.delivery_zone = "a2"
                elif i == 2:
                    self.delivery_zone = "a3"

                self.step = 1  # 다음 단계로 넘어감
                break

    def distance(self, mission_loc):
        d = math.sqrt(((self.pose[0] - mission_loc[0])**2) +
                      ((self.pose[1] - mission_loc[1])**2))*10
        print("distance :", d)
        if d < 5:
            return True
        else:
            return False

    def parking(self):  # 원하는 배달 구역에 정차
        # S = sub_erp_state()
        if self.stop == False:
            self.current_time = time.time()
        if self.distance(self.pickup_loc[0]) and time.time() - self.current_time < 12:
            print("stop!!")
            self.stop = True
            self.speed = -201  # 원하는 배달 구역에 도착했을 경우 정지
        if time.time() - self.current_time > 12:
            self.stop = False
            self.step = 2

    def update(self, pose, heading):  # 현재 s좌표 업데이트 + 배달 경로 트래킹해서 steer 업데이트
        self.pose = pose
        self.heading = heading
        self.s, _ = self.DP.xy2sl(pose[0], pose[1])
        self.steer = self.PT.gps_tracking(pose, heading)

    def run(self, pose, heading):
        self.update(pose, heading)

        if self.step == 0:
            self.check()
            self.speed = 90  # 속도는 조절 가능
        elif self.step == 1:
            print("Delivery :", self.delivery_zone)
            print(self.delivery)
            self.parking()  # 원하는 s좌표에 도달할 때까지 직진 -> 직진은 update함수에서
        elif self.step == 2:
            self.done = True
            self.speed = 70
            print("픽업 미션 완료...")

        return self.speed, self.steer

'''
'''
state.py 예시 코드

Mission_pickup = mission_pickup() # 객체 선언

# (중간 생략)
# elif문에서
............................................
Mission_pickup.run(erp.pose, erp.heading)

if Mission_pickup.done:
    MS.mission_done()
............................................
# 만약에 주차구역을 꺼내서 쓰고 싶으면 Mission_pickup.delivery_zone을 가져다 쓰면 됨
# -> "a1", "a2", "a3" 중 하나가 return
'''
