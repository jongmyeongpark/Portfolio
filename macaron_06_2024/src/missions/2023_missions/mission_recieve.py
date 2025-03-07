#!/usr/bin/env python
#-*-coding:utf-8-*-
from locale import currency
from tabnanny import check
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


# TODO : self.delivery_loc 좌표 최적화, self.distance의 기준 좌표 최적화
#        self.delivery_path 경로 하나 mapping

class mission_recieve:
    def __init__(self):
        # tracking
        self.delivery_path_name = "kcity_del.npy"
        self.PT = Path_Tracking(self.delivery_path_name)

        # control
        self.steer = 0
        self.speed = 30

        # position
        self.pose = [0, 0]
        self.heading = 0
        
        # bounding box
        self.bbox_size = [[], [], []]
        self.bbox = [[], [], []]  # [xmin, xmax] of a1, a2, a3
        self.average = [[0, 0], [0, 1], [0, 2]]
        self.bbox_max = 0

        # lidar
        self.points = []
        self._points = []
        self._min = 0.4
        self._max = 3.02
        self._stop = 3
        self.empty = 0

        # sign
        self.delivery = [0, 0, 0]  # count of a1, a2, a3
        self.passed = [False, False, False]
        self.order = 1
        self.current_sign = -1
        self.index = 0
        self.next_sign = [0, 0, 0]
        
        self.delivery_zone = ""
        self.delivery_index = -1

        # mission
        self.step = 1
        self.start = False
        self.done = False

        # time estimation
        self.stop = False
        self.scanned = False
        self.checked = False
        self.check_start = 0
        self.ready_to_stop = 0
        self.stop_start = 0
        self.stop_time = 0
        
        self.sub_sign = rospy.Subscriber('/traffic_obj', Traffic, self.obj_callback, queue_size=1)
        self.sub_lidar = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size = 1)
        
    def obj_callback(self, data):
        for cl in data.obj:
            if cl.ns[0:3] == "del":
                if cl.ns[9:11] == "b1":
                    self.delivery[0] += 1
                    self.bbox[0].append(cl.xmin)

                if cl.ns[9:11] == "b2":
                    self.delivery[1] += 1
                    self.bbox[1].append(cl.xmin)

                if cl.ns[9:11] == "b3":
                    self.delivery[2] += 1
                    self.bbox[2].append(cl.xmin)

    def scan_callback(self, data):
        self._points = []

        for p in list(data.ranges[0:240]): ##################
            if self._min < p < self._max:
                self._points.append(p)

        if len(self._points) > 0:
            self.points = self._points
        elif len(self._points) == 0:
            self.empty += 1

        if self.empty > 1:
            self.points = []
            self.empty = 0
    
    def reset(self):
        self.delivery = [0, 0, 0]
        self.bbox = [[], [], []]
        
    def order_sign(self):
        for i in range(3):
            if len(self.bbox[i]) == 0:
                self.reset()
                return
            
        print("bbox :", self.bbox)
        
        if len(self.bbox[0]) > 3 and len(self.bbox[1]) > 3 and len(self.bbox[2]) > 3:
            for i in range(3):
                self.average[i] = [sum(self.bbox[i]) / len(self.bbox[i]), i]
                
            self.step = 1
            self.average = sorted(self.average, reverse=True)
            self.current_sign = self.average[0][1]

            for i in range(3):
                self.next_sign[i] = self.average[i][1]

    def check_sign(self):
        # self.speed = 40 ####################

        checked = False
        index = -1
        avg = 0
        _max = 0

        if self.current_sign == 0:
            print("b1")
        elif self.current_sign == 1:
            print("b2")
        elif self.current_sign == 2:
            print("b3")
            
        if self.current_sign == -1:
            self.speed = 50 ####################
            print(self.bbox)
            for i, bbox in enumerate(self.bbox):
                if len(bbox) > 1:
                    checked = True
                    print(bbox)
                    avg = sum(bbox) / len(bbox)
                    # if len(bbox) > 2:
                    #     checked = True

                    if avg > _max: ###############
                    # if avg > _max and not self.passed[i]:
                        _max = avg
                        index = i 
                        self.current_sign = index
                        self.bbox_max = _max

            if checked:
                self.reset() 

            print("index :", index) 

        else:
            self.speed = 30 ####################
            print("len(self.points) :", len(self.points))
            if self.current_sign == self.delivery_index and len(self.points) == 0: # 
                self.step = 3
            elif len(self.points) > 0:
                if not self.checked:
                    self.check_start = time.time()
                    self.checked = True
                elif time.time() - self.check_start > 1.5:
                    self.checked = False
                    self.passed[self.current_sign] = True
                    self.current_sign = -1

    def pass_target(self): # step 2
        if len(self.points) > 0:
            targets = self.points
            num = len(targets)
            avg = sum(targets) / num
            
            # print(targets)
            print("count :", num)
            print("avg :", avg)

            #########################
            if num > 5 and self._min < avg < self._stop:
                self.step = 1
                self.passed[self.current_sign] = True
                self.reset()
        else:
            print("scan nothing..........")
        
    def find_target(self):  # step 3
        self.speed = 40 ####################
        if len(self.points) > 0:
            targets = self.points
            num = len(targets)
            avg = sum(targets) / num

            print(targets)
            print("count :", num)
            print("avg :", avg)

            if num > 0 and self._min < avg < self._stop:
                print("sleep...........")
                # time.sleep(1) ###################

                # self.speed = -201
                self.step = 4
        else:
            print("scan nothing..........")
                    
    def stop_to_recieve(self): # step 4
        if not self.scanned:
            self.ready_to_stop = time.time()
            self.scanned = True
        elif time.time() - self.ready_to_stop > 5.5:
            if not self.stop:
                self.stop_start = time.time()
                self.stop = True
                

            if self.stop and time.time() - self.stop_start < 6:
                print("stop!!!!!!!!!!!!")
                self.speed = -201

            if time.time() - self.stop_start > 6:
                self.speed = 100
                self.done = True
    
    def update(self, pose, heading, delivery_zone):
        self.pose = pose
        self.heading = heading
        self.steer = self.PT.gps_tracking(pose, heading)
        
        if not self.start:
            self.delivery_zone = delivery_zone
            self.start = True
            
            if delivery_zone == "a1":
                self.delivery_index = 0
            elif delivery_zone == "a2":
                self.delivery_index = 1
            elif delivery_zone == "a3":
                self.delivery_index = 2
 
        print("current_sign :", self.current_sign)
        print("delivery_zone :", self.delivery_zone)
        
        if self.index > 1 and not self.done and not self.stop:
            print("Error : Finished without mission complete......")

    def run(self, pose, heading, delivery_zone):
        self.update(pose, heading, delivery_zone)
        
        if self.step == 0:
            print("step 0..............")
            self.order_sign()
        elif self.step == 1:
            print("step 1..............")
            self.check_sign()
        elif self.step == 2:
            print("step 2..............")
            self.pass_target()
        elif self.step == 3:
            print("step 3..............")
            self.find_target()
        elif self.step == 4:
            print("step 4..............")
            self.stop_to_recieve()
            
        return self.speed, self.steer
        
'''
class mission_recieve:
    def __init__(self):
        self.sub_sign = rospy.Subscriber('/traffic_obj', Traffic, self.obj_callback, queue_size=1)
        
        ### 밑에 두 경로 수정
        self.delivery_path_name = "kcity_del.npy" # a표지판이 있는 쪽이 아닌 b표지판이 있는 도로 쪽의 가장자리 경로
        self.delivery_path = "/home/macaron/catkin_ws/src/macaron_4_advanced/path/npy_file/path/kcity_del.npy"
        
        # 첫번째는 b1, 두번째는 b2, 세번째는 b3의 x,y 좌표(state.py에서의 erp.pose)
        ### 여기 수정
        # 서울대 픽업 테스트
        # self.delivery_loc = [[931343.8129397788, 1929831.2024728793], # 세 좌표 모두 kcity가서 최적화
        #                      [931349.5947332676, 1929839.7092555398],
        #                      [931357.4958338822, 1929851.7934283696]]
        # 서울대 최종 리허설
        self.delivery_loc = [[935639.4774716623, 1916212.8102905804],
                             [935639.5952835067, 1916207.3508087827],
                             [935639.731906244, 1916199.7166271163]]
        self.PT = Path_Tracking(self.delivery_path_name)
        self.DP = GlobalPath(self.delivery_path)
        # self.parking_s = 0  # 주차 구역의 s좌표 -> 정차해야하는 s좌표

        self.bbox_size = [0, 0, 0]
        self.bbox = [[0, 0], [0, 0], [0, 0]]
        self.delivery = [0, 0, 0]
        self.delivery_index = -1
        self.delivery_zone = ""

        self.speed = 0
        self.steer = 0

        self.pose = []
        self.heading = 0
        self.s = 0  # 현재 세부 경로 기준 (state 경로 x) s좌표 -> update 함수에서 계속 갱신됨

        self.mission_start = False

        self.step = 0
        self.location_decision = False
        self.done = False
        self.stop = False
        # self.stop_time = 1 # 일단 1초로 설정
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
                    
    def set_delivery_index(self):
        if self.delivery_zone == "a1":
            self.delivery_index = 0
            self.step = 1 # 결정되면 다음 step으로
        elif self.delivery_zone == "a2":
            self.delivery_index = 1
            self.step = 1  # 결정되면 다음 step으로
        elif self.delivery_zone == "a3":
            self.delivery_index = 2
            self.step = 1 # 결정되면 다음 step으로
        else:
            print("Not Detected.........")
    
    def set_location(self):
        print("self.bbox[self.delivery_index][0] :", self.bbox[self.delivery_index][0])
        print("self.bbox_size[self.delivery_index] :", self.bbox_size[self.delivery_index])
        if self.delivery[self.delivery_index] > 15:
            if self.bbox_size[self.delivery_index] > 20 and self.bbox[self.delivery_index][0] > 200 and not self.location_decision:
                self.ready_to_stop = time.time()
                self.location_decision = True

            print("time :", time.time() - self.ready_to_stop)
            if self.location_decision and time.time() - self.ready_to_stop > self.stop_delay:
                self.step = 2

    def distance(self, mission_loc): # 재사용성이나 깔끔함을 위해서 그냥 함수 하나 만듦 -> 내가 원하는 위치에 왔는지 확인
        d = math.sqrt(((self.pose[0] - mission_loc[0])**2) + ((self.pose[1] - mission_loc[1])**2))*10
        print("distance :", d)
        
        if d < 5:  # 기준은 kcity 가서 최적화
            return True
        else:
            return False
    
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

    def update(self, pose, heading, delivery_zone):  # 현재 s좌표 업데이트 + 배달 경로 트래킹해서 steer 업데이트
        if self.mission_start == False:
            self.delivery_zone = delivery_zone # 미션 시작 시 한번만 배달구역 받음
            
        self.pose = pose
        self.heading = heading
        self.s, _ = self.DP.xy2sl(pose[0], pose[1])
        self.steer = self.PT.gps_tracking(pose, heading)
        self.mission_start = True
        self.speed = 80  # 속도는 조절 가능
        print("delivery s :", self.s)
        print("self.delivery_index :", self.delivery_index)
        
    def run(self, pose, heading, delivery_zone):  # delivery_zone은 state.py에서 정해진 배달구역 ("a1" or "a2" or "a3")
        self.update(pose, heading, delivery_zone)

        if self.step == 0: # 받은 배달구역을 확인해서 배달 좌표(delivery_index)를 확정하는 단계 -> self.delivery_loc이 결정됨
            self.set_delivery_index()
        elif self.step == 1:
            self.set_location()
        elif self.step == 2:
            print("Delivery :", self.delivery_zone)
            self.parking()  # 원하는 x, y좌표에 도달할 때까지 직진 -> 직진은 update함수에서
        elif self.step == 3:
            self.done = True
            print("배달 미션 완료...")

        return self.speed, self.steer
'''
'''

class mission_recieve:
    def __init__(self):
        ### 밑에 두 경로 수정
        # a표지판이 있는 쪽이 아닌 b표지판이 있는 도로 쪽의 가장자리 경로
        self.delivery_path_name = "kcity_del.npy"
        self.delivery_path = "/home/macaron/catkin_ws/src/macaron_4_advanced/path/npy_file/path/kcity_del.npy"

        # 첫번째는 b1, 두번째는 b2, 세번째는 b3의 x,y 좌표(state.py에서의 erp.pose)
        ### 여기 수정
        # 서울대 픽업 테스트
        # self.delivery_loc = [[931343.8129397788, 1929831.2024728793], # 세 좌표 모두 kcity가서 최적화
        #                      [931349.5947332676, 1929839.7092555398],
        #                      [931357.4958338822, 1929851.7934283696]]
        # 서울대 최종 리허설
        self.delivery_coord = np.load(self.delivery_path)
        self.delivery_loc = [[935639.4774716623, 1916212.8102905804],
                             [935639.5952835067, 1916207.3508087827],
                             [935639.731906244, 1916199.7166271163]]
        self.delivery_loc = [self.delivery_coord[90],
                            self.delivery_coord[95],
                            self.delivery_coord[99]]
        self.PT = Path_Tracking(self.delivery_path_name)
        self.DP = GlobalPath(self.delivery_path)
        # self.parking_s = 0  # 주차 구역의 s좌표 -> 정차해야하는 s좌표
        self.delivery = [0, 0, 0]
        self.delivery_index = -1
        self.delivery_zone = ""
        self.speed = 0
        self.steer = 0
        self.pose = []
        self.heading = 0
        self.s = 0  # 현재 세부 경로 기준 (state 경로 x) s좌표 -> update 함수에서 계속 갱신됨
        self.step = 0
        self.done = False
        self.stop = False
        self.mission_start = False
        self.current_time = 0

    def set_delivery_index(self):
        if self.delivery_zone == "a1":
            self.delivery_index = 0
            self.step = 1  # 결정되면 다음 step으로
        elif self.delivery_zone == "a2":
            self.delivery_index = 1
            self.step = 1  # 결정되면 다음 step으로
        elif self.delivery_zone == "a3":
            self.delivery_index = 2
            self.step = 1  # 결정되면 다음 step으로
        else:
            print("Not Detected.........")

    def distance(self, mission_loc):  # 재사용성이나 깔끔함을 위해서 그냥 함수 하나 만듦 -> 내가 원하는 위치에 왔는지 확인
        d = math.sqrt(((self.pose[0] - mission_loc[0])**2) + ((self.pose[1] - mission_loc[1])**2))
        print("distance :", d)

        if d < 3:  # 기준은 kcity 가서 최적화
            return True
        else:
            return False

    def parking(self):  # 원하는 배달 구역에 정차
        if self.stop == False:
            self.current_time = time.time()
        # 결정된 delivery_index(배달 구역)에 정차하기 위해서 계속 거리를 측정
        if self.distance(self.delivery_loc[self.delivery_index]) and time.time() - self.current_time < 12:
            print("stop!!")
            self.stop = True
            self.speed = 0  # 원하는 배달 구역에 도착했을 경우 정지
        if time.time() - self.current_time > 12:  # 정차 시간이 지났을 때 다음 step으로
            self.stop = False
            self.step = 2

    def update(self, pose, heading, delivery_zone):  # 현재 s좌표 업데이트 + 배달 경로 트래킹해서 steer 업데이트
        if self.mission_start == False:
            self.delivery_zone = delivery_zone  # 미션 시작 시 한번만 배달구역 받음

        self.pose = pose
        self.heading = heading
        self.s, _ = self.DP.xy2sl(pose[0], pose[1])
        self.steer = self.PT.gps_tracking(pose, heading)
        self.mission_start = True
        self.speed = 60  # 속도는 조절 가능
        print("delivery s :", self.s)
        self.delivery_index = 1
        self.step = 1

    # delivery_zone은 state.py에서 정해진 배달구역 ("a1" or "a2" or "a3")
    def run(self, pose, heading, delivery_zone):
        self.update(pose, heading, delivery_zone)
        # 받은 배달구역을 확인해서 배달 좌표(delivery_index)를 확정하는 단계 -> self.delivery_loc이 결정됨
        if self.step == 0:
            self.set_delivery_index()
        elif self.step == 1:
            print("Delivery :", self.delivery_zone)
            self.parking()  # 원하는 x, y좌표에 도달할 때까지 직진 -> 직진은 update함수에서
        elif self.step == 2:
            self.done = True
            print("배달 미션 완료...")
        return self.speed, self.steer
'''
'''
state.py 예시 코드

Mission_recieve = mission_recieve() # 객체 선언

# (중간 생략)
# elif문에서
............................................
# 픽업 미션에서 배달 구역을 받았다는 가정 하에 작성
speed, steer = Mission_recieve.run(erp.pose, erp.heading, Mission_pickup.mission_zone)

if Mission_recieve.done:
    MS.mission_done()
............................................
'''
