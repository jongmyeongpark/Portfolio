#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy
import os, sys
import numpy as np
import time
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/path_planning")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/src/path_planning")

from sensor_msgs.msg import PointCloud
from macaron_6.msg import erp_write
from cone_detection import Cone_detection
from geometry_msgs.msg import Point32

from path_planning_tracking import Path_Tracking
from path_planning_tracking_pure_pursuit import Path_Tracking_PP
from pure_pursuit_added_stanley import Path_Tracking_PP_Stanley
from trajectory_planner import TrajectoryPlanner
from global_path import GlobalPath

from planning_parking_parallel_3arc import Parking_Parallel_3arc
from sub_erp_state import sub_erp_state
from mission_cruising import *

from lidar_module import lidar_module

'''
step0 : 주차 경로 탐색 -> 경로 생성
step1: 직진 경로 추종 시작
step2: 후진 경로 추종 시작
step3: 앞으로 살짝 이동
step4: 10초간 정지
step5: 출차
'''

PATH_ROOT = os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))+"/path/npy_file/path/"

CRITERIA = 0

class Find_parking_lot:
    def __init__(self):
        cone_detection = Cone_detection()
        self.cones = rospy.Subscriber('/cone_lidar', PointCloud, self.callback, queue_size=1)
        self.criteria_pub = rospy.Publisher('/criteria_point', PointCloud, queue_size=1)
        self.path_pub = rospy.Publisher('/Parking_Path', PointCloud, queue_size=10)
        self.Finding = True
        self.Found = False
        self.dis = 0

    def callback(self, input_rosmsg):
        points = [[i.x, i.y, i.z] for i in input_rosmsg.points]
        points.sort(key=lambda x: x[0])
        self.sorting(points)

    def sorting(self, points):
        lists = [[]]
        current_list_index = 0
        if len(points) == 0:
            pass
        else:
            lists[0].append(points[0])
            for i in range(0, len(points) - 1):
                x_diff = points[i + 1][0] - points[i][0]
                if x_diff < 0.5:
                    lists[current_list_index].append(points[i + 1])
                else:
                        current_list_index += 1
                        lists.append([points[i + 1]])
                for i in range(len(lists)):
                    lists[i].sort(key=lambda x: abs(x[1]))
                
                final_list = [lst[0] for lst in lists]     
                # criteria_point = [0, 0]
                # for i in range(len(final_list) - 1):
                #     diff_x = final_list[i + 1][0] - final_list[i][0]
                #     if diff_x > 3:
                #         criteria_point = final_list[i]
                #         self.Found = True
                #         break
                #     else:
                #         pass
                criteria_point = [0, 0]
                for i in range(len(final_list) - 1):
                    current_y = final_list[i][1]
                    if abs(current_y) > 3:
                        criteria_point = final_list[i - 1]
                        self.Found = True
                        ass = PointCloud()
                        ass.header.frame_id = 'map'
                        bss = Point32()
                        bss.x = criteria_point[0]
                        bss.y = criteria_point[1]
                        bss.z = 0
                        ass.points.append(bss)
                        self.criteria_pub.publish(ass)
                        break
                    else:
                        pass
                
                self.dis = abs(criteria_point[1]) + 1.5
                x = criteria_point[0]
                # self.criteria_point = [(x, 0, 0)]
                self.criteria_point = [(x, 0, 0)]

    def point2gps(self, pose, heading):
        print('경로 생성 시작')
        lidar = lidar_module()
        ## 상대경로가 같고 싶을 때. (상대 경로를 만든 다음 tf2tm돌리는거는 시간이 많이 들지 않을까?)
        # point = [self.criteria_point[0][0], self.criteria_point[0][1]]
        # start = (0, 0)
        # parking_parallel = Parking_Parallel_3arc(start, point, A=1.8, d=self.dis)
        # path1, path2, path3, path4 = parking_parallel.final_path()
        # path = np.concatenate((path1, path2, path3, path4))
        # css = PointCloud()
        # css.header.frame_id = 'map'
        # for i in range(len(path)):
        #     dss = Point32()
        #     dss.x = path[i][0]
        #     dss.y = path[i][1]
        #     dss.z = 0
        #     css.points.append(dss)
        # self.path_pub.publish(css)
        # tm_path = lidar.tf2tm(path, pose[0], pose[1], heading)

        # 기준점을 gps값으로 변환 후 경로 생성.
        start = pose
        tm_point = lidar.tf2tm(self.criteria_point, pose[0], pose[1], heading)
        final_point = [tm_point[0][0], tm_point[0][1]]
        parking_parallel = Parking_Parallel_3arc(start, final_point, A=1.8, d=self.dis)
        path1, path2, path3, path4 = parking_parallel.final_path()
        parking_parallel.save_path2npy(path1, 'parking_parallel_path1')    
        parking_parallel.save_path2npy(path2, 'parking_parallel_path2')
        parking_parallel.save_path2npy(path3, 'parking_parallel_path3')
        parking_parallel.save_path2npy(path4, 'parking_parallel_path4')

        path = np.concatenate((path1, path2, path3, path4))
        css = PointCloud()
        css.header.frame_id = 'macaron'
        for i in range(len(path)):
            dss = Point32()
            dss.x = path[i][0]
            dss.y = path[i][1]
            dss.z = 0
            css.points.append(dss)
        self.path_pub.publish(css)
        print('경로 생성 완료')
        self.Finding = False
        return path1, path2, path3, path4

class mission_parking_parallel:
    def __init__(self, gp):
        self.step = 0
        self.s = 0
        self.end_s = 0
        self.done = False
        self.Fail = False

        self.starting_time = 0
        self.ending_time = 0

        self.parking_speed = [50, -30, 10, 40]

        self.Generated = False
    
        self.current_path = gp

        self.gp_PT = mission_cruising(gp)
        self.PT = Path_Tracking(self.current_path)
        self.PT_PP = Path_Tracking_PP(self.current_path)
        self.GP = GlobalPath(PATH_ROOT + self.current_path)
        self.PT_cr = mission_cruising(self.current_path)

    def path_generate(self, pose, heading):
        finding = Find_parking_lot()
        while finding.Finding:
            if finding.Found == True:
                # path1, path2, path3, path4 = finding.point2gps(pose, heading)
                finding.point2gps(pose, heading)
                self.path = ['parking_parallel_path1.npy', 
                            'parking_parallel_path2.npy', 
                            'parking_parallel_path3.npy',
                            'parking_parallel_path3.npy']
                self.straight_path = self.path[0]
                self.back_path = self.path[1]
                self.front_path = self.path[2]
                self.out_path = self.path[3]
                self.Generated = True
                break

    def speed_def(self):
        if self.step == 0 or  1:
            speed = self.parking_speed[0]
        elif self.step == 2:
            speed = self.parking_speed[1]
        elif self.step == 3:
            speed = self.parking_speed[2]
        elif self.step == 5:
            speed = self.parking_speed[3]
        else:
            speed = 0
        return speed
    
    def change_path(self):
        if self.step == 1:
            self.current_path = self.straight_path
            self.PT_cr = mission_cruising(self.current_path)
        elif self.step == 2:
            self.current_path = self.back_path
            self.PT_PP = Path_Tracking_PP(self.current_path)
        elif self.step == 3:
            self.current_path = self.front_path
            self.PT_cr = mission_cruising(self.current_path)
        elif self.step == 5:
            self.current_path = self.out_path
            self.PT_cr = mission_cruising(self.current_path)
        self.GP = GlobalPath(PATH_ROOT + self.current_path)

    def calculate_end_s(self):
        if self.step == 1:
            x, y = np.load(file = PATH_ROOT + self.straight_path)[-1]
            self.end_s, _ = self.GP.xy2sl(x, y)
        elif self.step == 2:
            x, y = np.load(file = PATH_ROOT + self.back_path)[-1]
            self.end_s, _ = self.GP.xy2sl(x, y)
        elif self.step == 3:
            x, y = np.load(file = PATH_ROOT + self.front_path)[-1]
            self.end_x, _ = self.GP.xy2sl(x, y)
        elif self.step == 5:
            x, y = np.load(file = PATH_ROOT + self.out_path)[-1]
            self.end_x, _ = self.GP.xy2sl(x, y)
        else:
            pass

    def update(self, pos, heading, obs):
        self.change_path()
        self.s, _ = self.GP.xy2sl(pos[0], pos[1])
        if self.step == 0:
            steer = self.PT_cr.path_tracking(pos, heading)
        elif self.step == 2:
            steer = self.PT_PP.gps_tracking(pos, heading, path_num=-1, parking=2)
        elif self.step == 5:
            steer = self.PT_cr.static_obstacle(pos, heading, obs)
        else:
            self.PT.stanley.k = 3
            steer = self.PT.gps_tracking(pos, heading)
        return steer

    def step_finished(self, pos, heading, obs):
        speed = self.speed_def()
        if self.step == 0:
            steer = self.update(pos, heading, obs)
            print('주차공간 탐색 중...')
            self.path_generate(pos, heading)
            if self.Generated == True:
                self.step += 1
        else:   
            self.calculate_end_s()
            steer = self.update(pos, heading, obs)
            print('현재 step == ', self.step)
            print('현재 경로 == ', self.current_path)
            print('현재 s == ', self.s)
            print('비교 s == ', self.end_s)
            if self.step == 1:
                print('주차 중...')
                if self.s >= self.end_s - 2:
                    self.step += 1
            elif self.step == 2:
                print('후진 중')
                if self.s >= self.end_s - 1.2:
                    self.step += 1
            elif self.step == 3:
                if self.s >= self.end_s:
                    self.time_start = time.time()
                    print('주차완료')
                    self.step += 1
                    return [0, 0]
            elif self.step == 4:
                print('정지 중')
                self.end_time = time.time()
                if self.end_time - self.time_start > 5:
                    self.step += 1
            elif self.step == 5:
                print('출차 중')
                if self.s >= self.end_s - 1.8:
                    self.done = True
                    self.step += 1
        return speed, steer
    
    def run(self, pos, heading, obs):
        if self.step < 6:
            speed, steer = self.step_finished(pos, heading, obs)
        else:
            print('주차미션 완료')
            speed = self.speed_def()
            steer = 0
            self.done = True
        return speed, steer

