#!/usr/bin/env python
# -*-coding:utf-8-*-

# Python packages
import rospy
import sys, os
import time
import numpy as np
from math import *
import pickle
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/path_planning")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/src/sensor")

from global_path import GlobalPath
from pure_pursuit_added_stanley import Path_Tracking_PP_Stanley
from path_planning_tracking_pure_pursuit import Path_Tracking_PP
from sklearn.cluster import DBSCAN
from collections import defaultdict

# K-city
# PARKING_POINT_XY = [[935535.2576625232, 1915869.9745163824],
#                     [935533.0447708366, 1915865.9641394662],
#                     [935530.6232717733, 1915861.3562524994]]

# 만해 test 용 - 2023-09-04-21-13-03.bag
# PARKING_POINT_XY = [[955771.7441545076, 1951257.0029895431],
#                     [955773.6651019057, 1951252.843299283],
#                     [955775.6117199984, 1951248.5281455426]]

# PARKING_POINT_XY = [[931342.0082606408, 1929889.654814554],
#                     [931338.8396739169, 1929885.5534891316],
#                     [931335.9104440911, 1929881.4834838365]]

# FMTC 대회
PARKING_POINT_XY = [[931354.7736472325, 1929837.454814554],
                    [931358.3020861725, 1929842.1341329583],
                    [931361.5542008495, 1929846.7007709795]]


class GenerateParkingPath:
    def __init__(self):
        self.data_length = 30

    # noinspection PyMethodMayBeStatic
    def rotate_point(self, point, rot_angle, center_of_rot):
        translated_point = np.array(point) - np.array(center_of_rot)

        rotation_matrix = np.array([[np.cos(rot_angle), -np.sin(rot_angle)],
                                    [np.sin(rot_angle), np.cos(rot_angle)]])

        rotated_point = np.dot(translated_point, rotation_matrix.T) + np.array(center_of_rot)
        return rotated_point

    def generate_clothoid_curve(self, start_point, end_point, start_angle, end_angle, direction):
        start_angle = radians(start_angle)
        end_angle = radians(end_angle) if direction == 'right' else -radians(end_angle)
        waypoint, waypoints = [], []

        # start_point 와 end_point 간의 직선 거리
        length = sqrt((start_point[0] - end_point[0]) ** 2 + (start_point[1] - end_point[1]) ** 2)

        # 곡률 변화
        curvature_change = (end_angle - start_angle) / length

        # end of Clothoid
        end_x = start_point[0] + length * cos(start_angle + length * curvature_change)
        end_y = start_point[1] + length * sin(start_angle + length * curvature_change)

        # theta1: start_point 와 end_point 간의 기울기, theta2: start_point 와 end of clothoid 간의 기울기
        theta1 = atan2(end_point[1] - start_point[1], end_point[0] - start_point[0])
        theta2 = atan2(abs(start_point[1] - end_y), abs(start_point[0] - end_x))

        for i in range(self.data_length):
            t = i / float(self.data_length - 1)
            theta = start_angle + curvature_change * length * t ** 2  # Clothoid equation
            if end_angle > 0:
                x = start_point[0] + length * t * cos(theta - theta2)
                y = start_point[1] + length * t * sin(theta - theta2)
            else:
                x = start_point[0] + length * t * cos(theta + theta2)
                y = start_point[1] + length * t * sin(theta + theta2)
            waypoint = self.rotate_point([x, y], theta1, start_point)
            waypoints.append(waypoint)
        return waypoints


class ParkingParallel:
    def __init__(self, global_path_npy):
        self.candidate_pub = rospy.Publisher('/CDpath', PointCloud, queue_size=3)
        self.cd_path = None

        # 경로 생성에 필요한 class 선언
        self.GPP = GenerateParkingPath()

        # 전역 경로 획득
        PATH_ROOT = (os.path.dirname(
            os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))) + "/path/npy_file/path/"
        gp_name = PATH_ROOT + global_path_npy

        try:   # ########## 피클 파일 불러오기
            self.GP = pickle.load(open(os.path.dirname(
                os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))+"/path/pkl_file/" +
                                       global_path_npy[0:len(global_path_npy)-4] + '.pkl', 'rb'))
        except ValueError:
            self.GP = GlobalPath(gp_name)

        # parking_point_xy 에 대응 하는 sl 좌표
        self.parking_point_sl = list(
            map(lambda x: list(self.GP.xy2sl(x[0], x[1], mode=1, mission="Parking_Parallel")), PARKING_POINT_XY))
        print('sl : ', self.parking_point_sl)

        # run 함수에서 실질적으로 작동하는 변수
        self.parking_step = 0
        self.search_step = 0
        self.count = 0
        self.current_time = None

        # 주차 공간의 정보
        self.long_line_length = 6.5  # [m]
        self.short_line_length = 3.0  # [m]
        self.perception_range = self.long_line_length / 4
        self.s_offset = 1.0
        self.q_offset = 0.6

        # speed 와 steer 초기화
        self.speed = 0.0
        self.steer = 0.0

        # 후에 생성할 경로들과 Tracking 클래스들 초기화
        self.gp_PT = Path_Tracking_PP_Stanley(global_path_npy)
        self.entry_path_PT = None
        self.entry_path = None
        self.reverse_parking_path = None
        self.reverse_parking_path2 = None
        self.straight_parking_path = None
        self.reverse_parking_path_PT = None
        self.reverse_parking_path_PT2 = None
        self.straight_parking_path_PT = None
        self.generate_path_flag = False

        # DBSCAN
        epsilon = 0.1
        min_points = 3
        self.DBSCAN = DBSCAN(eps=epsilon, min_samples=min_points)

        # 기타 용도
        self.steering_time = 0.0
        self.record_time = 0.0

    # ↓↓ 비주얼 코드 ↓↓
    def visual_candidate_paths(self, candidate_paths):
        self.cd_path = PointCloud()
        for i in range(len(candidate_paths)):
            for j in range(len(candidate_paths[i])):
                p = Point32()
                p.x = candidate_paths[i][j][0]
                p.y = candidate_paths[i][j][1]
                p.z = 0
                self.cd_path.points.append(p)
        self.candidate_pub.publish(self.cd_path)

    # def visual_selected_path(self, selected_path):
    #     self.sel_path = PointCloud()
    #     for i in range(len(selected_path)):
    #         p = Point32()
    #         p.x = selected_path[i][0]
    #         p.y = selected_path[i][1]
    #         p.z = 0
    #         self.sel_path.points.append(p)
    #     self.selected_pub.publish(self.sel_path)
    # ↑↑ 비주얼 코드 ↑↑

    def generate_parking_path(self, landmark_obs_sl, pose, pos_s):
        # 기준이 되는 obstacle 을 기준 으로 parking_point 의 s 값과 q 값을 결정
        s = landmark_obs_sl[0] - self.s_offset - 0.5 * self.long_line_length - 0.3
        q = landmark_obs_sl[1] + self.q_offset - 0.5 * self.short_line_length

        parking_point_xy = self.GP.sl2xy(s + 0.5 * self.perception_range, q - 0.5)  # q 에 - 0.5 해주는 것은 실험값

        # parking_point 의 s 값과 q 값을 이용해 start_point 와 end_point, 그리고 mid_point
        start_point_sl = [s + 3.4 * tan(radians(62)) - self.perception_range + 0.3, q + 3.5]
        end_point_sl = [s - self.perception_range, q]
        mid_point_sl = [0.5 * (start_point_sl[0] + end_point_sl[0]), 0.5 * (start_point_sl[1] + end_point_sl[1])]

        # sl 점들을 이용해 xy 점 획득
        start_point_xy = self.GP.sl2xy(*start_point_sl)
        end_point_xy = self.GP.sl2xy(*end_point_sl)
        mid_point_xy = self.GP.sl2xy(*mid_point_sl)

        # mid-point 부터 start-point 까지 왼쪽 으로 볼록한 clothoid 곡선 생성, 그 후 path2 와 합치기 위한 reverse 와 pop
        path1 = self.GPP.generate_clothoid_curve(mid_point_xy, start_point_xy, 0.0, 28.0, 'left')
        path1.reverse()
        path1.pop()

        # mid-point 부터 end-point 까지 왼쪽 으로 볼록한 clothoid 곡선 생성
        path2 = self.GPP.generate_clothoid_curve(mid_point_xy, end_point_xy, 0.0, 28.0, 'left')

        # ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ 후진 경로 생성 ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓
        selected_path = path1 + path2
        selected_path = list(map(lambda x: list(x), zip(*selected_path)))

        self.reverse_parking_path_PT = Path_Tracking_PP(selected_path, file=1)
        self.reverse_parking_path = list(map(lambda x: list(x), zip(*[self.reverse_parking_path_PT.glob_path.rx,
                                                                      self.reverse_parking_path_PT.glob_path.ry])))
        # ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ 후진 경로 생성 ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑

        # ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ 진입 경로 생성 ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓
        theta = degrees(atan2(abs(start_point_sl[1]), start_point_sl[0] - pos_s))
        self.entry_path = self.GPP.generate_clothoid_curve(self.reverse_parking_path[0], pose, 0.0, theta, 'left')
        self.entry_path.reverse()
        entry_path = list(map(lambda x: list(x), zip(*self.entry_path)))
        self.entry_path_PT = Path_Tracking_PP_Stanley(entry_path, file=1)
        self.entry_path = list(map(lambda x: list(x), zip(*[self.entry_path_PT.glob_path.rx,
                                                            self.entry_path_PT.glob_path.ry])))
        # ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ 진입 경로 생성 ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑

        # ↓ ↓ ↓ ↓ 후진이 끝난 후에 추종할 직진 경로 생성 ↓ ↓ ↓ ↓
        path3 = self.GPP.generate_clothoid_curve(end_point_xy, parking_point_xy, 0.0, 0.0, 'left')
        path3 = list(map(lambda x: list(x), zip(*path3)))
        self.straight_parking_path_PT = Path_Tracking_PP_Stanley(path3, file=1)
        self.straight_parking_path = list(map(lambda x: list(x), zip(*[self.straight_parking_path_PT.glob_path.rx,
                                                                       self.straight_parking_path_PT.glob_path.ry])))
        # ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ 직진 경로 생성 ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑
        print('generate!!!!!!!!!!!!!!!!!')

    def stop(self, stop_t, steering_t, steer):
        # 현재 시간 저장
        if self.current_time is None:
            self.current_time = time.time()

        # 'stop_t' 만큼 멈출 시간 제공 후 'steering_t' 만큼 'steer' 상태가 되도록 steering 할 시간 제공
        if time.time() - self.current_time <= stop_t:
            self.speed = 0
        elif time.time() - self.current_time <= stop_t + steering_t:
            self.speed, self.steer = 0, steer
        else:
            self.parking_step += 1
            self.current_time = None

    # noinspection PyMethodMayBeStatic
    def distance(self, point1, point2):
        return sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def run(self, pose, pos_s, pos_q, heading, obs_xy=None):
        # ↓ ↓ ↓ ↓ ↓ ↓ run 함수의 초기 설정 ↓ ↓ ↓ ↓ ↓ ↓
        if obs_xy is None or obs_xy == []:
            obs_xy = [[0.0, 0.0], [0.01, 0.01], [0.02, 0.02], [-0.01, -0.01], [-0.02, -0.02]]

        obs_xy = np.array(obs_xy, dtype=float)  # obs_xy를 숫자(float) 타입 배열로 변환
        labels = np.array(self.DBSCAN.fit_predict(obs_xy))

        mask = labels != -1
        filtered_point = np.hstack((obs_xy[mask], np.zeros((mask.sum(), 1))))[:, :2]
        filtered_label = labels[mask]

        label_points = defaultdict(list)
        for l, p in zip(filtered_label, filtered_point):
            label_points[l].append(p)
        obs_xy = list(map(lambda x: np.mean(label_points.get(x), axis=0), label_points))

        obs_sl = list(map(lambda x: list(self.GP.xy2sl(x[0], x[1], mode=1, mission="Parking_Parallel")),
                          obs_xy)) if self.parking_step == 0 or self.parking_step == 1 else [[0.0, 0.0]]

        if self.reverse_parking_path is not None:
            self.cd_path = [self.entry_path] + [self.reverse_parking_path] + [self.straight_parking_path]
            self.visual_candidate_paths(self.cd_path)
        # ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑

        # print('parking_step : ', self.parking_step, ' ,  %d번째 공간' % (self.search_step + 1), ' ,   time : ',
        #       round(time.time() - self.record_time, 3))

        if self.parking_step == 0:  # 주차 공간 탐색
            self.speed = 70
            self.steer = self.gp_PT.gps_tracking(pose, heading)

            parking_s, parking_q = self.parking_point_sl[self.search_step]

            if parking_s > pos_s + 5.0:
                for i in range(len(obs_sl)):
                    if parking_s - self.perception_range <= obs_sl[i][0] <= parking_s + self.perception_range \
                            and (parking_q < obs_sl[i][1] < 0):
                        self.count += 1
                # print('count : ', self.count)
                if self.count > 3:
                    self.search_step += 1
                    self.count = 0
            else:
                self.parking_step += 1

        elif self.parking_step == 1:  # 주차 시작 지점에 정지
            self.speed = 40
            if self.entry_path is None:
                self.steer = self.gp_PT.gps_tracking(pose, heading)
            else:
                self.steer = self.entry_path_PT.gps_tracking(pose, heading)

            # 주차 경로 생성 ( 최초 1회 )
            if self.generate_path_flag is False:
                if self.reverse_parking_path is None:
                    if self.distance(PARKING_POINT_XY[self.search_step], pose) < abs(
                            self.parking_point_sl[self.search_step][1]) + 0.6:
                        # 하나의 obstacle 을 landmark 로 지정
                        landmark_obs_sl = obs_sl[min(
                            (sum([abs(e) if e < 0 else e for e in element]), index) for index, element in
                            enumerate(obs_sl)
                            if obs_sl[index][1] > self.parking_point_sl[self.search_step][1])[1]]

                        # landmark 를 기준으로 후진 및 전진 경로 생성
                        self.generate_parking_path(landmark_obs_sl, pose, pos_s)
                else:
                    self.generate_path_flag = True

            # 후진 경로의 시작점과 차량 pos 의 거리 차이가 1.5m 보다 작아질 경우 stop
            if self.reverse_parking_path is not None:
                if self.distance(self.reverse_parking_path[0], pose) < 1.0:
                    self.stop(1, 1, 2000)

        elif self.parking_step == 2:  # 주차 - 후진
            self.speed = -25
            self.steer = 1.7 * self.reverse_parking_path_PT.gps_tracking(pose, heading, parking=1)

            # 주차 경로의 시작점과 차량 pos 의 거리 차이가 1.0m 보다 작아질 경우 stop
            if self.distance(self.reverse_parking_path[-1], pose) < 0.8:
                self.stop(1, 1, 2000)

        elif self.parking_step == 3:  # 주차 - 전진
            self.speed = 20
            r_yaw = self.GP.ryaw[int(pos_s * 10)] + 2 * pi \
                if self.GP.ryaw[int(pos_s * 10)] < 0 else self.GP.ryaw[int(pos_s * 10)]

            self.steer = 1.3 * degrees(heading - r_yaw) * 71 if 1.3 * degrees(heading - r_yaw) * 71 <= -200 else 500

            # 전진 경로의 끝점과 차량 pos 의 거리 차이가 1.0m 보다 작아질 경우 stop
            if self.distance(self.straight_parking_path[-1], pose) < 1.0:
                self.stop(1, 10, 0)

        elif self.parking_step == 4:  # 출차 - 후진
            self.speed = -20
            self.steer = 1000

            # 후진 경로의 끝점과 차량 pos 의 거리 차이가 0.7m 보다 작아질 경우 stop
            if self.distance(self.reverse_parking_path[-1], pose) < 0.5:
                self.stop(1, 1, -2000)
                self.steering_time = time.time()

        # elif self.parking_step == 5:  # 출차 - 후진 경로 추종
        #     # 이전에 추종 했던 후진 경로를 거꾸로 reverse
        #     if self.generate_path_flag is True:
        #         self.reverse_parking_path2 = list(reversed(self.reverse_parking_path))
        #         rospy.sleep(0.1)
        #         reverse_parking_path2 = list(map(lambda x: list(x), zip(*self.reverse_parking_path2)))
        #         self.reverse_parking_path_PT2 = Path_Tracking_PP_Stanley(reverse_parking_path2, file=1)
        #         self.generate_path_flag = False
        #     self.speed = 40
        #     self.steer = self.reverse_parking_path_PT2.gps_tracking(pose, heading)

            # # reverse 된 후진 경로의 끝점(기존 경로의 시작점)과 차량 pos 의 거리 차이가 3m 보다 작아질 경우 크루징으로 전환
            # if self.distance(self.reverse_parking_path2[-1], pose) < 3.0:
            #     self.parking_step += 1

        # elif self.parking_step == 5:  # 크루징 전환
        #     self.speed = 60
        #     self.steer = self.gp_PT.gps_tracking(pose, heading)
        #     if time.time() - self.steering_time < 2.5:
        #         self.steer = -2000
        elif self.parking_step == 5:  # 크루징 전환
            if abs(pos_q) > 2.5:
                self.speed = 40
                self.steer = -2000
            else:
                self.parking_step += 1
                self.speed = 80
                self.steer = self.gp_PT.gps_tracking(pose, heading)
            
            # if time.time() - self.steering_time < 2.5:
            #     self.steer = -2000
        
        elif self.parking_step == 6:  # 크루징 전환
            self.speed = 80
            self.steer = self.gp_PT.gps_tracking(pose, heading)
            
        else:
            pass

        return self.speed, self.steer
