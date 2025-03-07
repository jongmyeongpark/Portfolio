#!/usr/bin/env python3
# -*-coding:utf-8-*-

from math import *
import os
import numpy as np
import rospy
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float64, Float32

"""
mode 0: 차선 변경 없이 한 차선 안에서 좁은 범위의 정적 장애물 회피
mode 1: 차선을 변경 해야 하는 넓은 범위의 정적 장애물 회피 (왼쪽 차선 -> 오른쪽 차선)
mode 2: gps mapping 을 이용한 track 에서 rubber cone 회피
mode 3: 유턴
"""


class DWA:
    def __init__(self, glob_path):
        self.speed_sub = rospy.Subscriber('/speed', Float32, self.speed_callback, queue_size=1)
        self.candidate_pub = rospy.Publisher('/CDpath', PointCloud, queue_size=3)
        self.selected_pub = rospy.Publisher('/SLpath', PointCloud, queue_size=3)

        self.glob_path = glob_path

        self.visual = True
        self.cd_path = None
        self.sel_path = None

        # 로봇의 운동학적 모델 상수 설정
        self.max_speed = 8.0  # 최고 속도 [m/s]
        self.max_steer = np.deg2rad(50)  # 27도 [deg]
        self.max_a = 1.0  # 내가 정하면 됨 [m/s^2]
        self.max_steer_a = np.deg2rad(25.0)  # 내가 정하면 됨 [deg/s^2]

        self.length = 4.635  # 차 길이 [m]
        self.width = 1.892  # 차 폭 [m]
        self.tread = 1.6  # 같은 축의 바퀴 중심 간 거리 [m]
        self.wheel_radius = 0.165  # 바퀴 반지름 [m]
        self.wheel_base = 3.0  # 차축 간 거리 [m]

        self.cur_speed = 0
        self.cur_steer = 0
        self.current_s = 0
        self.current_q = 0

        self.predict_time = 0.1  # 미래의 위치를 위한 예측 시간
        self.search_frame = 5  # 정수로 입력 (range 에 사용)
        self.DWA_search_size = [3, 21]  # Dynamic Window 에서 steer 의 분할 수 (홀수 and 정수)

        self.obstacle_force = 3  # 2m
        self.car_size = 0.8
        self.obs_max_cost = 5.0
        self.obs_param = 0.5

        self.max_index = len(self.glob_path.rx)

        self.obstacle_force_uturn = 3.0
        
        self.detected_points=[]

        self.obj_data_list_x = [0]
        self.obj_data_list_y = [0]

    def erp_callback(self, data):
        self.cur_steer = radians(data.read_steer / 71)  # [rad]
        # self.cur_speed = data.read_speed * (1000 / 3600 / 10)  # [m/s]

    def speed_callback(self, data):
        if data.data == 0:
            self.cur_speed = 7 / 3.6
        else:
            self.cur_speed = data.data / 3.6

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

    def visual_selected_path(self, selected_path):
        self.sel_path = PointCloud()
        for i in range(len(selected_path)):
            p = Point32()
            p.x = selected_path[i][0]
            p.y = selected_path[i][1]
            p.z = 0
            self.sel_path.points.append(p)
        self.selected_pub.publish(self.sel_path)
        
    # ↑↑ 비주얼 코드 ↑↑

    # noinspection PyMethodMayBeStatic
    def convert_coordinate_l2g(self, d_x, d_y, d_theta):  # local -> global 좌표 변환 함수
        d_theta = -pi / 2 + d_theta
        trans_matrix = np.array([[cos(d_theta), -sin(d_theta), 0],  # 변환 행렬
                                 [sin(d_theta), cos(d_theta), 0],
                                 [0, 0, 1]])
        d_theta = pi / 2 + d_theta
        return np.dot(trans_matrix, np.transpose([d_x, d_y, d_theta]))
        # return : local coordinate 에서의 [d_x, d_y, d_theta] 를 global coordinate 에서의 [d_x', d_y', d_theta'] 로 반환

    def generate_predict_point(self, x, y, velocity, steer, heading):  # local 좌표계 에서 예측 점 좌표를 구해서 global 좌표로 return
        # 접선 이동 거리 (= 호의 길이로 사용할 예정)
        tan_dis = velocity * self.predict_time + 1

        # Assuming Bicycle model, (곡률 반경) = (차축 간 거리) / tan(조향각)
        R = self.wheel_base / tan(-steer) if steer != 0.0 else float('inf')

        theta, future_pos = 0.0, []
        for i in range(self.search_frame):
            if R == float('inf'):
                predict_point = [0, tan_dis * (i + 1), theta]
            else:
                theta += tan_dis / R
                predict_point = [R * (1 - cos(theta)), R * sin(theta), theta]  # [d_x, d_y, d_theta] at local coordinate
            pos = np.transpose(self.convert_coordinate_l2g(predict_point[0], predict_point[1], theta + heading))
            future_pos.append([x + pos[0], y + pos[1], pos[2]])
        return future_pos  # return 값은 global coordinate 의 예측 점 x, y 좌표  -> [[x1, y1, theta1], [x2, y2, theta2], .....]

    def calc_dynamic_window(self, velocity, steer=0.0):
        DWA_step_rot = 2 * self.max_steer_a / (self.DWA_search_size[1] - 1)
        DWA_velocity = velocity + self.max_a
        DWA_steer = [steer - self.max_steer_a + DWA_step_rot * i for i in range(self.DWA_search_size[1]) if
                     abs(steer - self.max_steer_a + DWA_step_rot * i) <= self.max_steer]
        dw = [DWA_velocity, DWA_steer]
        return dw

    def cost_function(self, pose, obs_xy):
        cost1, cost2 = 0.0, 0.0
        gp_separation = self.glob_path.xy2sl(pose[-1][0], pose[-1][1])[1]
        cost1 = abs(gp_separation/3) if -3 <= gp_separation <= 3 else 1
        length = len(pose)
        for i in range(length):
            if i == 0: continue
            x, y, _ = pose[i]
            try:
                obs_d = min([sqrt((x - obstacle[0]) ** 2 + (y - obstacle[1]) ** 2) for obstacle in obs_xy])
                # print(i,"==", obs_d)
            except:
                obs_d = self.obstacle_force
            
            if obs_d < self.car_size: return cost1, self.obs_max_cost

            cost2 += (self.obstacle_force - obs_d) / self.obstacle_force * (length-i) if obs_d < self.obstacle_force else 0

        return cost1, min(cost2, self.obs_max_cost * 0.4)
    
    def cost_function_global_path(self, pose, obs_xy, force_rate):
        cost1, cost2 = 0.0, 0.0
        obs_force = self.obstacle_force * force_rate
        length = len(pose)
        for i in range(length):
            x, y, _ = pose[i]
            try:
                obs_d = min([sqrt((x - obstacle[0]) ** 2 + (y - obstacle[1]) ** 2) for obstacle in obs_xy])
                # print(i,"==", obs_d)
            except:
                obs_d = obs_force
            # print(f'{i}: {obs_d}')
            # if obs_d < self.car_size: return cost1, self.obs_max_cost
            
            cost2 += (obs_force - obs_d) / obs_force * (length-i) if obs_d < obs_force else 0

        return cost1, min(cost2, self.obs_max_cost * 0.3)
    
    def cost_function_no_gps(self, pose, obs_xy):
        cost1, cost2 = 0.0, 0.0
        length = len(pose)
        for i in range(length):
            x, y, _ = pose[i]
            try:
                obs_d = min([sqrt((x - obstacle[0]) ** 2 + (y - obstacle[1]) ** 2) for obstacle in obs_xy])
                # print(i,"==", obs_d)
            except:
                obs_d = self.obstacle_force
            
            if obs_d < self.car_size: return cost1, self.obs_max_cost

            cost2 += (self.obstacle_force - obs_d) / self.obstacle_force * (length-i) if obs_d < self.obstacle_force else 0

        return cost1, min(cost2, self.obs_max_cost * 0.4)
        
    def tf2tm_obs(self, x, y, heading, obs_xy):
        obs_glob = []
        for obs in obs_xy:
            obs_x = obs[0] * np.cos(heading) - obs[1] * np.sin(heading) + (x + 1.5)
            obs_y = obs[1] * np.cos(heading) + obs[0] * np.sin(heading) + y
            obs_glob.append([obs_x, obs_y])
        return obs_glob
    
    def delete_global_obs(self):
        self.obj_data_list_x = [0]
        self.obj_data_list_y = [0]
            
    def DWA(self, x, y, heading, obs_xy=None, mode=0, current_index=0, dwa_mode=0):  # (차량의 x, y, heading), (장애물의 x, y)

        if obs_xy is None or obs_xy == [] or obs_xy == [[0.0, 0.0]]:
            obs_xy = [[100.0, 100.0]]

        obs_min_dis = 100
        for obs in obs_xy:
            dis = sqrt((obs[0]+1.5)**2 + obs[1]**2)
            if dis < obs_min_dis:
                obs_min_dis = dis

        obs_xy = self.tf2tm_obs(x, y, heading, obs_xy)

        best_cost = float('inf')
        candidate_paths, selected_path = [], []
        # 전방에 아무것도 없을 때
        if dwa_mode == 0:
            gp_path = []
        
            if current_index+102 < self.max_index:
                for i in range(current_index+1, current_index+101, 20):
                    gp_path.append([self.glob_path.rx[i], self.glob_path.ry[i], self.glob_path.ryaw[i]])
            
            else:
                for i in range(current_index+1, self.max_index, 20):
                    gp_path.append([self.glob_path.rx[i], self.glob_path.ry[i], self.glob_path.ryaw[i]])

            self.current_pose = [x, y, heading]
            self.glob_path.cur_ref_index = current_index

            selected_path = gp_path
            candidate_paths.append(gp_path)

        # cost_filter 사용한 dwa
        elif dwa_mode == 1:

            # obj_data_list_x = self.obj_data_list_x
            # obj_data_list_y = self.obj_data_list_y
            # for glob_obs in obs_xy:
            #     obs_dis = np.sqrt((obj_data_list_x - glob_obs[0]) ** 2 + (obj_data_list_y - glob_obs[1]) ** 2)
            #     distance = np.min(obs_dis)

            #     if len(obj_data_list_x) == 1:
            #         self.obj_data_list_x.append(glob_obs[0])
            #         self.obj_data_list_y.append(glob_obs[1])

            #     elif 0.5 < distance < 10:
            #         self.obj_data_list_x.append(glob_obs[0])
            #         self.obj_data_list_y.append(glob_obs[1])

            # obs_xy = [[obs_x, obs_y] for obs_x, obs_y in zip(self.obj_data_list_x, self.obj_data_list_y)]

            gp_path = []
            self.glob_path.cur_ref_index = current_index

            if current_index+102 < self.max_index:
                for i in range(current_index+1, current_index+51, 10):
                    gp_path.append([self.glob_path.rx[i], self.glob_path.ry[i], self.glob_path.ryaw[i]])
            
            else:
                for i in range(current_index+1, self.max_index, 10):
                    gp_path.append([self.glob_path.rx[i], self.glob_path.ry[i], self.glob_path.ryaw[i]])
            
            path_ = []
            for i in range(0, 5):
                path_x = np.cos(1.61) * i + x
                path_y = np.sin(1.61) * i + y
                path_.append([path_x, path_y, 1.61])
            
            glob_path_distance = sqrt((x-gp_path[0][0])**2 + (y-gp_path[0][1])**2 )
            if glob_path_distance > 0.05 and 2020 < current_index < 2100:
                gp_path = path_

            path_costs, obs_costs = [], []
            
            candidate_paths.append(gp_path)
            if obs_min_dis < 2:
                path_cost, obs_cost = 0, 5
                path_costs.append(path_cost)
                obs_costs.append(obs_cost)
            else:
                path_cost, obs_cost = self.cost_function_global_path(pose=gp_path, obs_xy=obs_xy, force_rate=1.0)
                path_costs.append(path_cost)
                obs_costs.append(obs_cost)

            dw = self.calc_dynamic_window(self.cur_speed)
            velocity = dw[0]

            for steer in dw[1]:

                future_pos = self.generate_predict_point(x, y, velocity, steer, heading)
                
                candidate_paths.append(future_pos)
                path_cost, obs_cost = self.cost_function(pose=future_pos, obs_xy=obs_xy)
                path_costs.append(path_cost)
                obs_costs.append(obs_cost)

            if min(obs_costs) == self.obs_max_cost:
                for idx, path in enumerate(candidate_paths):
                    if obs_costs[idx] < best_cost:
                        best_cost = obs_costs[idx]
                        selected_path = path

            else:
                for idx, path in enumerate(candidate_paths):
                    # print(path_costs[idx], obs_costs[idx])
                    if idx == 0 and obs_costs[idx] >= 1.0: continue
                    if path_costs[idx] == 1.0: continue
                    if obs_costs[idx] == self.obs_max_cost: continue

                    if path_costs[idx] == 0.8: continue

                    cost_t = path_costs[idx] * 0.05 + obs_costs[idx]
                    if cost_t < best_cost:
                        best_cost = cost_t
                        selected_path = path

                if selected_path == []:
                    selected_path = gp_path
                # print(obs_costs)
                # print('==============')

        elif dwa_mode == 2: # slam 첫번째 구간

            gp_path = []
            self.glob_path.cur_ref_index = current_index

            if current_index+102 < self.max_index:
                for i in range(current_index+1, current_index+51, 10):
                    gp_path.append([self.glob_path.rx[i], self.glob_path.ry[i], self.glob_path.ryaw[i]])
            
            else:
                for i in range(current_index+1, self.max_index, 10):
                    gp_path.append([self.glob_path.rx[i], self.glob_path.ry[i], self.glob_path.ryaw[i]])
            
            path_ = []
            for i in range(0, 5):
                path_x = np.cos(0.015) * i + x
                path_y = np.sin(0.015) * i + y
                path_.append([path_x, path_y, 0.015])
            
            glob_path_distance = sqrt((x-gp_path[0][0])**2 + (y-gp_path[0][1])**2 )
            if glob_path_distance > 1 and current_index > 4800:
                gp_path = path_

            path_costs, obs_costs = [], []

            candidate_paths.append(gp_path)
            path_cost, obs_cost = self.cost_function_global_path(pose=gp_path, obs_xy=obs_xy, force_rate=0.75)
            path_costs.append(path_cost)
            obs_costs.append(obs_cost)
            dw = self.calc_dynamic_window(self.cur_speed)
            velocity = dw[0]

            for steer in dw[1]:

                future_pos = self.generate_predict_point(x, y, velocity, steer, heading)
                
                candidate_paths.append(future_pos)
                path_cost, obs_cost = self.cost_function(pose=future_pos, obs_xy=obs_xy)
                path_costs.append(path_cost)
                obs_costs.append(obs_cost)

            if min(obs_costs) == self.obs_max_cost:
                selected_path = gp_path

            else:
                for idx, path in enumerate(candidate_paths):
                    if idx == 0 and obs_costs[idx] >= 1.0: continue
                    if obs_costs[idx] == self.obs_max_cost: continue
                    if path_costs[idx] >= 0.8: continue

                    cost_t = obs_costs[idx]
                    if cost_t < best_cost:
                        best_cost = cost_t
                        selected_path = path
                
                if selected_path == []:
                    selected_path = candidate_paths[8]

            


        elif dwa_mode == 3: # slam 두번째 구간

            gp_path = []
            self.glob_path.cur_ref_index = current_index

            if current_index+102 < self.max_index:
                for i in range(current_index+1, current_index+26, 10):
                    gp_path.append([self.glob_path.rx[i], self.glob_path.ry[i], self.glob_path.ryaw[i]])
            
            else:
                for i in range(current_index+1, self.max_index, 10):
                    gp_path.append([self.glob_path.rx[i], self.glob_path.ry[i], self.glob_path.ryaw[i]])

            path_ = []
            for i in range(0, 5):
                path_x = np.cos(4.7405) * i + x
                path_y = np.sin(4.7405) * i + y
                path_.append([path_x, path_y, 4.7405])
            
            glob_path_distance = sqrt((x-gp_path[0][0])**2 + (y-gp_path[0][1])**2 )
            if glob_path_distance > 0.1 and current_index < 6130:
                gp_path = path_

            path_costs, obs_costs = [], []

            candidate_paths.append(gp_path)
            path_cost, obs_cost = self.cost_function_global_path(pose=gp_path, obs_xy=obs_xy, force_rate=0.25)
            path_costs.append(path_cost)
            obs_costs.append(obs_cost)
            dw = self.calc_dynamic_window(self.cur_speed)
            velocity = dw[0]

            for steer in dw[1]:

                future_pos = self.generate_predict_point(x, y, velocity, steer, heading)
                
                candidate_paths.append(future_pos)
                path_cost, obs_cost = self.cost_function(pose=future_pos, obs_xy=obs_xy)
                path_costs.append(path_cost)
                obs_costs.append(obs_cost)

            if min(obs_costs) == self.obs_max_cost:
                selected_path = gp_path

            else:
                for idx, path in enumerate(candidate_paths):
                    if idx == 0 and obs_costs[idx] >= 1.0: continue
                    if obs_costs[idx] == self.obs_max_cost: continue
                    if path_costs[idx] == 0.5: continue

                    cost_t = obs_costs[idx] + path_costs[idx]
                    if cost_t < best_cost:
                        best_cost = cost_t
                        selected_path = path
                
                if selected_path == []:
                    selected_path = gp_path

            
        if self.visual:
            self.visual_candidate_paths(candidate_paths)
            self.visual_selected_path(selected_path)

        return selected_path
    
    