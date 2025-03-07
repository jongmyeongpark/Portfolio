#!/usr/bin/env python3
# -*-coding:utf-8-*-

from math import *
import os
import numpy as np
import rospy
from geometry_msgs.msg import Point32, Vector3, Point
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float64, Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

"""
mode 0: 차선 변경 없이 한 차선 안에서 좁은 범위의 정적 장애물 회피
mode 1: 차선을 변경 해야 하는 넓은 범위의 정적 장애물 회피 (왼쪽 차선 -> 오른쪽 차선)
mode 2: gps mapping 을 이용한 track 에서 rubber cone 회피
mode 3: 유턴
"""


class DWA_Track:
    def __init__(self):
        self.cdpath_pub = rospy.Publisher('/track_cdpath', MarkerArray, queue_size=1)
        self.slpath_pub = rospy.Publisher('/track_slpath', Marker, queue_size=1)

        self.visual = True
        self.cd_path = None
        self.sel_path = None

        # 로봇의 운동학적 모델 상수 설정
        self.max_speed = 8.0  # 최고 속도 [m/s]
        self.max_steer = np.deg2rad(40.0)  # 27도 [deg]
        self.max_a = 1.0  # 내가 정하면 됨 [m/s^2]
        self.max_steer_a = np.deg2rad(25.0)  # 내가 정하면 됨 [deg/s^2]

        self.length = 2.02  # 차 길이 [m]
        self.width = 1.16  # 차 폭 [m]
        self.tread = 1.03  # 같은 축의 바퀴 중심 간 거리 [m]
        self.wheel_radius = 0.165  # 바퀴 반지름 [m]
        self.wheel_base = 1.04  # 차축 간 거리 [m]

        self.predict_time = 0.5  # 미래의 위치를 위한 예측 시간
        self.search_frame = 5  # 정수로 입력 (range 에 사용)
        self.DWA_search_size = [3, 21]  # Dynamic Window 에서 steer 의 분할 수 (홀수 and 정수)
        self.obstacle_force = 0.4  # 2m
        self.gps2back = 0.5
        self.lidar2gps = 1.04

    # ↓↓ 비주얼 코드 ↓↓
    def visual_candidate_paths(self, candidate_paths):
        rviz_cdpaths = MarkerArray()
        id = 300
        for path in candidate_paths:
            rviz_msg_cdpath=Marker(
                header=Header(frame_id='velodyne', stamp=rospy.get_rostime()),
                ns="track_cdpath",
                id=id,
                type=Marker.LINE_STRIP,
                lifetime=rospy.Duration(0.5),
                action=Marker.ADD,
                scale=Vector3(0.2,0.0,0.0),
                color=ColorRGBA(r=1.0,g=0.5,b=0.5,a=0.8)
            )
            for i in path:
                p = Point()
                p.x = i[0] - self.lidar2gps
                p.y = i[1]
                p.z = 0.0
                rviz_msg_cdpath.points.append(p)
            rviz_cdpaths.markers.append(rviz_msg_cdpath)
            id +=1

        self.cdpath_pub.publish(rviz_cdpaths)

    def visual_selected_path(self, selected_path):
        rviz_msg_slpath=Marker(
            header=Header(frame_id='velodyne', stamp=rospy.get_rostime()),
            ns="sl_path",
            id=400,
            type=Marker.LINE_STRIP,
            lifetime=rospy.Duration(0.5),
            action=Marker.ADD,
            scale=Vector3(0.2,0.0,0.0),
            color=ColorRGBA(r=1.0,g=0,b=0,a=0.8)
        )
        for i in selected_path:
            p = Point()
            p.x = i[0] - self.lidar2gps
            p.y = i[1]
            p.z = 0.1
            rviz_msg_slpath.points.append(p)
        self.slpath_pub.publish(rviz_msg_slpath)
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
        tan_dis = velocity * self.predict_time / 3.6

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

    def obs_side_offset(self, obs_xy):
        if len(obs_xy) == 0:
            return 0
        obs_xy = np.array(obs_xy)
        dx = obs_xy[:, 0]
        dy = obs_xy[:, 1]
        x_plus_indexes = np.where(dx>0)[0]
        if x_plus_indexes.size == 0: 
            return 0
        
        local_x = dx[x_plus_indexes]
        local_y = dy[x_plus_indexes]
        obs_theta_list = []
        for obs_x, obs_y in zip(local_x, local_y):
            theta_rl = atan2(obs_x + self.gps2back, self.wheel_base - obs_y)
            theta_rr = atan2(obs_x + self.gps2back, self.wheel_base + obs_y)
            theta_j = np.pi - (theta_rl + theta_rr)
            obs_theta_list.append(abs(theta_j))
        max_theta_index = np.argmax(obs_theta_list)
        side_offset = local_y[max_theta_index]

        return side_offset

    def cost_function(self, pose, obs_xy, goal):
        cost_obs_dis, cost_obs_theta, dis = 0.0, 0.0, 1.0
        obs_xy = np.array(obs_xy)
        obs_count = 0

        for i in range(len(pose)):
            if i == 0: 
                continue

            x, y, yaw = pose[i]

            dx = obs_xy[:, 0] - x
            dy = obs_xy[:, 1] - y
            local_x = dx * np.cos(yaw) - dy * np.sin(yaw)
            local_y = dx * np.sin(yaw) + dy * np.cos(yaw)
            x_plus_indexes = np.where(local_x>0)[0]

            if x_plus_indexes.size == 0: 
                continue
            
            local_x = local_x[x_plus_indexes]
            local_y = local_y[x_plus_indexes]

            obs_dis = np.sqrt(np.power(local_x, 2) + np.power(local_y, 2))
            min_index = np.argsort(obs_dis)

            if len(min_index) >= 2:
                obs_count = 2

                for obs_d in [obs_dis[min_index[0]], obs_dis[min_index[1]]]:
                    if obs_d < self.obstacle_force:
                        cost_obs_dis = inf
                        break
                    else:
                        cost_obs_dis += obs_d

                for obs_x, obs_y in zip(local_x, local_y):
                    theta_rl = atan2(obs_x + self.gps2back, self.wheel_base - obs_y)
                    theta_rr = atan2(obs_x + self.gps2back, self.wheel_base + obs_y)
                    theta_j = np.pi - (theta_rl + theta_rr)
                    cost_obs_theta += abs(theta_j) * 0.5

            elif len(min_index) == 1:
                obs_count = 1
                if obs_dis[min_index[0]] <= self.obstacle_force:
                    cost_obs_dis = inf
                else:
                    cost_obs_dis += obs_dis[min_index[0]]

                theta_rl = atan2(local_x + self.gps2back, self.wheel_base - local_y)
                theta_rr = atan2(local_x + self.gps2back, self.wheel_base + local_y)
                theta_j = np.pi - (theta_rl + theta_rr)
                cost_obs_theta += abs(theta_j)

            else:
                pass

            if i == len(pose) - 1:
                goal = np.array(goal)
                last_pose = np.array(pose[-1][:2])
                dis = np.linalg.norm(goal - last_pose)
                # dis = np.sqrt(np.power(cx-last_pose[0],2) + np.power(cy-last_pose[1],2))
                # min_dis = np.min(dis)
                # print(min_dis)
        if obs_count != 0:
            return cost_obs_dis / obs_count, cost_obs_theta / obs_count, dis
        else:
            return 100, 100, dis
    
            
    def DWA(self, x, y, heading, obs_xy, target_speed, cx=[0, 1], cy=[0,0], ds=0.1):  # (차량의 x, y, heading), (장애물의 x, y)
        tt = rospy.Time.now().to_sec()
        side_offset = 0
        candidate_paths, selected_path = [], []
            
        cost_obs_dis_array = []
        cost_obs_theta_array = []
        goal_dis_array = []
        self.current_pose = [x, y, heading]

        dw = self.calc_dynamic_window(target_speed)
        velocity = dw[0]
        # print(velocity)

        move_dis =  velocity * self.predict_time / 3.6 * self.search_frame
        try:
            goal = [cx[int(move_dis/ds)], cy[int(move_dis/ds)]]
        except:
            goal = [cx[-1], cy[-1]]

        for steer in dw[1]:
            future_pos = self.generate_predict_point(0, 0, velocity, steer, 0)
            candidate_paths.append(future_pos)
            cost_obs_dis, cost_obs_theta, goal_dis = self.cost_function(future_pos, obs_xy=obs_xy, goal=goal)
            cost_obs_dis_array.append(cost_obs_dis)
            cost_obs_theta_array.append(cost_obs_theta)
            goal_dis_array.append(goal_dis)

        best_cost = float('inf')
        best_cost_index = -1
        all_dis_cost_inf = np.all(np.isinf(cost_obs_dis_array))
        if not all_dis_cost_inf:
            for i in range(self.DWA_search_size[1]):
                if cost_obs_dis_array[i] == float('inf') or goal_dis_array[i] > 0.3:
                    continue
                
                cost = cost_obs_theta_array[i] + cost_obs_dis_array[i]
                if cost < best_cost and goal_dis_array[i] <= 0.3:
                    best_cost = cost
                    best_cost_index = i

            if best_cost_index == -1:
                min_dis = 10
                for i in range(self.DWA_search_size[1]):
                    if goal_dis_array[i] < min_dis:
                        min_dis = goal_dis_array[i]
                        best_cost_index = i
                # print(best_cost_index)
                # print(min_dis)
            
        else:
            min_dis = 10
            for i in range(self.DWA_search_size[1]):
                if goal_dis_array[i] < min_dis:
                    min_dis = goal_dis_array[i]
                    best_cost_index = i

        # side_offset = self.obs_side_offset(obs_xy)
        # print(side_offset)
        # print(goal_dis_array[best_cost_index])
        # if side_offset < -1.5 and best_cost_index > 2:
        #     best_cost_index -= 1
        # if side_offset < -2.0 and best_cost_index > 3:
        #     best_cost_index -= 2
        # elif side_offset > 1.5 and best_cost_index < 19:
        #     best_cost_index += 1
        # elif side_offset > 2.0 and best_cost_index < 18:
        #     best_cost_index += 1

        # best_cost_index = min(best_cost_index, self.DWA_search_size[0]-1)
            # print(min_dis)
        selected_path = candidate_paths[best_cost_index]
        selected_path = [[0,0,0]] + selected_path[:4]
        # os.system('clear')
        # print(best_cost_index)
        # print(goal_dis_array)
        # print(cost_obs_dis_array)
        # print(cost_obs_theta_array)

        if self.visual:

            self.visual_candidate_paths(candidate_paths)
            self.visual_selected_path(selected_path)

        return selected_path