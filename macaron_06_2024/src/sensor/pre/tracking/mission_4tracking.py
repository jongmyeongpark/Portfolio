#!/usr/bin/env python3
# -- coding: utf-8 --

from copy import deepcopy
import os
import sys
import rospy
import math
import time
import numpy as np
from std_msgs.msg import Int8
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from scipy.spatial import KDTree
from macaron_06.msg import lidar_info
from macaron_06.msg import erp_read
from scipy.interpolate import CubicSpline
class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class MissionTracking:
    '''
    가장 가까운 점 2개 잡는 코드
    '''
    def __init__(self):
        rospy.init_node('mission_tracking_node')
        # 기본 파라미터 설정
        self.raw = []  # 원시 포인트 클라우드 데이터를 저장할 리스트
        self.offset = 3  # 도로 폭 (미터)
        self.center_offset = 1.25 # 중심 이격 거리
        self.noise_offset = 1.8  # 노이즈 허용 거리
        self.look_distance = 1.5  # 추종 거리
        self.coefficient = 1.0  # 보정치
        self.min_dis = 1  # 최소 거리
        self.max_dis = 3  # 최대 거리
        self.cone_flag = False
        self.point_flag = False
        self.cones = []  # 콘 포인트 리스트
        self.blue_cones = []
        self.yellow_cones = []
        self.cone_centroids = []
        self.steer = 0
        self.left_line = []
        self.right_line = []
        self.gradient  = 0
        
        self.consecutive_turns = 5
        self.left_turn_count = 0
        self.right_turn_count = 0
        
        # ROS Publisher 및 Subscriber 설정
        self.track_line_pub = rospy.Publisher('/track_line', PointCloud, queue_size=1)
        self.direction_pub = rospy.Publisher('/track_direction', Int8, queue_size=1)

        self.marker_array_pub = rospy.Publisher('/marker_array', MarkerArray, queue_size=1)
        self.lidar_sub = rospy.Subscriber('/cluster_cone', lidar_info, self.cone_callback) # 임시
        rospy.Subscriber('/point_assigned', PointCloud2, self.point_callback)
        
        # self.pose_sub = rospy.Subscriber('/current_pose', Point, self.pose_callback, queue_size=1)


    def remove_nan_and_duplicates(self, center_points):
        # NaN 값을 제거하고, 소수점 둘째 자리에서 반올림
        cleaned_points = [point for point in center_points if not np.isnan(point[0])]
        rounded_points = [(round(point[0], 5), point[1]) for point in cleaned_points]
        
        # 중복 제거 (첫 번째 요소를 반올림한 값 기준)
        seen = set()
        unique_points = []
        for point in rounded_points:
            if point[0] not in seen:
                seen.add(point[0])
                unique_points.append(point)

        return unique_points
    
    
    # def pose_callback(self, data):
    #     self.pose = [data.x, data.y]
    #     self.heading = data.z

    def cone_callback(self, msg):
        pcd = []
        for point in point_cloud2.read_points(msg.data, field_names=("x", "y", "z", "intensity")):
            pcd.append(point)
        pcd = np.array(pcd)[:, :3]
        
        if pcd.shape[0] == 0:
            return
        
        self.stamp = msg.header.stamp
        cluster_indices = list(msg.clusters)
        cone_indices = list(msg.cones)

        if len(cluster_indices) == 0 or len(cone_indices) == 0:
            return

        clusters = []
        count = 0
        
        for indice_size in msg.clusterSize:
            indice = cluster_indices[count : count+indice_size]
            count += indice_size

            clusters.append(pcd[indice, :])

        cones = [clusters[i] for i in cone_indices]
        self.lidar_data = np.vstack(cones)
        

        #cone 중심점 
        cone_centroids = []
        for cone in cones:
            cone_centroids.append(np.mean(cone, axis=0))

        self.cone_centroids = np.vstack(cone_centroids)
        self.cone_flag = True
        



    # def calculate_curvature(self, points):
    #     """
    #     주어진 2D 좌표 목록에서 곡률을 계산하는 함수.

    #     Args:
    #         points (list): [(x1, y1), (x2, y2), ...] 형식의 2D 점들 리스트.

    #     Returns:
    #         curvatures (list): 각 구간에 대한 곡률 값 리스트.
    #     """
    #     curvatures = []
        
    #     for i in range(1, len(points) - 1):
    #         x1, y1 = points[i - 1]
    #         x2, y2 = points[i]
    #         x3, y3 = points[i + 1]

    #         # 1차 도함수 (미분)
    #         dx1 = x2 - x1
    #         dy1 = y2 - y1
    #         dx2 = x3 - x2
    #         dy2 = y3 - y2

    #         # 2차 도함수 (이차 미분)
    #         ddx = dx2 - dx1
    #         ddy = dy2 - dy1

    #         # 곡률 공식 적용
    #         numerator = dx1 * ddy - dy1 * ddx
    #         denominator = (dx1**2 + dy1**2)**(3/2)
            
    #         # 곡률 계산 (0으로 나누는 오류 방지)
    #         curvature = numerator / denominator if denominator != 0 else 0
    #         curvatures.append(curvature)
        
    #     return curvatures

    # def determine_turn_direction(self, curvatures):
    #     """
    #     곡률 리스트를 기반으로 왼쪽 또는 오른쪽 방향을 결정하는 함수.

    #     Args:
    #         curvatures (list): 곡률 값 리스트.

    #     Returns:
    #         str: "left" 또는 "right"로 방향을 반환.
    #     """
    #     mean_curvature = np.mean(curvatures)
        
    #     if mean_curvature > 0:
    #         return "left"  # 양의 곡률일 경우 왼쪽 회전
    #     elif mean_curvature < 0:
    #         return "right"  # 음의 곡률일 경우 오른쪽 회전
    #     else:
    #         return "straight"  # 곡률이 거의 없으면 직선




    def point_callback(self, input_rosmsg):
        start_time = time.time()

        self.blue_cones = []
        self.yellow_cones = []
        self.raw = []

        try:
            cone_points = []
            for point in point_cloud2.read_points(input_rosmsg, field_names=("x", "y", "z", "intensity")):
                cone_points.append(Point(point[0], point[1], point[2]))

            if not cone_points:
                rospy.logwarn("No cone points received")
                return
            
            self.point_flag = True
            filtered_points = self.filter_points([(p.x, p.y, p.z) for p in cone_points])

            for point in filtered_points:
                x, y, z = point
                self.raw.append([x, y, z])
                dis = np.sqrt(x**2 + y**2)

                if z == 0:
                    self.blue_cones.append([x, y])
                elif z == 1:
                    self.yellow_cones.append([x, y])
            
        except Exception as e:
            rospy.logwarn(f"Error in point_callback: {e}")

    def filter_points(self, points, min_distance=0.5, max_distance=7.0):
        """포인트 클라우드 필터링"""
        return [point for point in points if min_distance < np.sqrt(point[0]**2 + point[1]**2) < max_distance]

    def filter_points_between_points(self, line, x1, x2):
        """주어진 x 좌표 구간 내에 있는 점들을 필터링하는 함수"""
        # return [p for p in line if (min(x1[0], x2[0]) < p[0] < max(x1[0], x2[0])) and (min(x1[1], x2[1]) < p[1] < max(x1[1], x2[1]))]
        return [p for p in line if (min(x1[0], x2[0]) < p[0] < max(x1[0], x2[0]))]

    
    def filter_points_by_distance(self, current_point, points, min_distance=0.5, max_distance=1.5):
        """
        현재 점에서 특정 거리 범위 내에 있는 후보 점들을 필터링하는 함수

        Parameters:
        current_point (tuple): 기준이 되는 현재 점 (x, y).
        points (list): 후보 점들의 리스트 [(x1, y1), (x2, y2), ...].
        min_distance (float): 거리의 하한값.
        max_distance (float): 거리의 상한값.

        Returns:
        list: 거리 범위 내의 후보 점들의 리스트.
        """
        filtered_points = []

        for point in points:
            # 두 점 사이의 유클리드 거리 계산
            distance = math.sqrt((current_point[0] - point[0]) ** 2 + (current_point[1] - point[1]) ** 2)
            
            # 거리 범위 내의 점들만 추가
            if min_distance <= distance <= max_distance:
                filtered_points.append(point)
        
        return filtered_points
    
    
    def add_interpolated_points(self, points, num_new_points=1):
        """각 점 쌍 사이에 새로운 점들을 추가합니다."""
        new_points = []
        if len(points) < 2:
            return points

        for i in range(len(points) - 1):
            p1 = np.array(points[i])
            p2 = np.array(points[i + 1])
            distance = np.linalg.norm(p2 - p1)
            # 일정 간격 이상인 경우에만 보간
            if distance > 0.5:
                num_interpolated = int(distance / 0.5)
                for j in range(1, num_interpolated + 1):
                    t = j / float(num_interpolated + 1)
                    new_points.append(list(p1 + t * (p2 - p1)))
            new_points.append(points[i + 1])

        return new_points


    def replace_points_with_polyfit(self, points):
        """
        기존 점들을 2차 곡선 피팅으로 계산된 점들로 대체합니다.
        기존 좌표와의 오차가 10cm 이내인 경우에만 대체합니다.

        Args:
            points: 2D 리스트로 된 점들 (x, y).
        
        Returns:
            2차 곡선 피팅을 통해 계산된 새로운 점들로 대체된 리스트.
        """
        if len(points) < 3:  # 최소 3개의 점이 있어야 2차 곡선 피팅 가능
            return points

        # 전체 점에 대해 x, y 값을 추출
        x_vals = np.array([p[0] for p in points])
        y_vals = np.array([p[1] for p in points])

        # 2차 곡선 피팅
        coefficients = np.polyfit(x_vals, y_vals, 2)
        polynomial = np.poly1d(coefficients)

        # 새로운 y 좌표 계산 및 기존 좌표와의 오차 계산
        new_points = []
        for i, x in enumerate(x_vals):
            new_y = polynomial(x)
            original_point = points[i]
            distance = np.sqrt((x - original_point[0])**2 + (new_y - original_point[1])**2)
            
            # 오차가 10cm 이내인 경우에만 대체
            if distance <= 0.5:  # 10cm = 0.1m
                new_points.append([x, new_y])
            else:
                new_points.append(original_point)

        return new_points



    def sort_by_closest_points(self, line, reference_point):
        sorted_line = []
        current_point = np.array(reference_point)
        points_candids = deepcopy(line)
        while points_candids:
            # 현재 포인트에서 거리가 3 이하인 점들만 필터링
            close_points = [cone for cone in points_candids if np.linalg.norm(np.array(cone) - current_point) < 6]

            if not close_points:
                # 거리가 3 이하인 점이 없으면 종료
                break

            # 그 중에서 가장 가까운 점을 찾음
            closest_point = min(close_points, key=lambda cone: np.linalg.norm(np.array(cone) - current_point))

            # 그 점을 sorted_line에 추가하고 line에서 제거
            sorted_line.append(closest_point)
            points_candids.remove(closest_point)

            # reference_point를 가장 가까운 점으로 업데이트
            current_point = np.array(closest_point)
        # print(f"sorted_line : {sorted_line}")
        return sorted_line


    def process_line(self, line, centroids, look_distance):
        """
        주어진 라인 데이터를 바탕으로 거리가 특정 기준보다 큰 경우, 새로운 포인트를 추가하는 함수.
        left_line 또는 right_line을 기반으로 보간 작업을 수행.

        Args:
            line (list): 기준 라인 데이터 리스트.
            centroids (list): 모든 콘의 중심 좌표 리스트.
            look_distance (float): 두 점 사이의 거리가 이 값보다 크면 중간 점을 추가.
        Returns:
            list: 업데이트된 라인 리스트.
        """
        updated_line = line[:]

        for i in range(1, len(line)):
            p1 = line[i - 1]
            p2 = line[i]
            distance = self.euclidean_distance(p1, p2)

            if distance > look_distance:
                points_in_range = self.filter_points_between_points(centroids, p1, p2)

                if len(points_in_range) > 0:
                    current_point = p1
                    while True:
                        # 현재 포인트보다 x값이 큰 포인트만 필터링
                        points_in_range = [point for point in points_in_range if point[0] > current_point[0]]
                        points_candid = self.filter_points_by_distance(current_point, points_in_range, min_distance=0.1, max_distance=2)

                        if len(points_candid) == 0:
                            break

                        # 가장 가까운 포인트 찾기
                        distances = np.array([self.euclidean_distance(point, current_point) for point in points_candid])
                        min_idx = np.argmin(distances)
                        closest_point = points_candid[min_idx][:2]

                        # 'closest_point'가 리스트 또는 튜플인지 확인
                        if isinstance(closest_point, (list, tuple)):
                            updated_line.append(closest_point[:2])  # 새로운 포인트 추가
                            current_point = closest_point
                        else:
                            print(f"오류 발생: closest_point가 잘못된 값입니다. closest_point: {closest_point}")
                            break

                        if self.euclidean_distance(current_point, p2) < 0.5:
                            break
             
        # # updated_line을 [0, 0]과 가까운 순서대로 정렬
        # updated_line.sort(key=lambda x: x[0]) 
        # # updated_line에서 [0, 0]과 가장 가까운 점 찾기
        # start_point = updated_line[0]
        # current_point = start_point

        # cluster_points = [point for point in centroids if -1 <= point[0] <= current_point[0]]
        # points_candid = self.filter_points_by_distance(current_point, cluster_points , min_distance=0.5, max_distance=2)
  
        # if len(points_candid) == 0:
        #     return updated_line
      

        # if len(points_candid) > 0:
        #     # 가장 가까운 군집화된 포인트 찾기
        #     distances = np.array([self.euclidean_distance(point, current_point) for point in points_candid])
        #     min_idx = np.argmin(distances)
        #     closest_cluster_point = points_candid[min_idx][:2]
        #     print(closest_cluster_point)
        #     updated_line.append(closest_cluster_point)



        return updated_line



    def process_centerline(self):

        left_cones = deepcopy(self.yellow_cones)
        right_cones = deepcopy(self.blue_cones)
        
        # left_cones.append([0, +1.3])  # 왼쪽 콘에 기준 점 추가
        # right_cones.append([0, -1.3])  # 오른쪽 콘에 기준 점 추가

        # 기준 점
        reference_point = np.array([0, 0])
        
        left_cones= self.remove_nan_and_duplicates(left_cones)
        right_cones= self.remove_nan_and_duplicates(right_cones)
        

        
        #print(f'left_line:{(left_line)} right_line:{(right_line)}')

        if isinstance(self.cone_centroids, np.ndarray):
            self.cone_centroids = deepcopy(self.cone_centroids.tolist())

        # try:
            # if self.cone_flag == True and self.point_flag == True:
            left_cones = self.sort_by_closest_points(left_cones, [0, 1.5])
            right_cones = self.sort_by_closest_points(right_cones, [0, -1.5])
            
            left_line = self.process_line(left_cones, self.cone_centroids, 1.5)
            right_line= self.process_line(right_cones, self.cone_centroids, 1.5)
            
            left_line = left_cones
            right_line= right_cones

            left_line= self.remove_nan_and_duplicates(left_line)
            right_line= self.remove_nan_and_duplicates(right_line)  
            
            left_line = self.sort_by_closest_points(left_line, [0, 1.5])
            right_line = self.sort_by_closest_points(right_line, [0, -1.5])
            if len(left_line)>5:
                left_line = left_line[:5]
            if len(right_line)>5:
                right_line = right_line[:5]                 
            final_left_line = left_line.copy()
            final_right_line = right_line.copy()
            # left_line = self.replace_points_with_polyfit(left_line)
            # right_line = self.replace_points_with_polyfit(right_line)

            
            # left_line.sort(key=lambda x: x[0])
            # right_line.sort(key=lambda x: x[0])  
            
            center_points = self.calculate_centerline(left_cones, right_cones, left_line, right_line)
            center_points = self.adjust_center_line(center_points, left_line, right_line)

            if len(center_points)> 0:
                self.publish_center_points(center_points)
            
            self.visualize_lines(final_left_line, final_right_line, center_points)
                
                # self.cone_flag = False 
                # self.point_flag = False
                # if not center_points:
                #     return
                # if not left_line and not right_line:
                  
                #     #print("Insufficient cones to generate lines")
                #     return
                # else:
                #     # 시각화 및 PointCloud 발행
                #     self.visualize_and_publish(center_points, self.left_line, self.right_line)

        # except Exception as e:
        #     print(f"An error occurred: {e}")
    




        # # if len(center_points) > 2:
        # #     center_points.sort(key=lambda x: x[0])
        # #     center_points = self.interpolate_points(center_points, interval=0.2)
        # #     center_points.sort(key=lambda x: x[0])
        # if not center_points:
        #     #rospy.logwarn("Failed to calculate centerline")
        #     return

        # # 시각화 및 PointCloud 발행
        # self.visualize_and_publish(center_points, self.left_line, self.right_line)


    def euclidean_distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    
    
    def interpolate_points(self, line, interval=1):
        """포인트 간격을 일정하게 보간"""
        interpolated_line = []
        for i in range(len(line) - 1):
            start_point = np.array(line[i])
            end_point = np.array(line[i + 1])
            dist = np.linalg.norm(end_point - start_point)
            num_new_points = int(dist // interval)  # 일정 간격으로 새로운 점 계산
            interpolated_line.append(start_point.tolist())  # 기존 시작점 추가
            for j in range(1, num_new_points + 1):
                new_point = start_point + (end_point - start_point) * (j * interval / dist)
                interpolated_line.append(new_point.tolist())
        interpolated_line.append(line[-1])  # 마지막 점 추가
        return interpolated_line

    def translate_points(self, points, translation_vector, distance):
        """
        주어진 포인트들을 일정 거리만큼 평행이동시킵니다.

        :param points: 이동할 포인트들의 리스트 (리스트의 리스트 형태로, [[x1, y1], [x2, y2], ...])
        :param translation_vector: 이동 방향을 나타내는 벡터 (리스트 형태로, [dx, dy])
        :param distance: 이동할 거리
        :return: 이동된 포인트들의 리스트
        """
        points = np.array(points)
        translation_vector = np.array(translation_vector)
        translation_vector_norm = np.linalg.norm(translation_vector)
        
        if translation_vector_norm == 0:
            raise ValueError("Translation vector cannot be zero.")

        # 이동 방향 벡터를 정규화
        unit_translation_vector = translation_vector / translation_vector_norm
        
        # 거리만큼 이동
        translation = unit_translation_vector * distance

        # 각 포인트에 대해 이동 적용
        translated_points = points + translation

        return translated_points.tolist()


        
    def calculate_turn_direction(self, center_line, left_threshold=5, right_threshold=-5):
        """
        center_line의 y값 합을 기반으로 좌회전, 우회전, 직진 여부를 판단하는 함수.
        
        :param center_line: (list) center_line의 좌표 리스트 [(x1, y1), (x2, y2), ...]
        
        :return: (int) -1 좌회전, 1 우회전, 0 직진
        """
        if len(center_line) == 0:
            return 0  # center_line이 없을 경우 직진으로 판단

        # center_line의 모든 y값의 합 계산
        y_sum = sum([y for _, y in center_line])
        # print(f"Y값 합계: {y_sum}")

        # y값의 합에 따라 좌회전, 우회전, 직진 여부 결정
        if y_sum >= left_threshold:
            self.left_turn_count += 1
            self.right_turn_count = 0  # 우회전 카운터 초기화
        elif y_sum <= right_threshold:
            self.right_turn_count += 1
            self.left_turn_count = 0  # 좌회전 카운터 초기화
        else:
            self.left_turn_count = 0
            self.right_turn_count = 0

        # 연속으로 좌회전, 우회전 발생 여부 확인
        if self.left_turn_count >= self.consecutive_turns:
            # print(f"연속 좌회전: {self.left_turn_count}회")
            return -1  # 좌회전으로 판단
        elif self.right_turn_count >= self.consecutive_turns:
            # print(f"연속 우회전: {self.right_turn_count}회")
            return 1  # 우회전으로 판단
        else:
            return 0  # 직진

    def select_uniform_points(self, points, num_points=5):
        """포인트 클라우드에서 균일한 점을 선택"""
        try:
            if len(points) > num_points:
                indices = np.linspace(0, len(points) - 1, num_points, dtype=int)
                return [points[i] for i in indices]
            else:
                return points
        except Exception as e:
            rospy.logwarn(f"Error in selecting uniform points: {e}")
            return points


    def calculate_centerline(self, left_cones, right_cones, left_line, right_line):
        """왼쪽과 오른쪽 라인에서 중심선을 계산하고, 점 사이의 거리가 self.look_distance 이상인 경우 보간"""
        center_points = []
        direction = 0
        direction_points = []
        # left_line.append([0, +1])  # 왼쪽 콘에 기준 점 추가
        # right_line.append([0, -1])  # 오른쪽 콘에 기준 점 추가
   
        
        self.left_line = deepcopy(left_line)
        self.right_line = deepcopy(right_line)
        
        

        def calculate_shortest_distances(left_line, right_line):
            # Find the shortest distances between each point in the smaller set and the larger set
            distances = []
            if len(left_line) > 0 and len(right_line)>0:
                left_line_d = np.array(left_line).reshape(-1, 2)
                right_line_d = np.array(right_line).reshape(-1, 2)
                
                for l_point in left_line_d:
                    min_distance = np.min(np.linalg.norm(right_line_d - l_point, axis=1))
                    distances.append(min_distance)
                return np.mean(distances) + 0.2 if distances else 0
            else:
                return 3.0

        # Calculate shortest distances and adjust s_offset and l_offset
        if len(left_line) > len(right_line):
            avg_distance = calculate_shortest_distances(right_line, left_line)
        else:
            avg_distance = calculate_shortest_distances(left_line, right_line)
        
        s_offset = avg_distance / 2 - 0.4
        l_offset = avg_distance / 2 + 0.3    
        
        # s_offset = avg_distance / 2
        # l_offset = avg_distance / 2     
        print(f"s_offset: {s_offset}, l_offset:{l_offset}")        
        # s_offset = 0.9
        # l_offset = 1.6
        straight_offset = avg_distance / 2 
        direction = int(direction)
        left_curve_points   = []
        right_curve_points = []
        
        # left_cones와 right_cones가 리스트인 경우 Numpy 배열로 변환
        left_points = np.array(left_line)
        right_points = np.array(right_line)
        left_points = np.array(left_line).reshape(-1, 2)
        right_points = np.array(right_line).reshape(-1, 2)
        
        # 왼쪽 좌표 필터링
        left_curve_points = right_points[(right_points[:, 1] > 0) & (right_points[:, 0] < 10)]
        # 오른쪽 좌표 필터링
        right_curve_points = left_points[(left_points[:, 1] < 0) & (left_points[:, 0] < 10)]
        
        if len(left_curve_points) > 0:
            direction = -1
        
        elif len(right_curve_points) > 0:
            direction = 1
        else:
            direction = 0
        print(f"direction : {direction}")
        self.direction_pub.publish(direction)
        self.left_line = left_line
        self.right_line = right_line
        try:
            if len(left_line) >= 3 and len(right_line) >= 3:
                if direction == -1:
                    center_points = self.turn_process(direction, left_line, right_line,s_offset, l_offset) 
                    if len(center_points) > 2:
                        return center_points
                    else:
                        return self.extrapolate_line(self.left_line, offset= s_offset , direction='left') 
                elif direction == 1:
                    
                    center_points = self.turn_process(direction, left_line, right_line,s_offset, l_offset)  
                    
                    if len(center_points) > 0:
                        return center_points
                    else:
                        return self.extrapolate_line(self.right_line, offset= s_offset , direction='right')  
                
                else:

                        # return self.extrapolate_line(left_line, offset=straight_offset,  direction='left')
                    # else: 
                    if len(left_line) > len(right_line):
                        return self.extrapolate_line(self.left_line, offset=straight_offset, direction='left')
                    else:
                        return self.extrapolate_line(self.right_line, offset=straight_offset, direction='right')


            elif len(right_line) > 3:

                if direction == -1:
                    center_points = self.turn_process(direction, left_line, right_line,s_offset, l_offset)  
                    if len(center_points) > 0:
                        return center_points
                    else:
                        center_points = self.extrapolate_line(self.right_line, offset=l_offset, direction='right') 
                        center_points  = self.remove_points_in_circles(self.right_line, center_points, radius=0.3)
                        return center_points
                
                elif direction == 1:
                    center_points = self.extrapolate_line(self.right_line, offset=s_offset, direction='right')
                    center_points  = self.remove_points_in_circles(self.right_line, center_points, radius=0.3)
                    return center_points
                
                else:
                    return self.extrapolate_line(self.right_line, offset=straight_offset, direction='right')
                    
                          
            elif len(left_line) > 3:
                
                if direction == -1:
                    return self.extrapolate_line(self.left_line, offset=s_offset, direction='left')  
                elif direction == 1:
                    center_points = self.turn_process(direction, left_line, right_line,s_offset, l_offset)  
                    if len(center_points) > 0:
                        return center_points
                    else:
                        print('left!!!!!!!!!!!!!!!!!!!')
                        
                        #print(left_line)
                        center_points = self.extrapolate_line(self.left_line, offset=l_offset, direction='left') 
                        center_points  = self.remove_points_in_circles(self.left_line, center_points, radius=0.3)
                        return center_points
                else:
                    return self.extrapolate_line(self.left_line, offset=straight_offset, direction='left') 
             
                
            # 왼쪽 또는 오른쪽 라인 중 어느 한쪽만 많이 있는 경우 처리
            elif len(left_line) > 1 or len(right_line) > 1:
                # if direction == -1:
                #     return self.extrapolate_line(right_line, offset=right_offset, direction='right')  
                # elif direction == 1:
                #     return self.extrapolate_line(left_line, offset=left_offset, direction='left') 
                # else:
                if len(left_line) > len(right_line):
                    return self.extrapolate_line(left_line, offset=straight_offset, direction='left')
                else: 
                    return self.extrapolate_line(right_line, offset=straight_offset, direction='right')

        except Exception as e:
            rospy.logwarn(f"Error in calculating centerline: {e}")
            return []

        return center_points



    def is_point_inside_circle(self, point, cone_center, radius):
        
        """ 점이 콘의 반경 내에 있는지 확인 """
        distance = np.linalg.norm(np.array(point) - np.array(cone_center))
        return distance <= radius

    def remove_points_in_circles(self, cones, center_points, radius):
        remaining_points = []
        
        for point in center_points:
            # center_point가 어떤 콘의 반경 내에 있는지 확인
            inside_any_circle = False
            for cone in cones:
                if self.is_point_inside_circle(point, cone, radius):
                    inside_any_circle = True
                    break
            
            # 반경 내에 없는 점들만 남기기
            if not inside_any_circle:
                remaining_points.append(point)
        
        return remaining_points

    def turn_process(self, turn_direction, left_cones, right_cones, s_offset, l_offset):
        # s_offset = 0.9
        # l_offset = 1.6
        straight_offset = (s_offset + l_offset) / 2
        
        # left_cones와 right_cones가 리스트인 경우 Numpy 배열로 변환
        # left_cones = np.array(left_cones)
        # right_cones = np.array(right_cones)
        
        left_points = self.sort_by_closest_points(left_cones, [0, 1.5])
        right_points = self.sort_by_closest_points(right_cones, [0, -1.5])
 
        left_points = np.array(left_points)
        right_points = np.array(right_points)       
        left_curve_points   = []
        right_curve_points = []
        straight_points = []
        straight_final_points = []
        
        if turn_direction == -1:
            # 왼쪽 좌표 필터링
            left_curve_points = right_points[right_points[:, 1] > 0]
            straight_points = right_points[right_points[:, 1] < 0]
            # print(straight_points)
            
            if len(left_curve_points ) > 1 and len(straight_points) > 1: 
               
                # left_curve_points의  최소 x 좌표보다 작은 straight_points 선택
                straight_final_points = straight_points[straight_points[:, 0] < np.min(left_curve_points[:, 0] - 0.1)]
                print(f"straight:{len(straight_final_points)}, left: {len(left_curve_points)}")
                if len(straight_final_points) > 1 and len(left_curve_points ) > 1:
                    # extrapolate_line 호출
                    # curve_point_l = [point.tolist() for point in [straight_final_points[-1] , left_curve_points[0]]]

                    center_candid_s = self.extrapolate_line(straight_final_points , offset=s_offset, direction='right')
                    center_candid_l = self.extrapolate_line(left_curve_points , offset=l_offset, direction='right')
                # last_point_l = center_candid_l[-1]
                # first_point_s = center_candid_s[0]
                # center_candid_interporate = self.interpolate_points([last_point_l, first_point_s], interval=4)
                    center_points = center_candid_s + center_candid_l
                    # center_points.insert(0, [-1, 0])
                    center_points = self.interpolate_points(center_points, interval=0.5)

                    center_points = self.sort_by_closest_points(center_points, [0, 0])
                    center_points = self.select_uniform_points(center_points, num_points=5)
                    center_points  = self.remove_points_in_circles(right_points, center_points, radius=0.3)

       
                    
                
                    return center_points
                else:
                    return []
            elif len(left_curve_points) > 1: 
               
                return self.extrapolate_line(right_points , offset= straight_offset , direction='right') 
                
            elif len(straight_points) > 1:
                #print(f'3333333333333333: {straight_points}')
                return self.extrapolate_line(right_points , offset= straight_offset , direction='right') 
            
            else:    
                return []

        elif turn_direction == 1:
            # 오른쪽 좌표 필터링
            right_curve_points = left_points[left_points[:, 1] < 0]
            straight_points = left_points[left_points[:, 1] > 0]
            
            if len(right_curve_points) > 1 and len(straight_points) > 1: 
                print('11111111111111111111111111')

                # right_curve_points의 최소 x 좌표보다 작은 straight_points 선택
                straight_final_points = straight_points[straight_points[:, 0] < np.min(right_curve_points[:, 0]-0.1)]
                print(f'straight:{len(straight_final_points)} right:{len(right_curve_points)}')
                if len(straight_final_points) > 1 and len(right_curve_points) > 1:
                    #print(f'straight:{len(straight_final_points)} right:{len(right_curve_points)}')
                    # extrapolate_line 호출
                    #curve_point_r = [point.tolist() for point in [straight_final_points[-1] , right_curve_points[0]]]
                    center_candid_s = self.extrapolate_line( straight_final_points , offset=s_offset, direction='left')
                    
                    center_candid_l = self.extrapolate_line(right_curve_points, offset=l_offset, direction='left')
                    
                    center_points = center_candid_s + center_candid_l 
                    print(center_points)                   
                    center_points = self.sort_by_closest_points(center_points, [0, 0])

                    center_points = self.interpolate_points(center_points, interval=0.5)
                    center_points  = self.remove_points_in_circles(left_points, center_points, radius=0.3)

                    # print(center_points)
                    center_points = self.select_uniform_points(center_points, num_points=5)
                    return center_points
                return []
            
            elif len(right_curve_points) > 0: 
                print('22222222222222222')
                center_points = self.extrapolate_line(left_points , offset= straight_offset , direction='left') 
                return center_points
            elif len(straight_points) > 0:
                #print(f'3333333333333333: {straight_points}')
                return self.extrapolate_line(left_points , offset= straight_offset , direction='left') 
            
            else:
                return []
        
        else:
            return []
            

    def extrapolate_line(self, line, offset=1.5, direction='right'):
        """단일 라인에서 법선 벡터와 기울기, 각도를 이용하여 중심선을 추정 (45도에서 120도까지 점진적으로 증가)"""
        extrapolated_points = []
        try:
            if len(line) >= 2:
                total_points = len(line) - 1  # 총 점 개수
                start_angle = np.radians(0)  # 시작 각도 (45도)
                end_angle = np.radians(0)   # 마지막 각도 (120도)

                for i in range(0, len(line) - 1):
                    p1 = np.array(line[i], dtype=np.float64)
                    p2 = np.array(line[i + 1], dtype=np.float64)

                    # 방향 벡터 계산
                    direction_vector = p2 - p1
                    norm = np.linalg.norm(direction_vector)

                    # 벡터의 크기가 0인 경우 건너뛰기
                    if norm == 0:
                        rospy.logwarn(f"Zero-length direction vector at index {i}. Skipping this point.")
                        continue

                    direction_vector /= norm

                    # 점 개수에 비례한 각도 증가
                    progress_ratio = i / total_points  # 진행률 계산 (0부터 1까지)
                    current_angle = start_angle + (end_angle - start_angle) * progress_ratio  # 각도 점진적으로 증가

                    # 법선 벡터 계산 (기본적으로 오른쪽 법선 벡터)
                    normal_vector = np.array([-direction_vector[1], direction_vector[0]])

                    if direction == 'left':
                        normal_vector = -normal_vector  # 왼쪽 방향이면 법선 벡터 반전

                    # 각도에 따른 회전 변환 행렬 계산
                    rotation_matrix = np.array([[np.cos(current_angle), -np.sin(current_angle)],
                                                [np.sin(current_angle), np.cos(current_angle)]])
                    
                    # 회전 변환 적용
                    rotated_normal_vector = np.dot(rotation_matrix, normal_vector)

                    # 조정된 법선 벡터를 이용해 중심선을 안쪽으로 꺾음
                    center_x2 = p2[0] + offset * rotated_normal_vector[0]
                    center_y2 = p2[1] + offset * rotated_normal_vector[1]

                    # 새로운 중심선 추가
                    extrapolated_points.append([center_x2, center_y2])

        except Exception as e:
            rospy.logwarn(f"Error in extrapolating line: {e}")

        return extrapolated_points




    def adjust_center_line(self, center_points, left_line, right_line):
        """center_line을 보정하는 함수"""
        center_points = self.remove_nan_and_duplicates(center_points)
        center_points.append((0,0))
        # print(f"center_points: {center_points}")
        
        center_points = self.sort_by_closest_points(center_points, [-1, 0])


        if len(center_points) > 1:
            center_points = self.interpolate_points(center_points, interval=0.2)
            
        center_points  = self.remove_points_in_circles(left_line, center_points, radius=1.0)
        center_points  = self.remove_points_in_circles(right_line, center_points, radius=1.0)
       
        center_points = self.sort_by_closest_points(center_points, [-1, 0])
        #center_points = self.replace_points_with_polyfit(center_points)

        
        # origin = np.array([-1, 0])
        # distances = [np.linalg.norm(np.array(p) - origin) for p in center_points]
        # closest_idx = np.argmin(distances)
        # closest_point = center_points[closest_idx]

        # num_interpolated_points = 3
        # x_interp = np.linspace(origin[0], closest_point[0], num=num_interpolated_points + 2)[1:-1]
        # y_interp = np.linspace(origin[1], closest_point[1], num=num_interpolated_points + 2)[1:-1]
        # interpolated_points = [[x, y] for x, y in zip(x_interp, y_interp)]
        
        # center_points.extend(interpolated_points)
        # center_points.append(closest_point)
        
        center_points = self.filter_points(center_points, min_distance=0.5, max_distance=8.0)
        center_points = self.sort_by_closest_points(center_points, [-0.5, 0])
        #center_points = self.select_points_by_distance(center_points, num_points=5, distance=1.5)
        center_points = self.select_uniform_points(center_points, num_points=4)
        return center_points



    def select_points_by_distance(self, center_points, num_points=5, distance=1.0):
        selected_points = []
        current_point = center_points[0]
        selected_points.append(current_point)
        
        for _ in range(1, num_points):
            next_point = None
            min_distance_diff = float('inf')

            # 선택된 점은 제외하기 위해 새로운 리스트를 사용
            remaining_points = [point for point in center_points if point not in selected_points]

            for point in remaining_points:
                # 두 점 사이의 유클리드 거리 계산
                dist = np.linalg.norm(np.array(point) - np.array(current_point))
                distance_diff = abs(dist - distance)
                
                # 거리 차이가 가장 작은 점을 선택
                if distance_diff < min_distance_diff:
                    min_distance_diff = distance_diff
                    next_point = point
            
            if next_point is not None:
                selected_points.append(next_point)
                current_point = next_point
        
        return selected_points


    def publish_center_points(self, center_points):
        """center_points를 publish하는 함수"""    
        # def pose_callback(self, data):
        #     self.pose = [data.x, data.y]
        #     self.heading = data.z

        point_cloud_msg = PointCloud()
        point_cloud_msg.header = Header()
        point_cloud_msg.header.stamp = rospy.Time.now() 
        point_cloud_msg.header.frame_id = "velodyne" 
        point_cloud_msg.points = [Point32(p[0], p[1], 0) for p in center_points]

        self.track_line_pub.publish(point_cloud_msg)
    
    def visualize_lines(self, left_line, right_line, center_points):
        """left_line, right_line, center_line 시각화"""
        marker_array = MarkerArray()
       
    #    processed = self.tf2tm(erp.lidar, self.pose[0], self.pose[1], self.heading)
        if left_line:
            marker_array.markers.append(self.create_left_marker(left_line))
        if right_line:
            marker_array.markers.append(self.create_right_marker(right_line))
        if center_points:
            center_marker = self.create_center_marker(center_points)
            if center_marker:
                marker_array.markers.append(center_marker)
            
        
        if marker_array.markers:
            self.marker_array_pub.publish(marker_array)
            
    # def tf2tm(self, no_z_points, x, y, heading):
    #     obs_tm=np.empty((1,3))
    #     T = [[np.cos(heading), -1*np.sin(heading), x], \
    #          [np.sin(heading),    np.cos(heading), y], \
    #          [      0     ,        0       , 1]] 
    #     for point in no_z_points:
    #         obs_tm = np.append(obs_tm,[np.dot(T,np.transpose([point[0]+1, point[1],1]))],axis=0) # point[0] -> 객체를 subscribe할 수 없음 오류
    #     obs_tm[:,2]=0
    #     obs_tm = np.delete(obs_tm, (0), axis = 0) 
    #     return obs_tm

    #     self.delete_markers()  # 시각화 후 삭제하여 남아있지 않도록 처리

    # def delete_markers(self):
    #     """이전 마커들을 삭제"""
    #     delete_marker_array = MarkerArray()
    #     delete_marker = Marker()
    #     delete_marker.action = Marker.DELETEALL
    #     delete_marker_array.markers.append(delete_marker)
        
    #     self.marker_array_pub.publish(delete_marker_array)  # Publish as a MarkerArray

    def create_center_marker(self, center_points):
        """중심선 마커 (초록색) 생성"""
        center_marker = Marker()
        center_marker.header.frame_id = "velodyne"
        center_marker.header.stamp = rospy.Time.now()
        center_marker.ns = "center_line"
        center_marker.id = 2
        center_marker.type = Marker.POINTS
        center_marker.action = Marker.ADD
        center_marker.scale.x = 0.2  # 점의 크기
        center_marker.scale.y = 0.2  # 점의 크기
        center_marker.color.r = 0.0
        center_marker.color.g = 1.0
        center_marker.color.b = 0.0
        center_marker.color.a = 1.0

        valid_points = []
        for p in center_points:
            if not np.isnan(p[0]) and not np.isnan(p[1]) and not np.isinf(p[0]) and not np.isinf(p[1]):
                valid_points.append(Point32(p[0], p[1], 0))

        if not valid_points:
            rospy.logwarn("No valid center points available for visualization.")
            return None

        center_marker.points = valid_points
        return center_marker

    def create_left_marker(self, left_line):
        """왼쪽 라인 마커 생성 (파란색)"""
        left_marker = Marker()
        left_marker.header.frame_id = "velodyne"
        left_marker.header.stamp = rospy.Time.now()
        left_marker.ns = "left_line"
        left_marker.id = 0
        left_marker.type = Marker.POINTS
        left_marker.action = Marker.ADD
        left_marker.scale.x = 0.2
        left_marker.scale.y = 0.2
        left_marker.color.r = 0.0
        left_marker.color.g = 0.0
        left_marker.color.b = 1.0
        left_marker.color.a = 1.0
        left_marker.points = [Point32(p[0], p[1], 0) for p in left_line]
        return left_marker

    def create_right_marker(self, right_line):
        """오른쪽 라인 마커 생성 (노란색)"""
        right_marker = Marker()
        right_marker.header.frame_id = "velodyne"
        right_marker.header.stamp = rospy.Time.now()
        right_marker.ns = "right_line"
        right_marker.id = 1
        right_marker.type = Marker.POINTS
        right_marker.action = Marker.ADD
        right_marker.scale.x = 0.2
        right_marker.scale.y = 0.2
        right_marker.color.r = 1.0
        right_marker.color.g = 1.0
        right_marker.color.b = 0.0
        right_marker.color.a = 1.0
        right_marker.points = [Point32(p[0], p[1], 0) for p in right_line]
        return right_marker


if __name__ == '__main__':
    try:
        mission_tracking = MissionTracking()
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            mission_tracking.process_centerline()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass