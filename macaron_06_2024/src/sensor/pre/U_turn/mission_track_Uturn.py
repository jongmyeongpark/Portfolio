#!/usr/bin/env python3
# -- coding: utf-8 --

import os
import sys
import rospy
import math
import random
import time
import numpy as np
import copy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Int16, Float32, Header
from macaron_06.msg import erp_read
import heapq
import math

class MissionTracking:
    def __init__(self):
        rospy.init_node('mission_tracking_node')

        # 기본 파라미터 설정
        self.raw = [] 
        self.offset = 3  # 도로 폭 (미터)
        self.center_offset = 1.3  # center 이격 거리
        self.noise_offset = 1.8  # noise 허용 거리
        self.look_distance = 1.5  # 추종 거리
        self.coefficient = 1.0  # 보정치
        self.min_dis = 1  # 최소 거리
        self.max_dis = 3  # 최대 거리

        # RANSAC 파라미터
        self.ransac_num_iter = 100
        self.ransac_dis = 0.3

        # 필터 및 상태 변수
        self.detect_flag = False
        self.out_point_dis = []
        self.out_point_ransac = []
        self.append_noise = []
        self.check_num = 3  # 최소 검출 개수
        self.ex_center_offset = 1.9  # 대체 center 이격 거리
        self.ex_follow_point = [2, 0]  # 대체 추종 점

        # 필터 키고 끄기
        self.distance_filter_flag = False
        self.distance_filter_left_flag = True
        self.distance_filter_right_flag = True

        # RANSAC 및 라인 관련 변수
        self.num_it = 100
        self.t_dis = 0.3
        self.line_offset = 0.3
        self.triangle_center_points = []

        # ROS Publisher 및 Subscriber 설정
        self.marker_array_pub = rospy.Publisher('/marker_array', MarkerArray, queue_size=1)
        self.track_line_pub = rospy.Publisher('/track_line', PointCloud, queue_size=1)  # PointCloud를 게시할 퍼블리셔
        rospy.Subscriber('/cone', PointCloud, self.point_callback)
    
    def add_interpolated_points(self, points, num_new_points=1):
        """각 점 쌍 사이에 새로운 점들을 추가합니다."""
        new_points = []
        for i in range(len(points) - 1):
            p1 = points[i]
            p2 = points[i + 1]
            for j in range(num_new_points + 1):  # num_new_points + 1 because we want to include p2
                t = j / float(num_new_points + 1)
                new_x = p1[0] + t * (p2[0] - p1[0])
                new_y = p1[1] + t * (p2[1] - p1[1])
                new_points.append([new_x, new_y])
        new_points.append(points[-1])  # 마지막 점 추가
        return new_points    
    
    def select_uniform_points(self, points, num_points=5):
        """균일하게 선택된 포인트들을 반환합니다."""
        if len(points) <= num_points:
            return points
        indices = np.linspace(0, len(points) - 1, num=num_points, dtype=int)
        return [points[i] for i in indices]
    
    def point_callback(self, input_rosmsg):
        ''' pointcloud를 callback으로 받는 함수입니다.

        Args
            input_rosmsg : 

        Return
            Return값이 없고, 전역변수에다가 ~ 할당합니다.

        '''
        start_time = time.time()

        # 색상에 따른 콘 분류
        blue_cones = []
        yellow_cones = []
        self.raw = []
        
        min_dis = float('inf')
        cnt_l = 0
        cnt_r = 0

        for point in input_rosmsg.points:
            self.raw.append([point.x, point.y])   
            dis = np.sqrt(point.x**2 + point.y**2)

            if -2 < point.x < 10 and -2.5 < point.y < -0.2:  # 파란 콘
                blue_cones.append([point.x, point.y])
                cnt_r += 1
                min_dis = min(min_dis, dis)
                
            elif -2 < point.x < 10 and 0.2 < point.y < 2.5:  # 노란 콘
                yellow_cones.append([point.x, point.y])
                cnt_l += 1
                min_dis = min(min_dis, dis)
                
            else:  # 노이즈 처리 (추후 활용 예정)
                pass

        # 콘 정렬 및 노이즈 체크
        left_cones = yellow_cones
        right_cones = blue_cones
        print(f"left_cones:{len(left_cones)}")
        print(f"right_cones:{len(right_cones)}")
        # left_cones.append([0, 1])  # 왼쪽 콘에 기준 점 추가
        # right_cones.append([0, -1])  # 오른쪽 콘에 기준 점 추가
        left_cones.sort(key=lambda x: x[0])
        right_cones.sort(key=lambda x: x[0])

        left_line = [left_cones[0]] if left_cones else []
        right_line = [right_cones[0]] if right_cones else []

        def find_closest_point(current_point, points):
            min_dist = float('inf')
            closest_point = None
            for point in points:
                dist = np.sqrt((current_point[0] - point[0])**2 + (current_point[1] - point[1])**2)
                if dist < min_dist:
                    min_dist = dist
                    closest_point = point

            if min_dist < 2.4:
                return closest_point
            
            else:
                return None

        current_left = left_cones[0] if left_cones else None
        current_right = right_cones[0] if right_cones else None

        if left_cones:
            while len(left_cones) > 1:
                next_left = find_closest_point(current_left, left_cones[1:])
                if next_left is None:
                    break
                left_line.append(next_left)
                current_left = next_left
                left_cones.remove(current_left)  # Remove the used point from the list

        if right_cones:
            while len(right_cones) > 1:
                next_right = find_closest_point(current_right, right_cones[1:])
                if next_right is None:
                    break
                right_line.append(next_right)
                current_right = next_right
                right_cones.remove(current_right)  # Remove the used point from the list

        
        center_points = []
        # center_points.append([0, 0])
        # center_points.append([0,0])
        # print(f"left_line_length:{len(left_line)}")
        # print(f"right_line_length:{len(right_line)}")
        if len(left_line) >= 2 and len(right_line) >= 2:
            min_length = min(len(left_line), len(right_line))
            for i in range(min_length):
                center_x = (left_line[i][0] + right_line[i][0]) / 2
                center_y = (left_line[i][1] + right_line[i][1]) / 2
                center_points.append([center_x, center_y])
        
        elif len(left_line) > 1:

            # Use the first two points to calculate the perpendicular offset
            p1 = np.array(left_line[0])
            p2 = np.array(left_line[1])

            # Calculate direction vector from p1 to p2
            direction = p2 - p1
            length = np.linalg.norm(direction)

            # Normalize the direction vector
            direction /= length

            # Calculate the perpendicular vector
            perpendicular_direction = np.array([-direction[1], direction[0]])

            for point in left_line:
                center_x = point[0] - self.center_offset * perpendicular_direction[0]
                center_y = point[1] - self.center_offset * perpendicular_direction[1]
                if center_x < 3.5:
                    center_points.append([center_x, center_y])

        
        elif len(right_line) > 1:
            # Use the first two points to calculate the perpendicular offset
            p1 = np.array(right_line[0])
            p2 = np.array(right_line[1])

            # Calculate direction vector from p1 to p2
            direction = p2 - p1
            length = np.linalg.norm(direction)

            # Normalize the direction vector
            direction /= length

            # Calculate the perpendicular vector
            perpendicular_direction = np.array([-direction[1], direction[0]])

            for point in right_line:
                center_x = point[0] + self.center_offset * perpendicular_direction[0]
                center_y = point[1] + self.center_offset * perpendicular_direction[1]
                if center_x < 3.5:
                    center_points.append([center_x, center_y])
        else:
            # center_points.append([0, 0])
            pass
        
        if 0 < len(center_points) < 5:
            center_points = self.add_interpolated_points(center_points, num_new_points=10)
            selected_points = self.select_uniform_points(center_points, num_points=5)
        elif len(center_points) > 5:
            selected_points = self.select_uniform_points(center_points, num_points=5)
        
       # PointCloud 메시지 생성 및 발행
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "velodyne"
        point_cloud_msg = PointCloud()
        point_cloud_msg.header = header
        point_cloud_msg.points = [Point32(p[0], p[1], 0) for p in selected_points]

        # PointCloud 메시지 발행
        self.track_line_pub.publish(point_cloud_msg)        
        
        # MarkerArray를 사용하여 시각화
        marker_array = MarkerArray()
        marker_id = 0
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        def create_marker(points, color, marker_id):
            for point in points:
                marker = Marker()
                marker.header.frame_id = "velodyne"
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = 0
                marker.scale.x = 0.2# 포인트 크기
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                marker.color.a = 1.0
                marker.id = marker_id
                marker_id += 1  
                marker_array.markers.append(marker)
            return marker_id

        # 중심 라인 포인트 시각화 (초록색)
        marker_id = create_marker(selected_points, (0.0, 1.0, 0.0), marker_id)

        self.marker_array_pub.publish(marker_array)

        end_time = time.time()
        #rospy.loginfo(f'Processing time: {end_time - start_time:.3f} seconds')

if __name__ == '__main__':
    # try:
    #     while not rospy.is_shutdown():
    #         MissionTracking()
    #         rospy.spin()
    # except rospy.ROSInterruptException:
    #     pass
    try:
        mission_tracking = MissionTracking()
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass