#!/usr/bin/env python3
# -- coding: utf-8 --

import os
import sys
import rospy
import math
import time
import numpy as np
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from scipy.spatial import KDTree

class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class MissionTracking:
    def __init__(self):
        rospy.init_node('mission_tracking_node')
        # 기본 파라미터 설정
        self.raw = []  # 원시 포인트 클라우드 데이터를 저장할 리스트
        self.offset = 3  # 도로 폭 (미터)
        self.center_offset = 1.2 # 중심 이격 거리
        self.noise_offset = 1.8  # 노이즈 허용 거리
        self.look_distance = 1.5  # 추종 거리
        self.coefficient = 1.0  # 보정치
        self.min_dis = 1  # 최소 거리
        self.max_dis = 3  # 최대 거리
        self.flag = True
        self.cones = []  # 콘 포인트 리스트
        self.blue_cones = []
        self.yellow_cones = []

        # ROS Publisher 및 Subscriber 설정
        self.track_line_pub = rospy.Publisher('/track_line', PointCloud, queue_size=1)
        self.marker_array_pub = rospy.Publisher('/marker_array', MarkerArray, queue_size=1)
        rospy.Subscriber('/point_assigned', PointCloud2, self.point_callback)

    def remove_duplicates(self, x, y):
        """중복된 x 값을 제거하고 y 값을 조정합니다."""
        try:
            unique_x, indices = np.unique(x, return_index=True)
            unique_y = y[indices]
            return unique_x, unique_y
        except Exception as e:
            rospy.logwarn(f"Error in removing duplicates: {e}")
            return x, y


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

            # Ensure 'cone_points' is a list of Point objects
            filtered_points = self.filter_points([(p.x, p.y, p.z) for p in cone_points])

            for point in filtered_points:
                # 'point' here is now a list or tuple of coordinates
                x, y, z = point
                self.raw.append([x, y, z])
                dis = np.sqrt(x**2 + y**2)

                if z == 0:
                    self.blue_cones.append([x, y])
                elif z == 1:
                    self.yellow_cones.append([x, y])
            print(self.blue_cones)
        except Exception as e:
            rospy.logwarn(f"Error in point_callback: {e}")

        rospy.loginfo(f"Processing time: {time.time() - start_time:.2f} seconds")
    
    def remove_duplicates(self, x, y):
        """중복된 x 값을 제거하고 y 값을 조정합니다."""
        try:
            unique_x, indices = np.unique(x, return_index=True)
            unique_y = y[indices]
            return unique_x, unique_y
        except Exception as e:
            rospy.logwarn(f"Error in removing duplicates: {e}")
            return x, y

    def filter_points(self, points, min_distance=0.5, max_distance=10.0):
        """포인트 클라우드 필터링"""
        return [point for point in points if min_distance < np.sqrt(point[0]**2 + point[1]**2) < max_distance]

    def calculate_deltas(self, p1, p2):
        """두 점 사이의 x 방향 변화량과 y 방향 변화량 계산 함수"""
        delta_x = p2[0] - p1[0]
        delta_y = p2[1] - p1[1]
        return delta_x, delta_y

    def filter_points_between_x(self, line, x1, x2):
        """주어진 x 좌표 구간 내에 있는 점들을 필터링하는 함수"""
        return [p for p in line if min(x1, x2) < p[0] < max(x1, x2)]

    
    def add_interpolated_points(self, points, num_new_points=1):
        """각 점 쌍 사이에 새로운 점들을 추가합니다."""
        new_points = []
        if len(points) < 2:
            return points

        for i in range(len(points) - 1):
            p1 = np.array(points[i])
            p2 = np.array(points[i + 1])
            distance = np.linalg.norm(p2 - p1)
            if distance > self.look_distance:
                num_interpolated = int(distance / self.look_distance)
                for j in range(num_interpolated + 1):
                    t = j / float(num_interpolated + 1)
                    new_points.append(list(p1 + t * (p2 - p1)))
            new_points.append(points[i + 1])

        return new_points


    def process_centerline(self):
        left_cones = self.blue_cones
        right_cones = self.yellow_cones

        left_cones.append([0, +1])  # 왼쪽 콘에 기준 점 추가
        right_cones.append([0, -1])  # 오른쪽 콘에 기준 점 추가
        left_cones.sort(key=lambda x: x[0])
        right_cones.sort(key=lambda x: x[0])
        
        # left_line = self.track_cones(left_cones)
        # right_line = self.track_cones(right_cones)
        
        left_line = left_cones
        right_line = right_cones
        
        if not left_line or not right_line:
            rospy.logwarn("Insufficient cones to generate lines")
            return

        center_points = self.calculate_centerline(left_line, right_line)
        if not center_points:
            rospy.logwarn("Failed to calculate centerline")
            return

        # 시각화 및 PointCloud 발행
        self.visualize_and_publish(center_points, left_line, right_line)

    # def track_cones(self, cones):
    #     """콘들을 추적하여 정렬된 라인 반환"""
    #     line = []
    #     try:
    #         if cones:
    #             line.append(cones[0])
    #             current_cone = cones[0]

    #             # KD-Tree를 이용한 가장 가까운 점 찾기
    #             tree = KDTree(cones)
    #             while len(cones) > 1:
    #                 distances, indices = tree.query(current_cone, k=2)
    #                 next_cone_idx = indices[1]
    #                 next_cone = cones[next_cone_idx]
    #                 if next_cone:
    #                     line.append(next_cone)
    #                     current_cone = next_cone
    #                     cones.remove(current_cone)
    #                     tree = KDTree(cones)
    #                 else:
    #                     break
    #     except Exception as e:
    #         rospy.logwarn(f"Error in tracking cones: {e}")
    #     return line

    def euclidean_distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
    
    def calculate_centerline(self, left_line, right_line):
        """왼쪽과 오른쪽 라인에서 중심선을 계산하고, 점 사이의 거리가 self.look_distance 이상인 경우 보간"""
        center_points = []
        try:
            left_points = np.array(left_line)
            right_points = np.array(right_line)

            if len(left_points) >= 2 and len(right_points) >= 2:
                
                if len(left_points) <= len(right_points):
                    for left_point in left_points:
                        # 모든 right_point에 대해 유클리드 거리 계산
                        distances = np.array([self.euclidean_distance(left_point, right_point) for right_point in right_points])
                        
                        # 가장 가까운 right_point 찾기
                        min_idx = np.argmin(distances)  # 유클리드 거리로 가장 가까운 인덱스 찾기
                        closest_right_point = right_points[min_idx]

                        center_x = (left_point[0] + closest_right_point[0]) / 2
                        center_y = (left_point[1] + closest_right_point[1]) / 2

                        center_points.append([center_x, center_y])
                else:
                    for right_point in right_points:
                        # 모든 left_point에 대해 유클리드 거리 계산
                        distances = np.array([self.euclidean_distance(right_point, left_point) for left_point in left_points])
                        
                        # 가장 가까운 left_point 찾기
                        min_idx = np.argmin(distances)  # 유클리드 거리로 가장 가까운 인덱스 찾기
                        closest_left_point = left_points[min_idx]

                        center_x = (right_point[0] + closest_left_point[0]) / 2
                        center_y = (right_point[1] + closest_left_point[1]) / 2

                        center_points.append([center_x, center_y])

                # 중심선을 x 기준으로 오름차순 정렬
                center_points.sort(key=lambda x: x[0])


                # 중앙점 보간 로직
                for i in range(1, len(center_points)):
                    p1 = center_points[i - 1]
                    p2 = center_points[i]
                    distance = np.hypot(p2[0] - p1[0], p2[1] - p1[1])  # 두 중앙점 사이의 유클리드 거리 계산

                    if distance > self.look_distance:  # 거리가 self.look_distance보다 클 때
                        # x 좌표 구간 내에 있는 left_line과 right_line의 점들 찾기
                        left_points_in_range = self.filter_points_between_x(left_line, p1[0], p2[0])
                        right_points_in_range = self.filter_points_between_x(right_line, p1[0], p2[0])

                        # left_points_in_range 또는 right_points_in_range에 점이 2개 이상 있을 경우 보간 수행
                        if len(left_points_in_range) >= 2:
                            for j in range(len(left_points_in_range) - 1):
                                p1_range = left_points_in_range[j]
                                p2_range = left_points_in_range[j + 1]
                                delta_x, delta_y = self.calculate_deltas(p1_range, p2_range)

                                # 중간 x 좌표 계산
                                interpolated_point_x = p1[0] + delta_x 

                                # 보간할 지점: p1에 delta_x와 delta_y를 추가
                                interpolated_point_y = p1[1] + delta_y 
                                interpolated_point = [interpolated_point_x, interpolated_point_y]

                                # 보간된 중앙점 추가
                                center_points.insert(i, interpolated_point)
                                i += 1  # 보간된 점을 추가했으므로 인덱스를 증가

                        if len(right_points_in_range) >= 2:
                            for j in range(len(right_points_in_range) - 1):
                                p1_range = right_points_in_range[j]
                                p2_range = right_points_in_range[j + 1]
                                delta_x, delta_y = self.calculate_deltas(p1_range, p2_range)

                                # 중간 x 좌표 계산
                                interpolated_point_x = p1[0] + delta_x 

                                # 보간할 지점: p1에 delta_x와 delta_y를 추가
                                interpolated_point_y = p1[1] + delta_y 
                                interpolated_point = [interpolated_point_x, interpolated_point_y]

                                # 보간된 중앙점 추가
                                center_points.insert(i, interpolated_point)
                                i += 1  # 보간된 점을 추가했으므로 인덱스를 증가
            # 왼쪽 라인이 많은 경우
            elif len(left_line) > 2:
                #return self.extrapolate_line(left_line, offset=self.center_offset, direction='left')
                pass
            
            # 오른쪽 라인이 많은 경우
            elif len(right_line) > 2:
                #return self.extrapolate_line(right_line, offset=self.center_offset, direction='right')
                pass

        except Exception as e:
            rospy.logwarn(f"Error in calculating centerline: {e}")
            return []

        return center_points



    def extrapolate_line(self, line, offset=1.5, direction='right'):
        """단일 라인에서 법선 벡터를 이용하여 중심선을 추정"""
        extrapolated_points = []
        try:
            if len(line) >= 2:
                for i in range(len(line)-1):
                    p1 = np.array(line[i])
                    p2 = np.array(line[i+1])

                    # 방향 벡터 계산
                    direction_vector = p2 - p1
                    direction_vector /= np.linalg.norm(direction_vector)

                    # 법선 벡터 계산
                    normal_vector = np.array([-direction_vector[1], direction_vector[0]])  # 오른쪽 법선 벡터

                    if direction == 'left':
                        normal_vector = -normal_vector  # 왼쪽 방향이면 법선 벡터 반전

                    # # 첫 번째 점과 두 번째 점에 대해 중심선을 계산
                    # center_x1 = p1[0] + offset * normal_vector[0]
                    # center_y1 = p1[1] + offset * normal_vector[1]
                    center_x2 = p2[0] + offset * normal_vector[0]
                    center_y2 = p2[1] + offset * normal_vector[1]

                    # 새로운 중심선 추가
                    # extrapolated_points.append([center_x1, center_y1])
                    extrapolated_points.append([center_x2, center_y2])
                    extrapolated_points

        except Exception as e:
            rospy.logwarn(f"Error in extrapolating line: {e}")
        
        # 중복된 점 제거
        extrapolated_points = [list(t) for t in {tuple(point) for point in extrapolated_points}]
        extrapolated_points.append([0,0])
        extrapolated_points.sort(key=lambda x: x[0])
        # extrapolated_points = self.add_interpolated_points(extrapolated_points, num_new_points=1)
        return extrapolated_points

    def visualize_and_publish(self, center_points, left_line, right_line):
        """포인트 시각화 및 발행"""
        try:
            if not center_points or not left_line or not right_line:
                rospy.logwarn("No valid data to visualize and publish")
                return

            # left_line = self.add_interpolated_points(left_line, num_new_points=10)
            # right_line = self.add_interpolated_points(right_line, num_new_points=10)

            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "velodyne"

            point_cloud_msg = PointCloud()
            point_cloud_msg.header = header
            point_cloud_msg.points = [Point32(p[0], p[1], 0) for p in self.select_uniform_points(center_points)]

            self.track_line_pub.publish(point_cloud_msg)

            marker_array = MarkerArray()
            marker_array.markers.extend(self.create_markers(left_line, right_line, center_points))
            self.marker_array_pub.publish(marker_array)
            
            # 디버깅: 시각화할 데이터의 범위 출력
            rospy.loginfo(f"Center points range: x: ({min(p[0] for p in center_points)}, {max(p[0] for p in center_points)}), y: ({min(p[1] for p in center_points)}, {max(p[1] for p in center_points)})")
            rospy.loginfo(f"Left line points range: x: ({min(p[0] for p in left_line)}, {max(p[0] for p in left_line)}), y: ({min(p[1] for p in left_line)}, {max(p[1] for p in left_line)})")
            rospy.loginfo(f"Right line points range: x: ({min(p[0] for p in right_line)}, {max(p[0] for p in right_line)}), y: ({min(p[1] for p in right_line)}, {max(p[1] for p in right_line)})")

        except Exception as e:
            rospy.logwarn(f"Error in visualization and publishing: {e}")

    def select_uniform_points(self, points, num_points=100):
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

    def create_markers(self, left_line, right_line, center_points):
        """마커 배열을 생성"""
        print(center_points)
        markers = []
        try:
            # 왼쪽 라인 마커 (파란색)
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
            markers.append(left_marker)

            # 오른쪽 라인 마커 (노란색)
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
            markers.append(right_marker)

            # 중심선 마커 (초록색)
            center_marker = self.create_center_marker(center_points)
            if center_marker:
                markers.append(center_marker)
            else:
                rospy.logwarn("Failed to create valid center line marker.")
            
        except Exception as e:
            rospy.logwarn(f"Error in creating markers: {e}")
        
        return markers

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


if __name__ == '__main__':
    try:
        mission_tracking = MissionTracking()
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            mission_tracking.process_centerline()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass




# #!/usr/bin/env python3
# # -- coding: utf-8 --

# import os
# import sys
# import rospy
# import math
# import time
# import numpy as np
# from sensor_msgs.msg import PointCloud
# from geometry_msgs.msg import Point32
# from visualization_msgs.msg import Marker, MarkerArray
# from std_msgs.msg import Header
# from sensor_msgs import point_cloud2
# from sensor_msgs.msg import PointCloud2

# class Point:
#     def __init__(self, x, y, z):
#         self.x = x
#         self.y = y
#         self.z = z

# class MissionTracking:
#     def __init__(self):
#         rospy.init_node('mission_tracking_node')
#         # 기본 파라미터 설정
#         self.raw = []  # 원시 포인트 클라우드 데이터를 저장할 리스트
#         self.offset = 3  # 도로 폭 (미터)
#         self.center_offset = 1.2 # 중심 이격 거리
#         self.noise_offset = 1.8  # 노이즈 허용 거리
#         self.look_distance = 1.5  # 추종 거리
#         self.coefficient = 1.0  # 보정치
#         self.min_dis = 1  # 최소 거리
#         self.max_dis = 3  # 최대 거리
#         self.flag = True
#         self.cones = []  # 콘 포인트 리스트
        
#         self.blue_cones = []
#         self.yellow_cones = []
#         # ROS Publisher 및 Subscriber 설정
#         self.track_line_pub = rospy.Publisher('/track_line', PointCloud, queue_size=1)
#         self.marker_array_pub = rospy.Publisher('/marker_array', MarkerArray, queue_size=1)
#         rospy.Subscriber('/point_assigned', PointCloud2, self.point_callback)

#     def remove_duplicates(self, x, y):
#         """중복된 x 값을 제거하고 y 값을 조정합니다."""
#         try:
#             unique_x, indices = np.unique(x, return_index=True)
#             unique_y = y[indices]
#             return unique_x, unique_y
#         except Exception as e:
#             rospy.logwarn(f"Error in removing duplicates: {e}")
#             return x, y

#     def add_interpolated_points(self, points, num_new_points=1):
#         """각 점 쌍 사이에 새로운 점들을 추가합니다."""
#         new_points = []
#         try:
#             for i in range(len(points) - 1):
#                 p1 = np.array(points[i])
#                 p2 = np.array(points[i + 1])
#                 for j in range(num_new_points + 1):
#                     t = j / float(num_new_points + 1)
#                     new_x = p1[0] + t * (p2[0] - p1[0])
#                     new_y = p1[1] + t * (p2[1] - p1[1])
#                     new_points.append([new_x, new_y])
#             new_points.append(points[-1])
#         except Exception as e:
#             rospy.logwarn(f"Error in interpolation: {e}")
#         return new_points

#     def point_callback(self, input_rosmsg):
#         start_time = time.time()

#         self.blue_cones = []
#         self.yellow_cones = []
#         self.raw = []

#         try:
#             cone_points = []
#             for point in point_cloud2.read_points(input_rosmsg, field_names=("x", "y", "z", "intensity")):
#                 cone_points.append(Point(point[0], point[1], point[2]))

#             cone_points = np.array(cone_points)
#             if cone_points.size == 0:
#                 rospy.logwarn("No cone points received")
#                 return

#             for point in cone_points:
#                 self.raw.append([point.x, point.y, point.z])
#                 dis = np.sqrt(point.x**2 + point.y**2)

#                 if point.z == 0:
#                     self.blue_cones.append([point.x, point.y])
#                 elif point.z == 1:
#                     self.yellow_cones.append([point.x, point.y])

#         except Exception as e:
#             rospy.logwarn(f"Error in point_callback: {e}")

#         rospy.loginfo(f"Processing time: {time.time() - start_time:.2f} seconds")

#     def process_centerline(self):
#         left_cones = self.blue_cones
#         right_cones = self.yellow_cones

#         left_cones.append([0, +1])  # 왼쪽 콘에 기준 점 추가
#         right_cones.append([0, -1])  # 오른쪽 콘에 기준 점 추가
#         left_cones.sort(key=lambda x: x[0])
#         right_cones.sort(key=lambda x: x[0])

#         # left_line = self.track_cones(left_cones)
#         # right_line = self.track_cones(right_cones)
#         left_line = left_cones
#         right_line = right_cones
#         if not left_line or not right_line:
#             rospy.logwarn("Insufficient cones to generate lines")
#             return

#         center_points = self.calculate_centerline(left_line, right_line)
#         if not center_points:
#             rospy.logwarn("Failed to calculate centerline")
#             return

#         # 시각화 및 PointCloud 발행
#         self.visualize_and_publish(center_points, left_line, right_line)

#     def track_cones(self, cones):
#         """콘들을 추적하여 정렬된 라인 반환"""
#         line = []
#         try:
#             if cones:
#                 line.append(cones[0])
#                 current_cone = cones[0]

#                 while len(cones) > 1:
#                     next_cone = self.find_closest_point(current_cone, cones[1:])
#                     if next_cone:
#                         line.append(next_cone)
#                         current_cone = next_cone
#                         cones.remove(current_cone)
#                     else:
#                         break
#         except Exception as e:
#             rospy.logwarn(f"Error in tracking cones: {e}")
#         return line

#     def find_closest_point(self, current_point, points):
#         """현재 점에 가장 가까운 점을 찾습니다."""
#         min_dist = float('inf')
#         closest_point = None
#         try:
#             for point in points:
#                 dist = np.sqrt((current_point[0] - point[0]) ** 2 + (current_point[1] - point[1]) ** 2)
#                 if dist < min_dist:
#                     min_dist = dist
#                     closest_point = point
#         except Exception as e:
#             rospy.logwarn(f"Error in finding closest point: {e}")

#         if min_dist < 100.0:
#             return closest_point
#         return None

#     def euclidean_distance(self, point1, point2):
#         return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

#     def calculate_centerline(self, left_line, right_line):
#         """왼쪽과 오른쪽 라인에서 중심선을 계산하고, 점 사이의 거리가 self.look_distance 이상인 경우 보간"""
#         center_points = []
#         try:
#             # 왼쪽과 오른쪽 라인이 모두 유효한 경우
#             if len(left_line) >= 2 and len(right_line) >= 2:
#                 center_points = []
#                 if len(left_line) <= len(right_line):
#                     # left_line을 기준으로 각 left_point에 대해 right_line에서 가장 가까운 점을 찾음
#                     for left_point in left_line:
#                         closest_right_point = min(right_line, key=lambda right_point: self.euclidean_distance(left_point, right_point))

#                         center_x = (left_point[0] + closest_right_point[0]) / 2
#                         center_y = (left_point[1] + closest_right_point[1]) / 2

#                         center_points.append([center_x, center_y])
#                 else:
#                     # right_line을 기준으로 각 right_point에 대해 left_line에서 가장 가까운 점을 찾음
#                     for right_point in right_line:
#                         closest_left_point = min(left_line, key=lambda left_point: self.euclidean_distance(right_point, left_point))

#                         center_x = (right_point[0] + closest_left_point[0]) / 2
#                         center_y = (right_point[1] + closest_left_point[1]) / 2

#                         center_points.append([center_x, center_y])

#                 # 중심선을 x 기준으로 오름차순 정렬
#                 center_points.sort(key=lambda x: x[0])

#                 # 두 점 사이의 x 방향 변화량과 y 방향 변화량 계산 함수
#                 def calculate_deltas(p1, p2):
#                     delta_x = p2[0] - p1[0]
#                     delta_y = p2[1] - p1[1]
#                     return delta_x, delta_y

#                 # 주어진 x 좌표 구간 내에 있는 점들을 필터링하는 함수
#                 def filter_points_between_x(line, x1, x2):
#                     return [p for p in line if min(x1, x2) < p[0] < max(x1, x2)]

#                 # 중앙점 보간 로직
#                 for i in range(1, len(center_points)):
#                     p1 = center_points[i - 1]
#                     p2 = center_points[i]
#                     distance = np.hypot(p2[0] - p1[0], p2[1] - p1[1])  # 두 중앙점 사이의 유클리드 거리 계산

#                     if distance > self.look_distance:  # 거리가 self.look_distance보다 클 때
#                         # x 좌표 구간 내에 있는 left_line과 right_line의 점들 찾기
#                         left_points_in_range = filter_points_between_x(left_line, p1[0], p2[0])
#                         right_points_in_range = filter_points_between_x(right_line, p1[0], p2[0])

#                         # left_points_in_range 또는 right_points_in_range에 점이 2개 이상 있을 경우 보간 수행
#                         if len(left_points_in_range) >= 2:
#                             for j in range(len(left_points_in_range) - 1):
#                                 p1_range = left_points_in_range[j]
#                                 p2_range = left_points_in_range[j + 1]
#                                 delta_x, delta_y = calculate_deltas(p1_range, p2_range)

#                                 # 중간 x 좌표 계산
#                                 interpolated_point_x = (p1[0] + p2[0]) / 2

#                                 # 보간할 지점: p1에 delta_x와 delta_y를 추가
#                                 interpolated_point_y = p1[1] + delta_y * (interpolated_point_x - p1_range[0]) / delta_x
#                                 interpolated_point = [interpolated_point_x, interpolated_point_y]

#                                 # 보간된 중앙점 추가
#                                 center_points.insert(i, interpolated_point)
#                                 i += 1  # 보간된 점을 추가했으므로 인덱스를 증가

#                         if len(right_points_in_range) >= 2:
#                             for j in range(len(right_points_in_range) - 1):
#                                 p1_range = right_points_in_range[j]
#                                 p2_range = right_points_in_range[j + 1]
#                                 delta_x, delta_y = calculate_deltas(p1_range, p2_range)

#                                 # 중간 x 좌표 계산
#                                 interpolated_point_x = (p1[0] + p2[0]) / 2

#                                 # 보간할 지점: p1에 delta_x와 delta_y를 추가
#                                 interpolated_point_y = p1[1] + delta_y * (interpolated_point_x - p1_range[0]) / delta_x
#                                 interpolated_point = [interpolated_point_x, interpolated_point_y]

#                                 # 보간된 중앙점 추가
#                                 center_points.insert(i, interpolated_point)
#                                 i += 1  # 보간된 점을 추가했으므로 인덱스를 증가
#             # 왼쪽 라인이 많은 경우
#             elif len(left_line) > 2:
#                 return self.extrapolate_line(left_line, offset=self.center_offset, direction='left')
            
#             # 오른쪽 라인이 많은 경우
#             elif len(right_line) > 2:
#                 return self.extrapolate_line(right_line, offset=self.center_offset, direction='right')
#         except Exception as e:
#             rospy.logwarn(f"Error in calculating centerline: {e}")

#         return center_points


#     def extrapolate_line(self, line, offset=1.5, direction='right'):
#         """단일 라인에서 법선 벡터를 이용하여 중심선을 추정"""
#         extrapolated_points = []
#         try:
#             if len(line) >= 2:
#                 for i in range(len(line)-1):
#                     p1 = np.array(line[i])
#                     p2 = np.array(line[i+1])

#                     # 방향 벡터 계산
#                     direction_vector = p2 - p1
#                     direction_vector /= np.linalg.norm(direction_vector)

#                     # 법선 벡터 계산
#                     normal_vector = np.array([-direction_vector[1], direction_vector[0]])  # 오른쪽 법선 벡터

#                     if direction == 'left':
#                         normal_vector = -normal_vector  # 왼쪽 방향이면 법선 벡터 반전

#                     # # 첫 번째 점과 두 번째 점에 대해 중심선을 계산
#                     # center_x1 = p1[0] + offset * normal_vector[0]
#                     # center_y1 = p1[1] + offset * normal_vector[1]
#                     center_x2 = p2[0] + offset * normal_vector[0]
#                     center_y2 = p2[1] + offset * normal_vector[1]

#                     # 새로운 중심선 추가
#                     # extrapolated_points.append([center_x1, center_y1])
#                     extrapolated_points.append([center_x2, center_y2])
#                     extrapolated_points

#         except Exception as e:
#             rospy.logwarn(f"Error in extrapolating line: {e}")
        
#         # 중복된 점 제거
#         extrapolated_points = [list(t) for t in {tuple(point) for point in extrapolated_points}]
#         extrapolated_points.append([0,0])
#         extrapolated_points.sort(key=lambda x: x[0])
#         # extrapolated_points = self.add_interpolated_points(extrapolated_points, num_new_points=1)
#         return extrapolated_points
#     def visualize_and_publish(self, center_points, left_line, right_line):
#         try:
#             # Track line visualization
#             track_line_msg = PointCloud()
#             track_line_msg.header = Header()
#             track_line_msg.header.stamp = rospy.Time.now()
#             track_line_msg.header.frame_id = 'base_link'
#             track_line_msg.points = [Point32(x=p[0], y=p[1], z=0) for p in center_points]

#             # Track line publisher
#             self.track_line_pub.publish(track_line_msg)

#             # Visualization markers
#             marker_array = MarkerArray()

#             # Left line markers
#             left_marker = Marker()
#             left_marker.header.frame_id = 'base_link'
#             left_marker.header.stamp = rospy.Time.now()
#             left_marker.ns = 'left_line'
#             left_marker.id = 0
#             left_marker.type = Marker.LINE_STRIP
#             left_marker.action = Marker.ADD
#             left_marker.scale.x = 0.1
#             left_marker.color.r = 1.0
#             left_marker.color.g = 0.0
#             left_marker.color.b = 0.0
#             left_marker.color.a = 1.0
#             left_marker.points = [Point32(x=p[0], y=p[1], z=0) for p in left_line]
#             marker_array.markers.append(left_marker)

#             # Right line markers
#             right_marker = Marker()
#             right_marker.header.frame_id = 'base_link'
#             right_marker.header.stamp = rospy.Time.now()
#             right_marker.ns = 'right_line'
#             right_marker.id = 1
#             right_marker.type = Marker.LINE_STRIP
#             right_marker.action = Marker.ADD
#             right_marker.scale.x = 0.1
#             right_marker.color.r = 0.0
#             right_marker.color.g = 0.0
#             right_marker.color.b = 1.0
#             right_marker.color.a = 1.0
#             right_marker.points = [Point32(x=p[0], y=p[1], z=0) for p in right_line]
#             marker_array.markers.append(right_marker)

#             # Center line markers
#             center_marker = Marker()
#             center_marker.header.frame_id = 'base_link'
#             center_marker.header.stamp = rospy.Time.now()
#             center_marker.ns = 'center_line'
#             center_marker.id = 2
#             center_marker.type = Marker.LINE_STRIP
#             center_marker.action = Marker.ADD
#             center_marker.scale.x = 0.1
#             center_marker.color.r = 0.0
#             center_marker.color.g = 1.0
#             center_marker.color.b = 0.0
#             center_marker.color.a = 1.0
#             center_marker.points = [Point32(x=p[0], y=p[1], z=0) for p in center_points]
#             marker_array.markers.append(center_marker)

#             # Marker array publisher
#             self.marker_array_pub.publish(marker_array)
#         except Exception as e:
#             rospy.logwarn(f"Error in visualization and publishing: {e}")

# if __name__ == '__main__':
#     mission_tracking = MissionTracking()
#     rate = rospy.Rate(10)  # 10 Hz
#     while not rospy.is_shutdown():
#         mission_tracking.process_centerline()
#         rate.sleep()
