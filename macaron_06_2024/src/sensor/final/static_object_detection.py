#!/usr/bin/env python3
# -- coding: utf-8 -- 
# static_object_detection.py 는 open3d를 활용해서 라이다의 데이터를 처리하고 군집화까지 하는 코드입니다.

import rospy
import time
import os
import sys
from math import *
from std_msgs.msg import Float64, Bool, String, Float32, Header
from sensor_msgs.msg import PointCloud2, Imu, ChannelFloat32, PointField
from geometry_msgs.msg import Point32
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
from sklearn.cluster import DBSCAN

class Static_Detection:
    def __init__(self):
        self.lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback, queue_size=1)
        
        self.right_lane_pub = rospy.Subscriber('/right_lane_points', PointCloud2, self.right_lane_callback, queue_size=1)
        self.left_lane_pub = rospy.Subscriber('/left_lane_points', PointCloud2, self.left_lane_callback, queue_size=1)
        
        self.cluster_pub = rospy.Publisher('/obstacle', PointCloud2, queue_size=1)
        
        self.lidar_flag = False 
        self.obs_xyz = [0, 0, 0]
        self.left_lane_points = []
        self.right_lane_points = []
        self.min_y_left = 0
        self.max_y_right = 0 

    # min ,max
 
    def right_lane_callback(self, lidar_msg):
        right_lane_points = list(pc2.read_points(lidar_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        
        if right_lane_points:
            self.max_y_right = max(point[1] for point in right_lane_points)
        else:
            self.max_y_right = 0
        
        self.right_lane_points = right_lane_points

    def left_lane_callback(self, lidar_msg):
        left_lane_points = list(pc2.read_points(lidar_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        
        if left_lane_points:
            self.min_y_left = min(point[1] for point in left_lane_points)
        else:
            self.min_y_left = 0
            
        self.left_lane_points = left_lane_points

    def lidar_callback(self, lidar):
        self.obs_xyz = list(pc2.read_points(lidar, field_names=("x", "y", "z"), skip_nans=True))
        self.lidar_timestamp = lidar.header.stamp
        self.lidar_flag = True
    
    def process(self):
        """
        ## ROI & Voxel & RANSAC ##
        Returns:
           List of outlier points
        """
        if not self.obs_xyz:
            return None
        
        pcd = o3d.geometry.PointCloud()  
        pcd.points = o3d.utility.Vector3dVector(self.obs_xyz)  
        
        ## lane roi 판별 ##
        if self.min_y_left > 0 and self.max_y_right < 0:   
            bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(0, self.max_y_right, -1), max_bound=(10, self.min_y_left, 2)) 
            pcd = pcd.crop(bbox)
        elif self.min_y_left > 0:   
            bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(0, -5, -1), max_bound=(10, self.min_y_left, 2)) 
            pcd = pcd.crop(bbox)
        elif self.max_y_right < 0:   
            bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(0, self.max_y_right, -1), max_bound=(10, 5, 2)) 
            pcd = pcd.crop(bbox)
        else: 
            bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(0, -6, -1), max_bound=(10, 6, 2)) 
            pcd = pcd.crop(bbox)
        

        ## Voxel ##
        pcd = pcd.voxel_down_sample(voxel_size=0.4)   
        pcd, inliers = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        ## Ransac ##
        plane_model, road_inliers = pcd.segment_plane(distance_threshold=0.2, ransac_n=3, num_iterations=100)
        outlier_cloud = pcd.select_by_index(road_inliers, invert=True)  
        outlier_cloud.paint_uniform_color([1, 0, 0])

        return np.asarray(outlier_cloud.points)



    # def crop_cluster_to_roi(self, points, labels, object_type="cone"):
    #     """
    #     DBSCAN 클러스터의 포인트들을 ROI로 잘라내는 함수.
    #     :param points: 전체 포인트 클라우드 배열
    #     :param labels: DBSCAN에서 얻은 클러스터 라벨
    #     :param object_type: "cone" 또는 "person" 중 하나 선택
    #     :return: ROI 내에 있는 포인트 클라우드
    #     """
    #     filtered_points = []

    #     for cluster_label in np.unique(labels):
    #         if cluster_label == -1:
    #             continue  # 노이즈는 제외
            
    #         cluster_points = points[labels == cluster_label]
            
    #         # 클러스터 내 x, y, z의 최소 및 최대 값 계산
    #         x_min, x_max = np.min(cluster_points[:, 0]), np.max(cluster_points[:, 0])
    #         y_min, y_max = np.min(cluster_points[:, 1]), np.max(cluster_points[:, 1])
    #         z_min, z_max = np.min(cluster_points[:, 2]), np.max(cluster_points[:, 2])

    #         # 객체 크기에 맞는 ROI 설정
    #         if object_type == "cone":
    #             min_width, max_width = 0, 2
    #             min_height, max_height = 0.3, 1.0
    #         elif object_type == "person":
    #             min_width, max_width = 0.5, 0.9
    #             min_height, max_height = 1.2, 2.0
    #         else:
    #             raise ValueError("Invalid object_type. Use 'cone' or 'person'.")

    #         # ROI 필터링 조건
    #         width_condition = (y_max - y_min) >= min_width and (y_max - y_min) <= max_width
    #         height_condition = (z_max - z_min) >= min_height and (z_max - z_min) <= max_height
            
    #         if width_condition and height_condition:
    #             print(f"cone -> y_diff:{y_max - y_min}, z_diff: {z_max - z_min}")
    #             for point in cluster_points:
    #                 # Ensure each point is appended as a list or tuple of floats
    #                 filtered_points.append([float(point[0]), float(point[1]), float(point[2])])

    #     return np.array(filtered_points) if filtered_points else np.array([])  # 루프가 끝난 후에 반환

    def crop_cluster_to_roi(self, points, labels, object_type="cone"):
        """
        DBSCAN 클러스터의 포인트들을 ROI로 잘라내는 함수.
        :param points: 전체 포인트 클라우드 배열
        :param labels: DBSCAN에서 얻은 클러스터 라벨
        :param object_type: "cone" 또는 "person" 중 하나 선택
        :return: ROI 내에 있는 포인트 클라우드
        """
        filtered_points = []

        for cluster_label in np.unique(labels):
            if cluster_label == -1:
                continue  # 노이즈는 제외
            
            cluster_points = points[labels == cluster_label]
            
            # 클러스터 내 x, y, z의 최소 및 최대 값 계산
            x_min, x_max = np.min(cluster_points[:, 0]), np.max(cluster_points[:, 0])
            y_min, y_max = np.min(cluster_points[:, 1]), np.max(cluster_points[:, 1])
            z_min, z_max = np.min(cluster_points[:, 2]), np.max(cluster_points[:, 2])

            # 객체 크기에 맞는 ROI 설정
            if object_type == "cone":
                min_width, max_width = 0, 2
                min_height, max_height = 0.3, 1.0
            elif object_type == "person":
                min_width, max_width = 0.5, 0.9
                min_height, max_height = 1.2, 2.0
            else:
                raise ValueError("Invalid object_type. Use 'cone' or 'person'.")

            # ROI 필터링 조건
            width_condition = (y_max - y_min) >= min_width and (y_max - y_min) <= max_width
            height_condition = (z_max - z_min) >= min_height and (z_max - z_min) <= max_height
            
            if width_condition and height_condition:
                if object_type == "cone":
                    print(f"cone -> y_diff:{y_max - y_min}, z_diff: {z_max - z_min}")
                
                elif object_type == "person":
                    print(f"person -> y_diff:{y_max - y_min}, z_diff: {z_max - z_min}")
                
                mean_x = np.mean(cluster_points[:, 0])
                mean_y = np.mean(cluster_points[:, 1])
                mean_z = np.mean(cluster_points[:, 2])
                if object_type == "cone":
                    filtered_points.append([mean_x, mean_y, -0.5])
                    filtered_points.append([mean_x, y_min, -0.5])
                    filtered_points.append([mean_x, y_max, -0.5])
                
                elif object_type == "person":
                    filtered_points.append([mean_x, mean_y, 0.5])
                    

            # if width_condition and height_condition:
            # #     print(f"cone -> y_diff:{y_max - y_min}, z_diff: {z_max - z_min}")
            #     for point in cluster_points:
            #         # Ensure each point is appended as a list or tuple of floats
            #         filtered_points.append([float(point[0]), float(point[1]), float(point[2])])

        return np.array(filtered_points) if filtered_points else np.array([])

    def static_judge(self, epsilon=0.4, min_points=4):
        """
        DBSCAN 클러스터링을 수행하는 함수.
        :param epsilon: 클러스터링 거리 기준 (eps)
        :param min_points: 최소 샘플 개수 (min_samples)
        :return: 노이즈가 제거된 클러스터링 결과 포인트와 라벨 리스트
        """
        self.epsilon = epsilon
        self.min_points = min_points
        
        # 전처리된 데이터 가져오기
        input_data = self.process()
        if input_data is None or input_data.shape[0] == 0:
            rospy.logwarn("No points available for clustering.")
            return [], []
            
        # DBSCAN 클러스터링 수행
        dbscan = DBSCAN(eps=self.epsilon, min_samples=self.min_points)
        labels = dbscan.fit_predict(input_data)

        # 클러스터에서 콘과 사람에 대한 포인트 ROI 추출
        cone_object_points = self.crop_cluster_to_roi(input_data, labels, object_type="cone")
        
        person_object_points = self.crop_cluster_to_roi(input_data, labels, object_type="person")
        
        no_noise_items = []
        no_noise_labels = []

        # 콘 포인트 필터링
        if cone_object_points is not None and len(cone_object_points) > 0:
            for idx, point in enumerate(cone_object_points):
                no_noise_items.append([point[0], point[1], point[2]])  # 콘의 경우 z 값을 1로 설정
                no_noise_labels.append(labels[idx])

        # 사람 포인트 필터링
        if person_object_points is not None and len(person_object_points) > 0:

            for idx, point in enumerate(person_object_points):
                no_noise_items.append([point[0], point[1], point[2]])  # 사람의 경우 z 값을 0으로 설정
                no_noise_labels.append(labels[idx])

        return no_noise_items, no_noise_labels

    def publish_clusters(self):
        """
        클러스터를 시각화하고, ROS PointCloud2 메시지로 퍼블리시하는 함수.
        """
        points, labels = self.static_judge()
        
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'velodyne'  

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
        
        filtered_points = pc2.create_cloud(header, fields, points)

        # 클러스터 퍼블리시
        self.cluster_pub.publish(filtered_points)


if __name__ == '__main__':
    rospy.init_node('static_object_detection_node', anonymous=True)
    
    detector = Static_Detection()
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if detector.lidar_flag:  
            detector.publish_clusters()
        rate.sleep()
