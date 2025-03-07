#!/usr/bin/env python3
# -- coding: utf-8 -- 

#z_lidar_open3d.py 는 open3d를 활용해서 라이다의 데이터를 처리하고 군집화까지 하는 코드입니다.

import rospy
import time
import os
import sys
from math import *
from std_msgs.msg import Float64, Bool, String, Float32
from sensor_msgs.msg import PointCloud, Imu, PointCloud2, ChannelFloat32
from geometry_msgs.msg import Point, Point32, Quaternion, Twist
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2
from tf.transformations import euler_from_quaternion
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from z_lidar_module import lidar_module
from sklearn.cluster import DBSCAN

### 클러스터 중심점 구하는 거 빼고 전체 다 보이게 하고 roi 설정하는 거 까지 해서 수정하자 


class Detection:
    def __init__(self):
        self.lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback, queue_size=1)
        self.cluster_pub = rospy.Publisher('object3D', PointCloud, queue_size=1)
        self.lidar_flag = False  # lidar_flag 속성을 초기화합니다.
        self.obs_xyz = [0,0,0]
        self.z_com_flag = True

        #z_compressor 에 사용됨 
        self.front=20 #몇미터 앞까지 볼건지 원래 20
        self.lidar_module = lidar_module()
        
    
    def lidar_callback(self, lidar):
        self.obs_xyz = list(map(lambda x: list(x), pc2.read_points(lidar, field_names=("x", "y", "z"), skip_nans=True)))
        # print(self.obs_xyz)
        self.lidar_timestamp = lidar.header.stamp
        self.lidar_flag = True
    
    ##ROI & Voxel & RANSAC##
    def process(self):
        if not self.obs_xyz:
        # If no points are available, return None
            return None
        
        # Open3D의 PointCloud 객체를 생성
        pcd = o3d.geometry.PointCloud()  
        # 리스트에서 Open3D의 Vector3dVector로 변환하여 할당
        pcd.points = o3d.utility.Vector3dVector(self.obs_xyz)  
        
        ##ROI##
        # 크롭 박스의 경계 정의 (XYZ 최소 및 최대)
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(0, -5, -1), max_bound=(20, 5, 1)) 
        pcd = pcd.crop(bbox) # 크롭 박스에 포함된 포인트만 필터링
        
        ##Voxel##
        pcd = pcd.voxel_down_sample(voxel_size=0.05)   
        ##remove outliers## => 필요없을듯 적용하면 너무 빡세서 잘 안됨
        #pcd, inliers = pcd.remove_radius_outlier(nb_points=20, radius=0.3)

        ##Ransac##
        plane_model, road_inliers = pcd.segment_plane(distance_threshold=0.1, ransac_n=3, num_iterations=100)
        pcd = pcd.select_by_index(road_inliers, invert=True)  
        
        ##z값 압축##
        #pcd = self.lidar_module.z_compressor(pcd)

        ## z값 압축 안할때 쓰는코드
        #return list(pcd.points)
        
        # pcd.points는 Vector3dVector 객체이므로, 이를 리스트로 변환
        points_list = np.asarray(pcd.points)
        # z값 압축을 위해 z_compressor 호출, np.asarray로 변환된 points_list 전달
        compressed_points_list = self.lidar_module.z_compressor(points_list)
        # 압축된 점들로 새로운 PointCloud 객체 생성
        compressed_pcd = o3d.geometry.PointCloud()
        compressed_pcd.points = o3d.utility.Vector3dVector(compressed_points_list)
        
        return list(compressed_pcd.points)


    def dbscan(self, epsilon=0.2, min_points=4):
        self.epsilon = epsilon
        self.min_points = min_points
        # eps과 min_points가 입력된 모델 생성
        model = DBSCAN(eps=self.epsilon, min_samples=self.min_points)
        
        #여기 예외 처리 
        input_data = self.process()  # process() 메서드를 통해 데이터를 가져옴

        # process() 메서드가 비어있는 배열을 반환하는 경우에 대한 예외 처리
        if not input_data:
            return [], []

        # 데이터를 라이브러리가 읽을 수 있게 np array로 변환
        DB_Data = np.array(input_data, dtype=object) 
        
        # 모델 예측
        labels = model.fit_predict(DB_Data)

        k=0
        ## input_data의 인덱스와 labels[k]의 인덱스가 서로 대응된다고 보는게 맞을듯 => 즉 n번 점의 labels 값이 서로 대응되는 값 
        no_noise_model=[]
        no_noise_label=[]
        for i in input_data:
            if labels[k] != -1 :
                if self.z_com_flag is True:
                    z=i[2]*(self.front*456)/(self.epsilon*10000)
                #점과 라벨이 같은 순서대로 대응되어서 저장됨
                #no_noise_model.append([i[0],i[1],i[2]])
                no_noise_model.append([i[0],i[1],z])
                no_noise_label.append(labels[k])
                # print(no_noise_model,no_noise_label)
            k+=1
                
        return no_noise_model, no_noise_label
    
    def show_clusters(self): 
        obs_xyz, labels = self.dbscan(epsilon=0.2, min_points=4)
        # 가공한 데이터 msg 에 담기
        self.local_lidar = PointCloud()
        channel = ChannelFloat32()
        channel.values = labels
        for i in obs_xyz:
            point = Point32()
            point.x = i[0]
            point.y = i[1]
            point.z = i[2]
            self.local_lidar.points.append(point)
        self.local_lidar.channels.append(channel)
        self.local_lidar.header.frame_id = 'velodyne' 
        self.cluster_pub.publish(self.local_lidar)
    
        
def main():
    rospy.init_node('z_lidar_data_open3d') #, anonymous=True
    D = Detection()
    start_rate = time.time()  
    while not rospy.is_shutdown():
        if time.time() - start_rate > 0.01:
            if D.lidar_flag is True:
                processed_data = D.process()
                if processed_data is not None:
                    # cluster_centers, cluster_label = pcl.dbscan(epsilon=0.2, min_points=4)
                    # if cluster_centers is not None and cluster_label is not None:
                    #     print(cluster_centers)
                    D.show_clusters()
                #else:
                    #print("No clusters found.")

if __name__ == '__main__':
    main() 

