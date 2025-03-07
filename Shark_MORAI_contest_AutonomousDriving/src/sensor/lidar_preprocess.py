#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
import time
import numpy as np
import open3d as o3d
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import PointCloud, PointCloud2, ChannelFloat32
from geometry_msgs.msg import Point32
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial import ConvexHull

'''
    장애물을 군집화해서 군집점들의 특징점을 보내주는 코드입니다. (sparse 한 라이다 군집점을 보내줌)
    라이다 x 0.4 y +- 0.8 안보내지게 
'''

class Detection:
    def __init__(self):
        # self.lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback, queue_size=1)
        self.cluster_pub = rospy.Publisher('object3D', PointCloud, queue_size=1)
        self.lidar_flag = False
        self.obs_xyz = []

    def lidar_callback(self, lidar):
        self.obs_xyz = np.array(list(pc2.read_points(lidar, field_names=("x", "y", "z"), skip_nans=True)))
        self.lidar_timestamp = lidar.header.stamp
        self.lidar_flag = True
        

    def filter_points(self, points):
        # x가 0.4 이하이고 y가 -0.8 이상 +0.8 이하인 점들을 제거
        mask = (points[:, 0] > 0.4) | (points[:, 1] < -0.8) | (points[:, 1] > 0.8)
        filtered_points = points[mask]
        return filtered_points

    def z_compressor_open3d(self, input_data):
        input_data[:, 2] *= 1/20
        return input_data

    def process(self):
        if len(self.obs_xyz) == 0:
            return None

        # 필터링된 점들만 사용
        self.a = time.time()
        filtered_points = self.filter_points(self.obs_xyz)

        # Open3D PointCloud 객체 생성
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(filtered_points)

        # ROI 적용 
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-2, -6, -0.8), max_bound=(20, 6, 1)) #라이다 높이 1.2m
        # 라이다 좌표계 x, y, z = 앞, 왼, 위 
        pcd = pcd.crop(bbox)

        # Voxel 다운샘플링
        pcd = pcd.voxel_down_sample(voxel_size=0.04)

        # # RANSAC 적용
        # _, road_inliers = pcd.segment_plane(distance_threshold=0.2, ransac_n=3, num_iterations=100)
        # pcd = pcd.select_by_index(road_inliers, invert=True)

        # numpy 배열로 변환
        points_list = np.asarray(pcd.points)

        # Z 축 압축
        compressed_points_list = self.z_compressor_open3d(points_list)

        return compressed_points_list

    def dbscan(self, epsilon, min_points):
        input_data = self.process()
        if input_data is None or len(input_data) == 0:
            return [], []

        model = DBSCAN(eps=epsilon, min_samples=min_points)
        labels = model.fit_predict(input_data)
        # print(labels)

        no_noise_mask = labels != -1
        no_noise_data = input_data[no_noise_mask]
        no_noise_labels = labels[no_noise_mask]

        # Z 값 복원
        no_noise_data[:, 2] *= 20

        return no_noise_data, no_noise_labels

    def extract_boundary_points(self, points):
        if len(points) < 3:
            return points  # Convex Hull을 계산하기에 점이 충분하지 않다면, 그대로 반환

        hull = ConvexHull(points[:, :2])  # 2D Convex Hull 계산 (x, y 좌표 사용)
        boundary_points = points[hull.vertices]  # Convex Hull의 점들 추출

        if len(boundary_points) > 2:
            # 시작점과 끝점 포함
            start_point = boundary_points[0]
            end_point = boundary_points[-1]

            # 중간에 균등한 간격으로 선택할 인덱스 계산 (시작점과 끝점을 제외한 중간 점들)
            indices = np.linspace(1, len(boundary_points) - 2, 3, dtype=int)  # 3개의 중간 특징점 선택
            selected_points = boundary_points[indices]

            # 시작점, 중간 점들, 끝점을 결합하여 최종적으로 반환
            boundary_points = np.vstack([start_point, selected_points, end_point])
            # boundary_points = np.vstack([start_point, end_point])

        return boundary_points


    def compute_mean_point(self, points):
        return np.mean(points, axis=0)
    

    def show_clusters(self,points):
        self.obs_xyz = points
        obs_xyz, labels = [], []
        obs_xyz, labels = self.dbscan(epsilon=0.2, min_points=5)
        # print(len(obs_xyz))
        # print(labels)
        # print(type(labels))

        if obs_xyz is None or labels is None:
            return
        # print("장애물 수 : ", labels.max() + 1)
        
        self.local_lidar = PointCloud()
        # channel = ChannelFloat32()
        # channel.values = labels.tolist()
        # print(channel.values)

        unique_labels = np.unique(labels)
        for label in unique_labels:
            if label == -1:
                continue  # 노이즈 라벨은 무시

            cluster_points = obs_xyz[labels == label]

            # sparse(듬성듬성)하게 군집된 점 받으려면 
            boundary_points = self.extract_boundary_points(cluster_points)  
    
            # 군집의 중심점(평균점) 계산
            mean_point = self.compute_mean_point(cluster_points)
            
            # 중심점을 boundary_points에 추가
            boundary_points = np.vstack([boundary_points, mean_point])
            
            # 군집화된거 다 보려면 
            # boundary_points = obs_xyz 

            for i in boundary_points:
                point = Point32(x=i[0], y=i[1], z=i[2])
                self.local_lidar.points.append(point)
                # channel.values.append(label)  # 각 점에 라벨 추가

        # self.local_lidar.channels.append(channel)
        self.local_lidar.header.frame_id = 'velodyne'
        self.cluster_pub.publish(self.local_lidar)
        # print(time.time() - self.a)

def main():
    rospy.init_node('lidar_preprocess')
    D = Detection()
    start_rate = time.time()
    
    while not rospy.is_shutdown():
        if time.time() - start_rate > 0.01:
            if D.lidar_flag:
                D.show_clusters()

            start_rate = time.time()  # 시간 초기화

if __name__ == '__main__':
    main()
# #!/usr/bin/env python3
# # -- coding: utf-8 --

# import rospy
# import time
# import numpy as np
# import open3d as o3d
# from sklearn.cluster import DBSCAN
# from sensor_msgs.msg import PointCloud, PointCloud2, ChannelFloat32
# from geometry_msgs.msg import Point32
# import sensor_msgs.point_cloud2 as pc2
# from scipy.spatial import ConvexHull

# '''
#     장애물을 군집화해서 군집점들의 특징점을 보내주는 코드입니다. (sparse 한 라이다 군집점을 보내줌)
# '''

# class Detection:
#     def __init__(self):
#         # self.lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback, queue_size=1)
#         self.cluster_pub = rospy.Publisher('object3D', PointCloud, queue_size=1)
#         self.lidar_flag = False
#         self.obs_xyz = []

#     def lidar_callback(self, lidar):
#         self.obs_xyz = np.array(list(pc2.read_points(lidar, field_names=("x", "y", "z"), skip_nans=True)))
#         self.lidar_timestamp = lidar.header.stamp
#         self.lidar_flag = True

#     def z_compressor_open3d(self, input_data):
#         input_data[:, 2] *= 1/5
#         return input_data

#     def process(self):
#         if len(self.obs_xyz) == 0:
#             return None

#         # Open3D PointCloud 객체 생성
#         pcd = o3d.geometry.PointCloud()
#         pcd.points = o3d.utility.Vector3dVector(self.obs_xyz)

#         # ROI 적용 
#         bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(1.5, -6, -2), max_bound=(20, 6, 1)) 
#         # 라이다 좌표계 x, y, z = 앞, 왼, 위 
#         pcd = pcd.crop(bbox)

#         # Voxel 다운샘플링
#         pcd = pcd.voxel_down_sample(voxel_size=0.04)

#         # RANSAC 적용
#         _, road_inliers = pcd.segment_plane(distance_threshold=0.2, ransac_n=3, num_iterations=100)
#         pcd = pcd.select_by_index(road_inliers, invert=True)

#         # numpy 배열로 변환
#         points_list = np.asarray(pcd.points)

#         # Z 축 압축
#         compressed_points_list = self.z_compressor_open3d(points_list)

#         return compressed_points_list

#     def dbscan(self, epsilon, min_points):
#         input_data = self.process()
#         if input_data is None or len(input_data) == 0:
#             return [], []

#         model = DBSCAN(eps=epsilon, min_samples=min_points)
#         labels = model.fit_predict(input_data)

#         no_noise_mask = labels != -1
#         no_noise_data = input_data[no_noise_mask]
#         no_noise_labels = labels[no_noise_mask]

#         # Z 값 복원
#         no_noise_data[:, 2] *= 5

#         return no_noise_data, no_noise_labels

#     def extract_boundary_points(self, points):
#         if len(points) < 3:
#             return points  # Convex Hull을 계산하기에 점이 충분하지 않다면, 그대로 반환

#         hull = ConvexHull(points[:, :2])  # 2D Convex Hull 계산 (x, y 좌표 사용)
#         boundary_points = points[hull.vertices]  # Convex Hull의 점들 추출

#         # print(len(boundary_points) )
#         if len(boundary_points) > 0:
#             # 시작점과 끝점 포함
#             start_point = boundary_points[0]
#             end_point = boundary_points[-1]

#             # 중간에 균등한 간격으로 선택할 인덱스 계산 (시작점과 끝점을 제외한 중간 점들)
#             indices = np.linspace(1, len(boundary_points) - 2, 3, dtype=int)  # 3개의 중간 특징점 선택
#             selected_points = boundary_points[indices]
#             # print(len(boundary_points) )

#             # 시작점, 중간 점들, 끝점을 결합하여 최종적으로 반환
#             boundary_points = np.vstack([start_point, selected_points, end_point])

#         return boundary_points


#     def show_clusters(self, points):
#         self.obs_xyz = points
#         obs_xyz, labels = [], []
#         obs_xyz, labels = self.dbscan(epsilon=0.5, min_points=5)
#         if obs_xyz is None or labels is None:
#             return
#         # print("장애물 수 : ", labels.max() + 1)
#         self.local_lidar = PointCloud()
#         channel = ChannelFloat32()
#         # channel.values = labels.tolist()

#         unique_labels = np.unique(labels)
#         for label in unique_labels:
#             if label == -1:
#                 continue  # 노이즈 라벨은 무시

#             cluster_points = obs_xyz[labels == label]
#             boundary_points = self.extract_boundary_points(cluster_points)

#             for i in boundary_points:
#                 point = Point32(x=i[0], y=i[1], z=i[2])
#                 self.local_lidar.points.append(point)
#                 channel.values.append(label)  # 각 점에 라벨 추가

#         self.local_lidar.channels.append(channel)
#         self.local_lidar.header.frame_id = 'velodyne'
#         self.cluster_pub.publish(self.local_lidar)


# def main():
#     rospy.init_node('lidar_preprocess')
#     D = Detection()
#     print("work")
#     start_rate = time.time()

#     while not rospy.is_shutdown():
#         if time.time() - start_rate > 0.01:
#             if D.lidar_flag:
#                 D.show_clusters()
#             start_rate = time.time()  # 시간 초기화

# if __name__ == '__main__':
#     main()


# #!/usr/bin/env python3
# # -- coding: utf-8 --

# import rospy
# import time
# import numpy as np
# import open3d as o3d
# from sklearn.cluster import DBSCAN
# from sensor_msgs.msg import PointCloud, PointCloud2, ChannelFloat32
# from geometry_msgs.msg import Point32
# import sensor_msgs.point_cloud2 as pc2

# '''
#     라이다 높이 : 1.5643m 실제 세팅으로는 1.2m
#     라이다로 객체의 군집화된 점들의 테두리 점들을 보내는 코드 
#     장애물 판단, 주차장 주차 공간 판단에 쓰일 수 있음
# '''

# class Detection:
#     def __init__(self):
#         self.lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback, queue_size=1)
#         self.cluster_pub = rospy.Publisher('object3D', PointCloud, queue_size=1)
#         self.lidar_flag = False
#         self.obs_xyz = []

#     def lidar_callback(self, lidar):
#         self.obs_xyz = np.array(list(pc2.read_points(lidar, field_names=("x", "y", "z"), skip_nans=True)))
#         self.lidar_timestamp = lidar.header.stamp
#         self.lidar_flag = True

#     def z_compressor_open3d(self, input_data):
#         input_data[:, 2] *= 1/5
#         return input_data

#     def process(self):
#         if len(self.obs_xyz) == 0:
#             return None

#         # Open3D PointCloud 객체 생성
#         pcd = o3d.geometry.PointCloud()
#         pcd.points = o3d.utility.Vector3dVector(self.obs_xyz)

#         # ROI 적용 
#         bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(0, -6, -1.5), max_bound=(20, 6, 1)) 
#         # 라이다 좌표계 x, y, z = 앞, 왼, 위 
#         pcd = pcd.crop(bbox)

#         # Voxel 다운샘플링
#         pcd = pcd.voxel_down_sample(voxel_size=0.04)

#         # RANSAC 적용
#         _, road_inliers = pcd.segment_plane(distance_threshold=0.2, ransac_n=3, num_iterations=100)
#         pcd = pcd.select_by_index(road_inliers, invert=True)

#         # numpy 배열로 변환
#         points_list = np.asarray(pcd.points)

#         # Z 축 압축
#         compressed_points_list = self.z_compressor_open3d(points_list)

#         return compressed_points_list

#     def dbscan(self, epsilon, min_points):
#         input_data = self.process()
#         if input_data is None or len(input_data) == 0:
#             return [], []

#         model = DBSCAN(eps=epsilon, min_samples=min_points)
#         labels = model.fit_predict(input_data)

#         no_noise_mask = labels != -1
#         no_noise_data = input_data[no_noise_mask]
#         no_noise_labels = labels[no_noise_mask]

#         # Z 값 복원
#         no_noise_data[:, 2] *= 5

#         return no_noise_data, no_noise_labels

#     def show_clusters(self):
#         obs_xyz, labels = self.dbscan(epsilon=0.5, min_points=5)
#         print("장애물 수 : ",labels.max() + 1)
        
#         self.local_lidar = PointCloud()
#         channel = ChannelFloat32()
#         channel.values = labels.tolist()

#         for i in obs_xyz:
#             point = Point32(x=i[0], y=i[1], z=i[2])
#             self.local_lidar.points.append(point)
        
#         self.local_lidar.channels.append(channel)
#         self.local_lidar.header.frame_id = 'velodyne'
#         self.cluster_pub.publish(self.local_lidar)

# def main():
#     rospy.init_node('lidar_preprocess')
#     D = Detection()
#     print("work")
#     start_rate = time.time()

#     while not rospy.is_shutdown():
#         if time.time() - start_rate > 0.01:
#             if D.lidar_flag:
#                 D.show_clusters()
#             start_rate = time.time()  # 시간 초기화

# if __name__ == '__main__':
#     main()


# 최적화 시키기 이전 코드 

# #!/usr/bin/env python3
# # -- coding: utf-8 -- 

# import rospy
# import time
# import os
# import sys
# from math import *
# from std_msgs.msg import Float64, Bool, String, Float32
# from sensor_msgs.msg import PointCloud, Imu, PointCloud2, ChannelFloat32
# from geometry_msgs.msg import Point, Point32, Quaternion, Twist
# from nav_msgs.msg import Odometry
# import sensor_msgs.point_cloud2 as pc2
# from tf.transformations import euler_from_quaternion
# import open3d as o3d
# import numpy as np
# import matplotlib.pyplot as plt
# from z_lidar_module import lidar_module
# from sklearn.cluster import DBSCAN


# class Detection:
#     def __init__(self):
#         self.lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback, queue_size=1)
#         self.cluster_pub = rospy.Publisher('object3D', PointCloud, queue_size=1)
#         self.lidar_flag = False  # lidar_flag 속성을 초기화합니다.
#         self.obs_xyz = [0,0,0]
        

#     def lidar_callback(self, lidar):
#         self.obs_xyz = list(map(lambda x: list(x), pc2.read_points(lidar, field_names=("x", "y", "z"), skip_nans=True)))
#         # print(self.obs_xyz)
#         self.lidar_timestamp = lidar.header.stamp
#         self.lidar_flag = True


#     def z_compressor_open3d(self, input_data):
#         def z_com(input_point):
#             input_point[2] = input_point[2] * (1/5)
#             return input_point
#         # 이미 input_data는 리스트 형태의 점들이므로, 직접 z_com 함수를 적용합니다.
#         compressed_data = list(map(z_com, input_data))
#         return compressed_data
    

#     ##ROI & Voxel & RANSAC##
#     def process(self):
#         if not self.obs_xyz:
#         # If no points are available, return None
#             return None
        
#         # Open3D의 PointCloud 객체를 생성
#         pcd = o3d.geometry.PointCloud()  
#         # 리스트에서 Open3D의 Vector3dVector로 변환하여 할당
#         pcd.points = o3d.utility.Vector3dVector(self.obs_xyz)  
        
#         ##ROI##
#         # 크롭 박스의 경계 정의 (XYZ 최소 및 최대)
#         bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(0, -6, -5), max_bound=(20, 6, 1)) 
#         pcd = pcd.crop(bbox) # 크롭 박스에 포함된 포인트만 필터링
        
#         ##Voxel##
#         pcd = pcd.voxel_down_sample(voxel_size=0.2)   
#         ##remove outliers## => 필요없을듯 적용하면 너무 빡세서 잘 안됨
#         #pcd, inliers = pcd.remove_radius_outlier(nb_points=20, radius=0.3)

#         ##Ransac##
#         plane_model, road_inliers = pcd.segment_plane(distance_threshold=0.2, ransac_n=3, num_iterations=100)
#         pcd = pcd.select_by_index(road_inliers, invert=True)  
        
#         # pcd.points는 Vector3dVector 객체이므로, 이를 리스트로 변환
#         points_list = np.asarray(pcd.points)
#         # z값 압축을 위해 z_compressor 호출, np.asarray로 변환된 points_list 전달
#         compressed_points_list = self.z_compressor_open3d(points_list)
#         # 압축된 점들로 새로운 PointCloud 객체 생성
#         compressed_pcd = o3d.geometry.PointCloud()
#         compressed_pcd.points = o3d.utility.Vector3dVector(compressed_points_list)
#         # return list(compressed_pcd.points)
#         return compressed_pcd.points


#     def dbscan(self, epsilon, min_points): 
#         # eps과 min_points가 입력된 모델 생성
#         model = DBSCAN(eps=epsilon, min_samples=min_points)
#         input_data = self.process()  # process() 메서드를 통해 데이터를 가져옴
#         if not input_data:
#             return [], []
#         # 데이터를 라이브러리가 읽을 수 있게 np array로 변환
#         DB_Data = np.array(input_data, dtype=object) 
#         # 모델 예측
#         labels = model.fit_predict(DB_Data)
#         k=0
#         ## input_data의 인덱스와 labels[k]의 인덱스가 서로 대응된다고 보는게 맞을듯 => 즉 n번 점의 labels 값이 서로 대응되는 값 
#         no_noise_model=[]
#         no_noise_label=[]
#         for i in input_data:
#             if labels[k] != -1 :
#                 z=i[2]*5
#                 no_noise_model.append([i[0],i[1],z])
#                 no_noise_label.append(labels[k])
#             k+=1
#         return no_noise_model, no_noise_label
    
#     def show_clusters(self): 
#         obs_xyz, labels = self.dbscan(epsilon=0.4, min_points=10) #0.2
#         # 가공한 데이터 msg 에 담기
#         self.local_lidar = PointCloud()
#         channel = ChannelFloat32()
#         channel.values = labels
#         for i in obs_xyz:
#             point = Point32()
#             point.x = i[0]
#             point.y = i[1]
#             point.z = i[2]
#             self.local_lidar.points.append(point)
#         self.local_lidar.channels.append(channel)
#         self.local_lidar.header.frame_id = 'velodyne' 
#         self.cluster_pub.publish(self.local_lidar)
    
        
# def main():
#     rospy.init_node('lidar_preprocess') #, anonymous=True
#     D = Detection()
#     print("work")
#     start_rate = time.time()  
#     while not rospy.is_shutdown():
#         if time.time() - start_rate > 0.01:
#             if D.lidar_flag is True:
#                 processed_data = D.process()
#                 if processed_data is not None:
#                     D.show_clusters()

# if __name__ == '__main__':
#     main() 