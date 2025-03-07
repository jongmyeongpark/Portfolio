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
    장애물을 군집화해서 군집점들의 특징점을 보내주는 코드입니다. 
'''

class Detection:
    def __init__(self):
        self.lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback, queue_size=1)
        self.cluster_pub = rospy.Publisher('object3D', PointCloud, queue_size=1)
        self.lidar_flag = False
        self.obs_xyz = []

    def lidar_callback(self, lidar):
        self.obs_xyz = np.array(list(pc2.read_points(lidar, field_names=("x", "y", "z"), skip_nans=True)))
        self.lidar_timestamp = lidar.header.stamp
        self.lidar_flag = True

    def z_compressor_open3d(self, input_data):
        input_data[:, 2] *= 1/5
        return input_data

    def process(self):
        if len(self.obs_xyz) == 0:
            return None

        # Open3D PointCloud 객체 생성
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.obs_xyz)

        # ROI 적용 
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(0, -5, -0.9), max_bound=(10, 5, 1)) 
        # 라이다 좌표계 x, y, z = 앞, 왼, 위 
        pcd = pcd.crop(bbox)

        # Voxel 다운샘플링
        pcd = pcd.voxel_down_sample(voxel_size=0.04)

        # RANSAC 적용
        _, road_inliers = pcd.segment_plane(distance_threshold=0.1, ransac_n=3, num_iterations=100)
        pcd = pcd.select_by_index(road_inliers, invert=True)

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

        no_noise_mask = labels != -1
        no_noise_data = input_data[no_noise_mask]
        no_noise_labels = labels[no_noise_mask]

        # Z 값 복원
        no_noise_data[:, 2] *= 5

        return no_noise_data, no_noise_labels

    # def extract_boundary_points(self, points):
    #     if len(points) < 3:
    #         return points  # Convex Hull을 계산하기에 점이 충분하지 않다면, 그대로 반환

    #     hull = ConvexHull(points[:, :2])  # 2D Convex Hull 계산 (x, y 좌표 사용)
    #     boundary_points = points[hull.vertices]  # Convex Hull의 점들 추출

    #     # print(len(boundary_points) )
    #     if len(boundary_points) > 0:
    #         # 시작점과 끝점 포함
    #         start_point = boundary_points[0]
    #         end_point = boundary_points[-1]

    #         # 중간에 균등한 간격으로 선택할 인덱스 계산 (시작점과 끝점을 제외한 중간 점들)
    #         indices = np.linspace(1, len(boundary_points) - 2, 3, dtype=int)  # 3개의 중간 특징점 선택
    #         selected_points = boundary_points[indices]
    #         print(len(boundary_points) )

    #         # 시작점, 중간 점들, 끝점을 결합하여 최종적으로 반환
    #         boundary_points = np.vstack([start_point, selected_points, end_point])

    #     return boundary_points


    def show_clusters(self):
        obs_xyz, labels = self.dbscan(epsilon=0.5, min_points=3)
        # print("장애물 수 : ", labels.max() + 1)
        
        self.local_lidar = PointCloud()
        channel = ChannelFloat32()
        channel.values = []  # 빈 리스트로 초기화

        unique_labels = np.unique(labels)
        for label in unique_labels:
            if label == -1:
                continue  # 노이즈 라벨은 무시

            cluster_points = obs_xyz[labels == label]
            for point in cluster_points:
                self.local_lidar.points.append(Point32(x=point[0], y=point[1], z=point[2]))
                channel.values.append(float(label))  # 각 점에 라벨 추가 (리스트에 추가)

        self.local_lidar.channels.append(channel)
        self.local_lidar.header.frame_id = 'velodyne'
        self.cluster_pub.publish(self.local_lidar)


def main():
    rospy.init_node('lidar_preprocess')
    D = Detection()
    print("work")
    start_rate = time.time()

    while not rospy.is_shutdown():
        if time.time() - start_rate > 0.01:
            if D.lidar_flag:
                D.show_clusters()
            start_rate = time.time()  # 시간 초기화

if __name__ == '__main__':
    main()

