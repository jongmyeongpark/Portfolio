#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
import numpy as np
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import Point
import csv
import os
import joblib
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

pkl_path = '/home/pc/catkin_ws/src/morai_2/src/sensor/sub.pkl'
MODEL = joblib.load(pkl_path)

class ProcessedPointCloudSub:
    def __init__(self):
        # self.imu_sub = rospy.Subscriber('/current_pose', Point, self.current_pose_callback, queue_size=1)
        # self.lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback, queue_size=1)

        # self.lidar_pub = rospy.Publisher('/processed_pointcloud', PointCloud2, queue_size=1)
        self.heading = None

        self.current_pose_x = None
        self.current_pose_y = None

        self.global_mean_cloud = np.array([])
        self.noise_global_mean_cloud = []

        # File setup
        self.csv_file_path = r'/home/pc/catkin_ws/src/morai_2/src/sensor/shark_dynamic_static_data.csv'
        self.initialize_csv()

    def initialize_csv(self):
        """Initialize the CSV file with headers if it does not exist."""
        if not os.path.isfile(self.csv_file_path):
            with open(self.csv_file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['x', 'y', 'label'])

    def current_pose_callback(self, current_pose_msg: Point) -> None:
        self.current_pose_x = current_pose_msg.x
        self.current_pose_y = current_pose_msg.y
        self.heading = current_pose_msg.z

    def lidar_callback(self, lidar_msg: PointCloud2) -> None:
        points = list(map(lambda x: list(x), pc2.read_points(lidar_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)))
        prev_global_mean_cloud = self.global_mean_cloud.copy()
        
        # 데이터 모을때 
        # self.save_points_to_csv([[self.current_pose_x, self.current_pose_y]], 1) # 동적 장애물
        if points:
            points_np = np.array(points)
            o3d_point_cloud = o3d.geometry.PointCloud()
            o3d_point_cloud.points = o3d.utility.Vector3dVector(points_np[:, :3])

            crop_cloud = self.apply_roi(o3d_point_cloud)
            dbscan_cloud = self.apply_dbscan(crop_cloud)
            mean_cloud = self.mean_dbscan(dbscan_cloud)

            self.global_mean_cloud = self.transform_local2global(mean_cloud, points_np[:, :2])

            # == 데이터 모을 때 == 
            # self.save_points_to_csv(self.global_mean_cloud, 0) # 정적 장애물 
            # ====================
        
            dynamic_points = self.tracking_dynamic_global_points(prev_global_mean_cloud, self.global_mean_cloud)

            local_points = self.transform_global2local(dynamic_points) # 확인할 때
            # local_points = self.transform_global2local(self.global_mean_cloud) # 데이터 모을 때 

            processed_points_np = np.asarray(local_points)
            header = lidar_msg.header
            cloud_msg = pc2.create_cloud_xyz32(header, processed_points_np)
            self.lidar_pub.publish(cloud_msg)

    def activate(self, points: list, pose: Point) -> None:
        
        self.current_pose_x, self.current_pose_y = pose.x, pose.y
        self.heading = pose.z
        prev_global_mean_cloud = self.global_mean_cloud.copy()
        
        # 데이터 모을때 
        # self.save_points_to_csv([[self.current_pose_x, self.current_pose_y]], 1) # 동적 장애물
        if points:
            points_np = np.array(points)
            o3d_point_cloud = o3d.geometry.PointCloud()
            o3d_point_cloud.points = o3d.utility.Vector3dVector(points_np[:, :3])

            crop_cloud = self.apply_roi(o3d_point_cloud)
            dbscan_cloud = self.apply_dbscan(crop_cloud)
            mean_cloud = self.mean_dbscan(dbscan_cloud)

            self.global_mean_cloud = self.transform_local2global(mean_cloud, points_np[:, :2])

            # == 데이터 모을 때 == 
            # self.save_points_to_csv(self.global_mean_cloud, 0) # 정적 장애물 
            # ====================
            return self.tracking_dynamic_global_points(prev_global_mean_cloud, self.global_mean_cloud)

        return []
            # local_points = self.transform_global2local(dynamic_points) # 확인할 때
            # local_points = self.transform_global2local(self.global_mean_cloud) # 데이터 모을 때 
            # processed_points_np = np.asarray(local_points)
            # cloud_msg = pc2.create_cloud_xyz32(header, processed_points_np)
            # self.lidar_pub.publish(cloud_msg)

    def apply_roi(self, o3d_point_cloud: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        x_min, x_max, y_min, y_max, z_min, z_max = 0.5, 16, -8, 8, -0.5, 0.5
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(x_min, y_min, z_min), max_bound=(x_max, y_max, z_max))
        crop_cloud = o3d_point_cloud.crop(bbox)
        return crop_cloud

    def apply_dbscan(self, o3d_point_cloud: o3d.geometry.PointCloud, eps: float = 1, min_points: int = 20) -> dict:
        clusters = {}
        if o3d_point_cloud.points:
            labels = np.array(o3d_point_cloud.cluster_dbscan(eps=eps, min_points=min_points))
            o3d_point_cloud_np = np.asarray(o3d_point_cloud.points)
            for label in set(labels):
                if label == -1:
                    continue
                cluster_points = o3d_point_cloud_np[labels == label]
                clusters[label] = cluster_points
        return clusters

    def mean_dbscan(self, clusters: dict) -> list:
        centroids = []
        for _, points in clusters.items():
            centroid = np.mean(points, axis=0)
            centroids.append(centroid)
        return centroids

    def transform_local2global(self, local_points: list, intensities: np.ndarray):
        if self.heading is not None and self.current_pose_x is not None and self.current_pose_y is not None:
            T = [[np.cos(self.heading), -np.sin(self.heading), self.current_pose_x],
                 [np.sin(self.heading), np.cos(self.heading), self.current_pose_y],
                 [0, 0, 1]]
            
            # 라이다 위치: 2.80, 0.0, 0.8
            # GPS 위치: 1.30, 0.0, 0.8
            global_points = []
            for i, (obs_x, obs_y, _) in enumerate(local_points):
                obs_tm = np.dot(T, np.transpose([obs_x+1.5, obs_y, 1]))
                gx = obs_tm[0]
                gy = obs_tm[1]
                global_points.append([gx, gy])

            return global_points
        return []

    def transform_global2local(self, global_points: list):
        if self.heading is not None and self.current_pose_x is not None and self.current_pose_y is not None:
            local_points = []

            for gp in global_points:
                gp_x, gp_y = gp[0], gp[1]

                trans_x = gp_x - self.current_pose_x
                trans_y = gp_y - self.current_pose_y

                local_x = trans_x * np.cos(-self.heading) - trans_y * np.sin(-self.heading)
                local_y = trans_x * np.sin(-self.heading) + trans_y * np.cos(-self.heading)

                local_points.append([local_x-1.5, local_y, 0])

            return local_points
        return []
    
    def compare_prev_current_global_points(self, prev_global_points: list, curr_global_points: np.ndarray):
        dynamic_global_points = []
        for i in range(len(curr_global_points)):
            # print(curr_global_points[i])
            sample = np.array(curr_global_points[i][:2]).reshape(1, -1)
            
            y_pred = MODEL.predict(sample)

            if y_pred == 1:
                dynamic_global_points.append(curr_global_points[i])

            # print('Y prediction: ', y_pred)
        
        print('Dynamic: ', dynamic_global_points)
        return dynamic_global_points[:3]

    def save_points_to_csv(self, points: list, label: int):
        """Append points to the CSV file with the given label."""
        with open(self.csv_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            for point in points:
                x, y = point
                writer.writerow([x, y, label])


    def tracking_dynamic_global_points(self, prev_global_points: list, curr_global_points: np.ndarray):
        rm_first_global_points = self.compare_prev_current_global_points(prev_global_points, curr_global_points)
        return rm_first_global_points

def main():
    rospy.init_node("lidar_processing", anonymous=True)
    pp = ProcessedPointCloud()
    rospy.spin()

if __name__ == "__main__":
    main()