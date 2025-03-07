#!/usr/bin/env python3
# -*-coding:utf-8-*-
import os
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d

from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String

class ProcessedPointCloud:
    def __init__(self):
        self.imu_sub = rospy.Subscriber('/current_pose', Point, self.current_pose_callback, queue_size=1)
        self.lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback, queue_size=1)

        self.lidar_pub = rospy.Publisher('/processed_pointcloud', PointCloud2, queue_size=1)
        self.parking_lot_pub = rospy.Publisher('/parking', PointCloud2, queue_size=1)
        self.status_pub = rospy.Publisher('/parking_status', String, queue_size=1)

        self.current_pose_x = None
        self.current_pose_y = None
        self.gps_position = None
        self.heading = None

        self.local_parking_lot = None
        self.closet_value = None
        self.last_closet_value = None
        self.last_closet_time = None

        self.real_global_lot = np.array([
            [966883.0219870011, 1935790.3820852807,0],
            [966885.260391943, 1935790.4820035344, 0],
            [966887.4655736537, 1935790.3786974582,0],
            [966889.7761404542, 1935790.3912502257,0],
            [966892.0579714774, 1935790.5105478845,0],
            [966894.4252551483, 1935790.6029213013,0],
            [966896.6911982417, 1935790.7080223535,0],
            [966899.0037262351, 1935790.6771503098,0],
            [966901.3016965914, 1935790.7270317753,0],
            [966903.5713436026, 1935790.7908791858,0],
            [966905.8225832751, 1935790.677703329, 0],
            [966908.1275648307, 1935790.752677677, 0],
            [966910.5437012353, 1935790.9481107865,0],
            [966912.8922681527, 1935790.886026902, 0],
            [966915.1466462176, 1935791.0302732498,0],
            [966917.3596532875, 1935790.9033079676,0],
            [966919.65132297, 1935790.9098238335, 0],
            [966921.9385651791, 1935790.9921590027,0],
            [966924.2494093282, 1935791.039545082, 0],
            [966926.575152908, 1935791.0724159572 ,0],
            [966928.8836540156, 1935791.1432509832,0],
            [966931.2032716349, 1935791.1471147034,0],
            [966933.4685309101, 1935791.3095207657,0],
            [966935.709768572, 1935791.1804188914 ,0],
            [966938.0226256344, 1935791.2623150172,0],
            [966940.3465576388, 1935791.384250695, 0],
            [966942.700705846, 1935791.3838278714, 0],
            [966944.9454949885, 1935791.4096894918,0],
            [966947.2444400941, 1935791.467176213, 0],
            [966949.4651861562, 1935791.3976804705,0],
            [966951.7452383387, 1935791.370944586, 0]
        ])

    def current_pose_callback(self, current_pose_msg: Point) -> None:
        self.current_pose_x = current_pose_msg.x
        self.current_pose_y = current_pose_msg.y
        self.heading = current_pose_msg.z

    def lidar_callback(self, lidar_msg: PointCloud2) -> None:
        points = list(map(lambda x: list(x), pc2.read_points(lidar_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)))
        
        if points:
            points_np = np.array(points)
            o3d_point_cloud = o3d.geometry.PointCloud()
            o3d_point_cloud.points = o3d.utility.Vector3dVector(points_np[:, :3])

            crop_cloud = self.apply_roi(o3d_point_cloud)

            dbscan_cloud = self.apply_dbscan(crop_cloud)
            mean_cloud = self.mean_dbscan(dbscan_cloud)
            global_mean_cloud = self.transform_local2global(mean_cloud)
            
            self.canidate_parking_lot(global_mean_cloud)

            processed_points_np = np.asarray(mean_cloud)
            header = lidar_msg.header
            cloud_msg = pc2.create_cloud_xyz32(header, processed_points_np.tolist())
            self.lidar_pub.publish(cloud_msg)

    def publish_local_parking_lot(self):
        if self.local_parking_lot is not None:
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "velodyne"
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1),
                     ]
            
            points = [self.local_parking_lot]
            
            pc2_msg = pc2.create_cloud(header, fields, points)
            self.parking_lot_pub.publish(pc2_msg)

    def apply_roi(self, o3d_point_cloud: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        x_min, x_max, y_min, y_max, z_min, z_max = -3, 10, -5, -1, -0.5, 5
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(x_min, y_min, z_min), max_bound=(x_max, y_max, z_max))
        crop_cloud = o3d_point_cloud.crop(bbox)
        return crop_cloud

    def apply_dbscan(self, o3d_point_cloud: o3d.geometry.PointCloud, eps: float = 0.3, min_points: int = 10) -> dict:
        labels = -1
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
        else:
            return clusters

    def mean_dbscan(self, clusters: dict) -> list:
        centroids = []
        for _, points in clusters.items():
            centroid = np.mean(points, axis=0)
            centroids.append(centroid)
        return centroids        
    
    def transform_local2global(self, mean_cloud: list) -> list:
        if self.heading is not None and self.current_pose_x is not None and self.current_pose_y is not None:
            T = [[np.cos(self.heading), -1*np.sin(self.heading), self.current_pose_x], 
                 [np.sin(self.heading),  np.cos(self.heading), self.current_pose_y], 
                 [      0     ,      0       , 1]] 
            
            global_mean_cloud = []
            for obs_x, obs_y, _ in mean_cloud:
                obs_tm = np.dot(T, np.transpose([obs_x, obs_y, 1]))
                gx = obs_tm[0]
                gy = obs_tm[1]
                global_mean_cloud.append([gx, gy])

            return global_mean_cloud
        return []
    
    def transform_global2local(self, global_parking_lot):
        if self.heading is not None and self.current_pose_x is not None and self.current_pose_y is not None:
            global_parking_lot[0] = global_parking_lot[0] - self.current_pose_x
            global_parking_lot[1] = global_parking_lot[1] - self.current_pose_y

            global_parking_lot[0] = global_parking_lot[0] * np.cos(-self.heading) - global_parking_lot[1] * np.sin(-self.heading)
            global_parking_lot[1] = global_parking_lot[0] * np.sin(-self.heading) + global_parking_lot[1] * np.cos(-self.heading)


            return global_parking_lot

        return []

    def transform_global2local_real_global_lot(self, real_global_parking_lot):
        real_global_parking_lot[0] = real_global_parking_lot[0] - self.current_pose_x
        real_global_parking_lot[1] = real_global_parking_lot[1] - self.current_pose_y

        real_global_parking_lot[0] = real_global_parking_lot[0] * np.cos(-self.heading) - real_global_parking_lot[1] * np.sin(-self.heading)
        real_global_parking_lot[1] = real_global_parking_lot[0] * np.sin(-self.heading) + real_global_parking_lot[1] * np.cos(-self.heading)

        return real_global_parking_lot


    def canidate_parking_lot(self, global_mean_cloud: list, threshold=4):
        def calculate_global_x(global_mean_cloud):
            if global_mean_cloud is not None and len(global_mean_cloud) > 0:
                global_mean_cloud_np = np.array(global_mean_cloud)

                if global_mean_cloud_np.ndim == 1:
                    global_mean_cloud_np_sorted = np.sort(global_mean_cloud_np)
                    global_x_differences = np.diff(global_mean_cloud_np_sorted)
                else:
                    sorted_indices = np.argsort(global_mean_cloud_np[:, 0])
                    global_mean_cloud_np_sorted = global_mean_cloud_np[sorted_indices]
                    global_x_values = global_mean_cloud_np_sorted[:, 0]
                    global_x_differences = np.diff(global_x_values)

                return global_x_differences, global_mean_cloud_np_sorted
            return np.array([0]), np.array([])

        
        global_x_diff, global_mean_cloud_np_sorted = calculate_global_x(global_mean_cloud)
        global_parking_lot_candidate = global_x_diff >= threshold

        if sum(global_parking_lot_candidate) >= 1:
            print("*" * 50)
            true_indices = np.where(global_parking_lot_candidate)[0]
            
            if len(true_indices) >= 2:
                true_indices = true_indices[0]
            indices = int(true_indices)
            
            global_parking_lot = [(global_mean_cloud_np_sorted[indices][0] + global_mean_cloud_np_sorted[indices+1][0]) / 2, 
                           (global_mean_cloud_np_sorted[indices][1] + global_mean_cloud_np_sorted[indices+1][1]) / 2, 0]
            
            # print('Global parking_lot: ', global_parking_lot)
            
            self.closet_value = self.real_global_lot[np.abs(self.real_global_lot[:, 0]-global_parking_lot[0]).argmin()]
                
            # print('closet value: ', self.closet_value)

            self.local_parking_lot = self.transform_global2local_real_global_lot(global_parking_lot)
            # print('Local parking_lot: ', self.local_parking_lot)
            # print("*" * 50)

            self.publish_local_parking_lot()

class ProcessedPointCloud_:
    def __init__(self):
    #     self.imu_sub = rospy.Subscriber('/current_pose', Point, self.current_pose_callback, queue_size=1)
    #     self.lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback, queue_size=1)

        self.lidar_pub = rospy.Publisher('/processed_pointcloud', PointCloud2, queue_size=1)
        # self.parking_lot_pub = rospy.Publisher('/parking', PointCloud2, queue_size=1)

        self.current_pose_x = 0
        self.current_pose_y = 0
        self.gps_position = None
        self.heading = None

        self.gps2lidar = 1.5
        
        self.local_parking_lot = None
        self.closet_value = []
        self.last_closet_value = None
        self.last_closet_time = None

        self.real_global_lot = np.array([
            [966883.0219870011, 1935790.3820852807,0],
            [966885.260391943, 1935790.4820035344, 0],
            [966887.4655736537, 1935790.3786974582,0],
            [966889.7761404542, 1935790.3912502257,0],
            [966892.0579714774, 1935790.5105478845,0],
            [966894.4252551483, 1935790.6029213013,0],
            [966896.6911982417, 1935790.7080223535,0],
            [966899.0037262351, 1935790.6771503098,0],
            [966901.3016965914, 1935790.7270317753,0],
            [966903.5713436026, 1935790.7908791858,0],
            [966905.8225832751, 1935790.677703329, 0],
            [966908.1275648307, 1935790.752677677, 0],
            [966910.5437012353, 1935790.9481107865,0],
            [966912.8922681527, 1935790.886026902, 0],
            [966915.1466462176, 1935791.0302732498,0],
            [966917.3596532875, 1935790.9033079676,0],
            [966919.65132297, 1935790.9098238335, 0],
            [966921.9385651791, 1935790.9921590027,0],
            [966924.2494093282, 1935791.039545082, 0],
            [966926.575152908, 1935791.0724159572 ,0],
            [966928.8836540156, 1935791.1432509832,0],
            [966931.2032716349, 1935791.1471147034,0],
            [966933.4685309101, 1935791.3095207657,0],
            [966935.709768572, 1935791.1804188914 ,0],
            [966938.0226256344, 1935791.2623150172,0],
            [966940.3465576388, 1935791.384250695, 0],
            [966942.700705846, 1935791.3838278714, 0],
            [966944.9454949885, 1935791.4096894918,0],
            [966947.2444400941, 1935791.467176213, 0],
            [966949.4651861562, 1935791.3976804705,0],
            [966951.7452383387, 1935791.370944586, 0]
        ])

    def activate(self, points: list, pose: Point) -> None:

        self.current_pose_x, self.current_pose_y, self.heading = pose.x, pose.y, pose.z 

        if points:
            points_np = np.array(points)
            o3d_point_cloud = o3d.geometry.PointCloud()
            o3d_point_cloud.points = o3d.utility.Vector3dVector(points_np[:, :3])

            crop_cloud = self.apply_roi(o3d_point_cloud)

            dbscan_cloud = self.apply_dbscan(crop_cloud)
            mean_cloud = self.mean_dbscan(dbscan_cloud)
            global_mean_cloud = self.transform_local2global(mean_cloud)
            self.canidate_parking_lot(global_mean_cloud)
            # processed_points_np = np.asarray(mean_cloud)
            # header = Header()
            # header.stamp = rospy.Time.now()
            # header.frame_id = "velodyne"
            # cloud_msg = pc2.create_cloud_xyz32(header, processed_points_np.tolist())
            # self.lidar_pub.publish(cloud_msg)
            return np.array(self.closet_value)
        
        else:
            return np.array([])
        
    
    def publish_local_parking_lot(self):
        if self.local_parking_lot is not None:
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "velodyne"
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1),
                     ]
            
            points = [self.local_parking_lot]
            
            pc2_msg = pc2.create_cloud(header, fields, points)
            self.parking_lot_pub.publish(pc2_msg)

    def apply_roi(self, o3d_point_cloud: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        x_min, x_max, y_min, y_max, z_min, z_max = -3, 10, -5, -1, -0.5, 5
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(x_min, y_min, z_min), max_bound=(x_max, y_max, z_max))
        crop_cloud = o3d_point_cloud.crop(bbox)
        return crop_cloud

    def apply_dbscan(self, o3d_point_cloud: o3d.geometry.PointCloud, eps: float = 0.3, min_points: int = 10) -> dict:
        labels = -1
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
        else:
            return clusters

    def mean_dbscan(self, clusters: dict) -> list:
        centroids = []
        for _, points in clusters.items():
            centroid = np.mean(points, axis=0)
            centroids.append(centroid)
        return centroids        
    
    def transform_local2global(self, mean_cloud: list) -> list:
        if self.heading is not None and self.current_pose_x is not None and self.current_pose_y is not None:
            T = [[np.cos(self.heading), -1*np.sin(self.heading), self.current_pose_x], 
                 [np.sin(self.heading),  np.cos(self.heading), self.current_pose_y], 
                 [      0     ,      0       , 1]] 
            
            global_mean_cloud = []
            for obs_x, obs_y, _ in mean_cloud:
                obs_tm = np.dot(T, np.transpose([obs_x + self.gps2lidar, obs_y, 1]))
                gx = obs_tm[0]
                gy = obs_tm[1]
                global_mean_cloud.append([gx, gy])

            return global_mean_cloud
        return []
    
    def transform_global2local(self, global_parking_lot):
        if self.heading is not None and self.current_pose_x is not None and self.current_pose_y is not None:
            global_parking_lot[0] = global_parking_lot[0] - self.current_pose_x
            global_parking_lot[1] = global_parking_lot[1] - self.current_pose_y

            global_parking_lot[0] = global_parking_lot[0] * np.cos(-self.heading) - global_parking_lot[1] * np.sin(-self.heading)
            global_parking_lot[1] = global_parking_lot[0] * np.sin(-self.heading) + global_parking_lot[1] * np.cos(-self.heading)


            return global_parking_lot

        return []

    def transform_global2local_real_global_lot(self, real_global_parking_lot):
        real_global_parking_lot[0] = real_global_parking_lot[0] - self.current_pose_x
        real_global_parking_lot[1] = real_global_parking_lot[1] - self.current_pose_y

        real_global_parking_lot[0] = real_global_parking_lot[0] * np.cos(-self.heading) - real_global_parking_lot[1] * np.sin(-self.heading)
        real_global_parking_lot[1] = real_global_parking_lot[0] * np.sin(-self.heading) + real_global_parking_lot[1] * np.cos(-self.heading)

        return real_global_parking_lot


    def calculate_global_x(self, global_mean_cloud):
        if global_mean_cloud is not None and len(global_mean_cloud) > 0:
            global_mean_cloud_np = np.array(global_mean_cloud)

            if global_mean_cloud_np.ndim == 1:
                global_mean_cloud_np_sorted = np.sort(global_mean_cloud_np)
                global_x_differences = np.diff(global_mean_cloud_np_sorted)

            else:
                sorted_indices = np.argsort(global_mean_cloud_np[:, 0])
                global_mean_cloud_np_sorted = global_mean_cloud_np[sorted_indices]
                global_x_values = global_mean_cloud_np_sorted[:, 0]
                global_x_differences = np.diff(global_x_values)

            return global_x_differences, global_mean_cloud_np_sorted
        
        return np.array([0]), np.array([])

    def canidate_parking_lot(self, global_mean_cloud: list, threshold=4):
        
        global_x_diff, global_mean_cloud_np_sorted = self.calculate_global_x(global_mean_cloud)
        global_parking_lot_candidate = global_x_diff >= threshold

        if sum(global_parking_lot_candidate) >= 1:
            # print("*" * 50)
            true_indices = np.where(global_parking_lot_candidate)[0]
            
            if len(true_indices) >= 2:
                true_indices = true_indices[0]
            indices = int(true_indices)
            
            global_parking_lot = [(global_mean_cloud_np_sorted[indices][0] + global_mean_cloud_np_sorted[indices+1][0]) / 2, 
                           (global_mean_cloud_np_sorted[indices][1] + global_mean_cloud_np_sorted[indices+1][1]) / 2, 0]
            # os.system('clear')
            # print('Global parking_lot: ', global_parking_lot)
            
            self.closet_value = self.real_global_lot[np.abs(self.real_global_lot[:, 0]-global_parking_lot[0]).argmin()]
                
            # print('closet value: ', self.closet_value)

            self.local_parking_lot = self.transform_global2local_real_global_lot(global_parking_lot)
            # print('Local parking_lot: ', self.local_parking_lot)
            # print("*" * 50)

            # self.publish_local_parking_lot()




def main():
    rospy.init_node('parking')
    pp = ProcessedPointCloud()
    rospy.spin()                     

if __name__ == "__main__":
    main()