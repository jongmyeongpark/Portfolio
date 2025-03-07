#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
import numpy as np
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
import utm
import struct
import time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32, Header
from morai_msgs.msg import GPSMessage
from geometry_msgs.msg import Point

class ObjectInfo:
    def __init__(self, centroid, centroid_global):
        self.centroid = centroid
        self.centroid_global = centroid_global
        self.diff = 0
        self.disappear_count = 3
        self.b_dynamic = False
        self.last_update_time = time.time()  

        color = np.random.randint(0, 255, (3, ))
        self.rgb = struct.unpack('I', struct.pack('BBBB', color[2], color[1], color[0], 255))[0]

    def get_differences(self, centroid_matrix, get_global=False):
        if get_global and self.centroid_global is not None:
            if centroid_matrix.size == 0:
                return np.array([])  # 빈 배열 반환
            diff = np.linalg.norm(centroid_matrix - self.centroid_global, axis=1)
        else:
            if centroid_matrix.size == 0:
                return np.array([])  # 빈 배열 반환
            diff = np.linalg.norm(centroid_matrix - self.centroid, axis=1)
        return diff

    def update(self, centroid, centroid_global, diff):
        self.centroid = centroid
        self.centroid_global = centroid_global
        self.diff = diff
        self.last_update_time = time.time()  

    def convert_to_tracking_point(self):
        return [*(self.centroid), self.rgb]
    
    def convert_to_dynamic_point(self):
        return [*(self.centroid), int(self.b_dynamic)]

class ProcessedPointCloud:
    def __init__(self):
        # self.imu_sub = rospy.Subscriber('/current_pose', Point, self.current_pose_callback, queue_size=1)
        # self.gps_sub = rospy.Subscriber('/gps', GPSMessage, self.gps_callback, queue_size=1)
        # self.lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback, queue_size=1)

        self.lidar_pub = rospy.Publisher('/processed_pointcloud', PointCloud2, queue_size=1)
        self.tracking_pub = rospy.Publisher('/tracking', PointCloud2, queue_size=1)
        self.dynamic_pub = rospy.Publisher('/dynamic', PointCloud2, queue_size=1)
        self.dynamic_global_pub = rospy.Publisher('/dynamic_global', PointCloud2, queue_size=1)


        self.gps_position = None
        self.heading = None
        self.object_list = []
        self.track_threshold = 1
        self.dynamic_threshold = 1.5
        self.stale_object_timeout = 1.0  

        self.current_pose_x = None
        self.currnet_pose_y = None

    def current_pose_callback(self, current_pose_msg: Point) -> None:
        self.current_pose_x = current_pose_msg.x
        self.current_pose_y = current_pose_msg.y
        self.heading = current_pose_msg.z

    def gps_callback(self, gps_msg: GPSMessage) -> None:
        easting, northing, _, _ = utm.from_latlon(gps_msg.latitude, gps_msg.longitude)
        self.gps_position = (easting, northing)

    def lidar_callback(self, lidar_msg: PointCloud2) -> None:
        points = list(map(lambda x: list(x), pc2.read_points(lidar_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)))
        
        if points:
            points_np = np.array(points)
            o3d_point_cloud = o3d.geometry.PointCloud()
            o3d_point_cloud.points = o3d.utility.Vector3dVector(points_np[:, :3])

            crop_cloud = self.apply_roi(o3d_point_cloud)

            # 정적/동적 판단
            dbscan_cloud = self.apply_dbscan(crop_cloud)
            mean_cloud = self.mean_dbscan(dbscan_cloud)
            global_mean_cloud = self.transform_local2global(mean_cloud)

            self.track_objects(mean_cloud, global_mean_cloud)

            # processed_points_np = np.asarray(crop_cloud.points)
            # header = lidar_msg.header
            # cloud_msg = pc2.create_cloud_xyz32(header, processed_points_np.tolist())
            # self.lidar_pub.publish(cloud_msg)

    def is_activated(self, points: list, pose: Point) -> None:
        
        self.gps_position = [pose.x, pose.y]
        self.heading = pose.z
        if points:
            points_np = np.array(points)
            o3d_point_cloud = o3d.geometry.PointCloud()
            o3d_point_cloud.points = o3d.utility.Vector3dVector(points_np[:, :3])

            crop_cloud = self.apply_roi(o3d_point_cloud)

            # 정적/동적 판단
            dbscan_cloud = self.apply_dbscan(crop_cloud)
            mean_cloud = self.mean_dbscan(dbscan_cloud)
            global_mean_cloud = self.transform_local2global(mean_cloud)
            return self.track_objects(mean_cloud, global_mean_cloud)

            # processed_points_np = np.asarray(crop_cloud.points)
            # header = Header()
            # header.stamp = rospy.Time.now()
            # header.frame_id = "macaron"
            # cloud_msg = pc2.create_cloud_xyz32(header, processed_points_np.tolist())
            # self.lidar_pub.publish(cloud_msg)

    def apply_roi(self, o3d_point_cloud: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        x_min, x_max, y_min, y_max, z_min, z_max = 0.4, 20, -6, 6, -1.0, 0.5
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(x_min, y_min, z_min), max_bound=(x_max, y_max, z_max))
        crop_cloud = o3d_point_cloud.crop(bbox)
        return crop_cloud

    def apply_dbscan(self, o3d_point_cloud: o3d.geometry.PointCloud, eps: float = 1, min_points: int = 10) -> dict:
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

    def transform_local2global(self, centroids: list):
        if centroids and self.gps_position and self.heading is not None:
            centroids = np.array(centroids)
            R_matrix = np.array([
                [np.cos(self.heading), -np.sin(self.heading), 0, 0],
                [np.sin(self.heading), np.cos(self.heading), 0, 0],
                [0, 0, 1, 0],
                [self.gps_position[0], self.gps_position[1], 0, 0]
            ])

            centroids_homogeneous = np.hstack([centroids, np.ones((centroids.shape[0], 1))])
            centroids_global_homogeneous = (R_matrix @ centroids_homogeneous.T).T
            centroids_global = centroids_global_homogeneous[:, :3]

            return centroids_global
        return []

    def tf2tm(self, dynamic_points:list):
        if self.heading is not None and self.gps_position is not None:
            T = [[np.cos(self.heading), -1*np.sin(self.heading), self.gps_position[0]], 
                 [np.sin(self.heading),    np.cos(self.heading), self.gps_position[1]], 
                 [          0         ,             0          ,           1         ]] 
            
            global_dynamic_points = []
            for obs_x, obs_y, _, _, index in dynamic_points:
                obs_tm = np.dot(T, np.transpose([obs_x, obs_y, 1]))
                gx = obs_tm[0]
                gy = obs_tm[1]
                global_dynamic_points.append([gx, gy, index])

            return global_dynamic_points
        return []

    def track_objects(self, centroids, centroids_global):
        delete_index = []

        # 현재 시간
        current_time = time.time()

        for i, obj in enumerate(self.object_list):
            if len(centroids_global) == 0:
                continue

            diff = obj.get_differences(centroids_global, get_global=True)

            if len(diff) == 0:
                continue

            min_index = np.argmin(diff)
            if self.is_same_object(diff[min_index], obj.diff, self.track_threshold):
                obj.b_dynamic = self.is_dynamic_object(diff[min_index], obj.diff, self.dynamic_threshold)
                self.object_list[i].update(centroids[min_index], centroids_global[min_index], diff[min_index])
                centroids = np.delete(centroids, min_index, axis=0)
                centroids_global = np.delete(centroids_global, min_index, axis=0)
            else:
                obj.disappear_count -= 1
                if obj.disappear_count <= 0:
                    delete_index.append(i)

        for i in delete_index[::-1]:
            self.object_list.pop(i)

        for centroid, centroid_global in zip(centroids, centroids_global):
            self.object_list.append(ObjectInfo(centroid, centroid_global))

        self.object_list = [obj for obj in self.object_list if (current_time - obj.last_update_time) < self.stale_object_timeout]
        # self.publish_tracking()
        # self.publish_dynamic()
        # self.publish_dynamic_global()

        return self.dynamic_global()
    


    def is_same_object(self, diff, previous_diff, threshold):
        return diff < threshold + previous_diff * 0.2

    def is_dynamic_object(self, diff, previous_diff, threshold):
        return diff > threshold

    def publish_tracking(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "velodyne"
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgb', 16, PointField.UINT32, 1)]
        points = [obj.convert_to_tracking_point() for obj in self.object_list]
        pc2_msg = pc2.create_cloud(header, fields, points)
        self.tracking_pub.publish(pc2_msg)

    def publish_dynamic(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "velodyne"
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('intensity', 16, PointField.UINT32, 1),
                  PointField('index', 20, PointField.UINT32, 1)]
        
        points = [obj.convert_to_dynamic_point() + [index] for index, obj in enumerate(self.object_list)]
        pc2_msg = pc2.create_cloud(header, fields, points)
        self.dynamic_pub.publish(pc2_msg)

    def publish_dynamic_global(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "velodyne"
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('index', 8, PointField.UINT32, 1)]
        points = [obj.convert_to_dynamic_point() + [index] for index, obj in enumerate(self.object_list)]
        global_points = self.tf2tm(points)

        # print('current_pose_x, current_pose_y: ', self.current_pose_x, self.current_pose_y)
        print('global_points: ', global_points)
        pc2_msg = pc2.create_cloud(header, fields, global_points)
        self.dynamic_global_pub.publish(pc2_msg)

    def dynamic_global(self):
        points = [obj.convert_to_dynamic_point() + [index] for index, obj in enumerate(self.object_list)]
        return self.tf2tm(points)

def main():
    rospy.init_node("lidar_processing", anonymous=True)
    pp = ProcessedPointCloud()
    rospy.spin()

if __name__ == "__main__":
    main()
