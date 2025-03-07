import time

import numpy as np
import rospy
import struct

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField, NavSatFix, Imu
from std_msgs.msg import Header
from macaron_06.msg import lidar_info

class ObjectInfo:
    def __init__(self, centroid):
        """information about object

        Args:
            centroid (ndarray): [x, y, z]
        """
        self.centroid = centroid
        self.previous_centroid = centroid
        self.diff = 0
        self.disappear_count = 3

        self.b_dynamic = False

        color = np.random.randint(0, 255, (3, ))
        self.rgb = struct.unpack('I', struct.pack('BBBB', color[2], color[1], color[0], 255))[0]

    def get_differences(self, centroid_matrix):
        """get distance for multiple objects

        Args:
            centroid_matrix (ndarray): [[x, y, z], ...]

        Returns:
            differences (ndarray): [d1, d2, ...]
        """
        diff = np.linalg.norm(centroid_matrix - self.centroid, axis=1)

        return diff

    def update(self, centroid, diff):
        """update centroid and diff

        Args:
            centroid (ndarray): [x, y, z]
            diff (float): difference from previous object
        """
        self.previous_centroid = self.centroid
        self.centroid = centroid
        self.diff = diff

    def convert_to_tracking_point(self):
        """convert to point with tracking color

        Returns:
            point (list): [x, y, z, color]
        """
        return [*(self.centroid), self.rgb]
    
    def convert_to_dynamic_point(self):
        """convert to point with dynamic info

        Returns:
            point (list): [x, y, z, 0/1]
        """

        return [*(self.centroid), int(self.b_dynamic)]
    

class Tracker:
    def __init__(self, track_threshold, dynamic_threshold, **kwargs):
        self.object_list = []
        self.track_threshold = track_threshold
        self.dynamic_threshold = dynamic_threshold
        self.previous_time = rospy.Time.now()
        self.points = []

        # self.topic_sub_person = kwargs['topic_sub_person'] if ('topic_sub_person' in kwargs) else '/cluster'
        # self.sub_person = rospy.Subscriber(self.topic_sub_person, data_class=lidar_info, callback=self.cluster_callback)

    def cluster_callback(self, msg):
        """centroid callback, execute track function

        Args:
            msg (PointCloud2): lidar_info.msg
        """
        
        pcd = []
        for point in point_cloud2.read_points(msg.data, field_names=("x", "y", "z", "intensity")):
            pcd.append(point)
        pcd = np.array(pcd)[:, :3]
        
        if pcd.shape[0] == 0:
            return

        cluster_indices = list(msg.clusters)
        cone_indices = list(msg.person)

        if len(cluster_indices) == 0 or len(cone_indices) == 0:
            return

        clusters = []
        count = 0
        
        for indice_size in msg.clusterSize:
            indice = cluster_indices[count : count+indice_size]
            count += indice_size

            clusters.append(pcd[indice, :])

        people = [clusters[i] for i in cone_indices]
        
        centroids = []
        for person in people:
            centroids.append(np.mean(person, axis=0))
        centroids = np.array(centroids)

        # print(msg.header.stamp)

        deltatime = msg.header.stamp - self.previous_time
        deltatime = deltatime.secs + deltatime.nsecs * 1e-9
        self.previous_time = msg.header.stamp
        
        self.track(centroids, deltatime)

    def publish_dynamic(self):
        """publish dynamic
        dynamic: 1
        static: 0
        """
        
        points = []
        for object in self.object_list:
            points.append(object.convert_to_dynamic_point())

        self.points = points

        

    def is_same_object(self, diff, previous_diff, threshold, deltatime):
        """

        Args:
            diff (float): difference
            previous_diff (float): n-1 vs n-2 difference
            threshold (float): threshold
            deltatime (float): deltatime

        Returns:
            result: bool
        """
        return diff < threshold + previous_diff * 0.2
    
    def is_dynamic_object(self, centroid, diff, object, threshold, deltatime, velocity=None):
        """

        Args:
            diff (float): difference
            previous_diff (float): n-1 vs n-2 difference
            threshold (float): threshold
            deltatime (float): deltatime

        Returns:
            result: bool
        """
            
        previous_diff = object.diff
        previous_centroid = object.centroid
        pprevious_centroid = object.previous_centroid

        diff_xyz = centroid - previous_centroid
        diff_xyz /= deltatime
        previous_diff_xyz = previous_centroid - pprevious_centroid

        # print(deltatime)
        return abs(diff_xyz[1]) > threshold

    def track(self, centroids, deltatime):
        """track and detect dynamic object

        Args:
            centroids (ndarray): centroids matrix
            deltatime (float): deltatime
        """
        delete_index = [] # index of object which is disappeared
        for i, object in enumerate(self.object_list):
            if centroids.shape[0] == 0:
                for j, o in enumerate(self.object_list[i:]):
                    o.disappear_count -= 1
                    if o.disappear_count <= 0:
                        delete_index.append(i + j)

                break

            diff = object.get_differences(centroids)
            min_index = np.argmin(diff)
            if self.is_same_object(diff[min_index], object.diff, self.track_threshold, deltatime):
                object.b_dynamic = self.is_dynamic_object(centroids[min_index], diff[min_index], object, self.dynamic_threshold, deltatime)
                self.object_list[i].update(centroids[min_index], diff[min_index])
                centroids = np.delete(centroids, min_index, axis=0)
            else:
                object.disappear_count -= 1
                if object.disappear_count <= 0:
                    delete_index.append(i)

        for i in reversed(sorted(delete_index)):
            self.object_list.pop(i)

        for centroid in centroids:
            self.object_list.append(ObjectInfo(centroid))

        self.publish_dynamic()

        return self.points

    def get_dynamic(self):
        return self.points

    def detect_dynamic(self, msg):
        """centroid callback, execute track function

        Args:
            msg (PointCloud2): lidar_info.msg
        """
        
        pcd = []
        for point in point_cloud2.read_points(msg.data, field_names=("x", "y", "z", "intensity")):
            pcd.append(point)
        pcd = np.array(pcd)[:, :3]
        
        if pcd.shape[0] == 0:
            return

        cluster_indices = list(msg.clusters)
        cone_indices = list(msg.person)

        if len(cluster_indices) == 0 or len(cone_indices) == 0:
            return

        clusters = []
        count = 0
        
        for indice_size in msg.clusterSize:
            indice = cluster_indices[count : count+indice_size]
            count += indice_size

            clusters.append(pcd[indice, :])

        people = [clusters[i] for i in cone_indices]
        
        centroids = []
        for person in people:
            centroids.append(np.mean(person, axis=0))
        centroids = np.array(centroids)

        deltatime = msg.header.stamp - self.previous_time
        deltatime = deltatime.secs + deltatime.nsecs * 1e-9
        self.previous_time = msg.header.stamp
        
        self.dynamic_tracking(centroids, deltatime)

    def dynamic_tracking(self, centroids, deltatime):
        """track and detect dynamic object

        Args:
            centroids (ndarray): centroids matrix
            deltatime (float): deltatime
        """
        delete_index = [] # index of object which is disappeared
        for i, object in enumerate(self.object_list):
            if centroids.shape[0] == 0:
                for j, o in enumerate(self.object_list[i:]):
                    o.disappear_count -= 1
                    if o.disappear_count <= 0:
                        delete_index.append(i + j)

                break

            diff = object.get_differences(centroids)
            min_index = np.argmin(diff)
            if self.is_same_object(diff[min_index], object.diff, self.track_threshold, deltatime):
                object.b_dynamic = self.is_dynamic_object(centroids[min_index], diff[min_index], object, self.dynamic_threshold, deltatime)
                self.object_list[i].update(centroids[min_index], diff[min_index])
                centroids = np.delete(centroids, min_index, axis=0)
            else:
                object.disappear_count -= 1
                if object.disappear_count <= 0:
                    delete_index.append(i)

        for i in reversed(sorted(delete_index)):
            self.object_list.pop(i)

        for centroid in centroids:
            self.object_list.append(ObjectInfo(centroid))

        self.publish_dynamic()
        # return self.points


if __name__ == '__main__':
    rospy.init_node("tracker")
   
    Tracker(track_threshold=0.5, dynamic_threshold=0.5, publish_tracking_topic=True)
   
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
