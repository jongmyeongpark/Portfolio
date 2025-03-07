#!/usr/bin/env python3

import numpy as np
import rospy
import matplotlib.pyplot as plt

from macaron_06.msg import lidar_info
from sensor_msgs import point_cloud2

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from find_parking_area import find_parking_area, draw_rectangle

class ParkingDetector:
    def __init__(self) -> None:

        self.sub = rospy.Subscriber("/cluster", data_class=lidar_info, callback=self.cluster_callback)

        self.rect_pub = rospy.Publisher("/rect", PointCloud2, queue_size=1)

        self.size_x = (-4.0, 4.0)
        self.size_y = (-5.0, 0.0)

        self.obstacles = []
        self.is_finded = False

    def cluster_callback(self, msg):
        
        pcd = []
        for point in point_cloud2.read_points(msg.data, field_names=("x", "y", "z", "intensity")):
            pcd.append(point)
        pcd = np.array(pcd)[:, :2]
        
        if pcd.shape[0] == 0:
            return

        cluster_indices = list(msg.clusters)
        cone_indices = list(msg.cones)

        if len(cluster_indices) == 0 or len(cone_indices) == 0:
            return

        clusters = []
        count = 0
        
        for indice_size in msg.clusterSize:
            indice = cluster_indices[count : count+indice_size]
            count += indice_size

            clusters.append(pcd[indice, :])

        cones = [clusters[i] for i in cone_indices]
        cones = [np.mean(cone, axis=0) for cone in cones]

        p = []
        for cone in cones:
            if self.size_x[0] <= cone[0] <= self.size_x[1] and self.size_y[0] <= cone[1] <= self.size_y[1]:
                p.append(cone)
        self.obstacles = p
        self.obstacles = np.array(self.obstacles)

        self.is_finded, rect = find_parking_area(self.obstacles, (4.85, 2.75 / 2), 0.5, 10)

        if self.is_finded:

            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "macaron"
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                ]

            rect_3d = np.hstack([rect, np.zeros((rect.shape[0], 1))])
            points = rect_3d.tolist()

            pc2 = point_cloud2.create_cloud(header, fields, points)

            self.rect_pub.publish(pc2)

        # print(is_finded)

if __name__ == '__main__':
    rospy.init_node("parking")
    parking_detector = ParkingDetector()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
