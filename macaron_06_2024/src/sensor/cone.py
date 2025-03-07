#!/usr/bin/python3

import time

import numpy as np
import rospy

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from macaron_06.msg import lidar_info


def cone_callback(msg):
    pcd = []
    for point in point_cloud2.read_points(msg.data, field_names=("x", "y", "z", "intensity")):
        pcd.append(point)
    pcd = np.array(pcd)[:, :3]

    
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
    cones = np.array(cones)

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "velodyne"
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        ]
    
    print(cones.shape)
    points = cones.tolist()

    pc2 = point_cloud2.create_cloud(header, fields, points)

    cone_pub.publish(pc2)


cone_pub = rospy.Publisher("/cones", PointCloud2, queue_size=10)


if __name__ == '__main__':
    rospy.init_node("cone")

    cone_sub = rospy.Subscriber("/cluster", data_class=lidar_info, callback=cone_callback)
   
   
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()