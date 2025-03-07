#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
import time
import numpy as np

from math import *
from std_msgs.msg import Header
from sklearn.cluster import DBSCAN
from numpy import array,transpose,dot,cross
from std_msgs.msg import Float64, Bool, String, Int32
from sensor_msgs.msg import LaserScan, NavSatFix, PointCloud, Imu, PointCloud2, ChannelFloat32, PointField
from geometry_msgs.msg import Point, Point32, Quaternion
import sensor_msgs.point_cloud2 as pc2
import random

from macaron_6.msg import erp_read

from location import gps_imu_fusion
from lidar_module import lidar_module

epsilon = 0.1  # 입실론 값 0.4
min_points = 5  # 한 군집을 만들 최소 포인트 개수 4
epsilon2 = 1
min_points2 = 2
error = 0

# Parameters
k = 0.15  # look forward gain
Lfc = 2.0  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time tick
WB = 1.03  # [m] wheelbase of vehicle
MAX_STEER = 2000
MIN_STEER = -2000


class Visual:
    def __init__(self):
        self.goal_pub = rospy.Publisher("/goal_point", Point, queue_size=1)

    def pub_goal(self, x, y):
        gp = Point()
        gp.x = x
        gp.y = y
        gp.z = 0
        self.goal_pub.publish(gp)

class LeftTurn:
    def __init__(self):
        # 구독자 선언
        self.lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback, queue_size=1)

        # 발행자 선언
        self.lidar_left_pub = rospy.Publisher('/left_filtered_points', PointCloud2, queue_size=1)
        self.left_turn_pub = rospy.Publisher('/left_turn_steer', Int32, queue_size=1)
        self.left_turn_start_sub = rospy.Subscriber('/left_turn_start', Bool, self.left_turn_start_callback, queue_size=1)

        self.obj_pointcloud_before_check = PointCloud()
        self.obj_pointcloud = PointCloud()
        self.obs_check_point = Point()

        self.lidar = None
        self.lidar_flag = False
        self.lidar_module = lidar_module()
        self.left_turn_flag = False
        self.last_steer = -1000
        self.left_turn_start = False


    def lidar_callback(self, lidar):
        self.lidar = lidar
        self.lidar_flag = True

    def left_turn_start_callback(self, msg):
        self.left_turn_start = msg.data
        if self.left_turn_start is False:
            self.last_steer = -1000
        
    def left_update(self, lidar):

        if self.lidar_flag is False:
            return
        # 객체 선언
        cloud_msg = PointCloud2()
        obs_xyz = np.array(list(map(lambda x: list(x), pc2.read_points(lidar, field_names=("x", "y", "z"),
                                                                       skip_nans=True))))
        header = Header()
        points = []

        header.frame_id = 'velodyne'

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        obs_xyz = obs_xyz[np.logical_and(0 < obs_xyz[:, 2], obs_xyz[:, 2] < 15) & #(obs_xyz[:, 0] > 0) &
                          np.logical_and(-10 < obs_xyz[:, 0], obs_xyz[:, 0] < 5 ) & np.logical_and(0 < obs_xyz[:, 1], obs_xyz[:, 1] < 15)]

        points = obs_xyz[:, :2]

        model = DBSCAN(eps=epsilon, min_samples=min_points)
        DB_Data = np.array(points, dtype=object)

        labels = model.fit_predict(DB_Data)

        k = 0
        no_noise_model = []
        no_noise_label = []

        for i in points:
            if labels[k] != -1:
                no_noise_model.append([i[0], i[1], 0])
                no_noise_label.append(labels[k])
            k += 1

        no_noise_model = np.array(no_noise_model)

        clusters = np.unique(no_noise_label)

        cluster_means = []
        for cluster in clusters:
            cluster_points = no_noise_model[no_noise_label == cluster]
            cluster_mean = np.mean(cluster_points, axis=0)
            cluster_means.append(cluster_mean)
        # print('cluster_means : ', len(cluster_means)) #여기부터 시작!!
        # print('cluster : ',type(cluster_means))

        points2 = np.array(cluster_means)
        # print('points2 : ', points2)
        model2 = DBSCAN(eps=epsilon2, min_samples=min_points2)
        DB_Data2 = np.array(points2, dtype=object)

        labels2 = model2.fit_predict(DB_Data2)

        k = 0
        no_noise_model2 = []
        no_noise_label2 = []

        for i in points2:
            if labels2[k] != -1:
                no_noise_model2.append([i[0], i[1], 0])
                no_noise_label2.append(labels2[k])
            k += 1

        no_noise_model2 = np.array(no_noise_model2)

        clusters2 = np.unique(no_noise_label2)
        # print(type(no_noise_model2))
        cluster_means2 = []
        for cluster2 in clusters2:
            cluster_points2 = no_noise_model2[no_noise_label2 == cluster2]
            cluster_mean2 = np.mean(cluster_points2, axis=0)
            if 5 < cluster_mean2[1] <= 14:
                cluster_means2.append(cluster_mean2)

 
        print('cluster_means2 : ', cluster_means2)

        # 일정한 거리 12.7m로 pid_p제어 하기
        # 마지막으로 해야 할 것은 아래에 만든 pid(left, right)의 파라미터값 수정은 어떤 기준으로 하는가?
        # erp에 어떻게 적용할 것인가?
        # 만약 포인트가 안잡히면 차량이 많이 흔들릴 수 있음으로 아무런 점도 잡히지 않는 순간은 이전 steer 저장하는 방식으로 예외처리

        # if 12.5 <= cluster_means2[0][1] <= 13:
        #     if cluster_means2[0][1] < 12.7:
        #         short_error = 12.7 - cluster_mean2[0][1]
        #         pid_right = 2*short_error
        #         return pid_right
        #     else:
        #         long_error = cluster_mean2[0][1] - 12.7
        #         pid_left = 2*long_error
        #         return pid_left


        if len(cluster_means2) == 0:
            steer = self.last_steer 

        else:
            theta = pi/2
            local_coord = [-cluster_means2[0][1], cluster_means2[0][0]]
            r = 11
            follow_point = [r*cos(theta) - local_coord[0], r*sin(theta) + local_coord[1]]               
            steer = self.get_steer_state(follow_point)
            self.last_steer = steer

        print('steer : ', steer)

        cloud_msg = pc2.create_cloud(header=header, fields=fields, points=cluster_means2)

        self.lidar_left_pub.publish(cloud_msg)
        self.left_turn_pub.publish(steer)

    def pure_pursuit_steer_control(state, goal, g_dis):
        tx = goal[0]
        ty = goal[1]
        v = Visual()
        v.pub_goal(tx, ty)

        alpha = atan2(ty, tx)
        delta = atan2(2.0 * WB * sin(alpha) / g_dis, 1.0)  # pure_pursuit 공식
        return delta

    def get_steer_state(self, goal):
        g_dis = sqrt((goal[0]) ** 2 + (goal[1]) ** 2)
        steer = self.pure_pursuit_steer_control(goal, g_dis)

        target_steer = np.rad2deg(steer) * 71
        if target_steer > MAX_STEER:
            target_steer = MAX_STEER
        if target_steer < MIN_STEER:
            target_steer = MIN_STEER

        return int(1.5 * -target_steer)
    

def main():
    rospy.init_node('left_turn')
    Data = LeftTurn()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        Data.left_update(Data.lidar)
        rate.sleep()


if __name__ == '__main__':
    main()
