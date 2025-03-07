#!/usr/bin/env python3
import os, sys
from math import *

import numpy as np
import rospy
import time

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/path_planning")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/src/sensor")

from lidar_lane_detection_test import LidarLineDetection
from path_planning_tracking_dwa_PP import Path_Tracking_DWA
from pure_pursuit_added_stanley import Path_Tracking_PP_Stanley
from mission_traffic_light_fmtc import mission_traffic
from left_turn import LeftTurn
from std_msgs.msg import Float32, Int32

from pyproj import Proj, transform
from sensor_msgs.msg import NavSatFix, Imu
from sklearn.cluster import DBSCAN
from collections import defaultdict
from tf.transformations import euler_from_quaternion

# FINISH_POINT_XY = [935552.700315463, 1915703.9975436954]


class MissionTunnel:
    def __init__(self, global_path_npy):
        self.lane_steer_sub = rospy.Subscriber("lane_q", Float32, self.lane_callback, queue_size=1)
        self.sub_tm = rospy.Subscriber("ublox_gps/fix", NavSatFix, self.tm, queue_size=1)
        self.left_turn_sub = rospy.Subscriber('/left_turn_steer', Int32, self.left_turn_steer, queue_size=1)
        self.sub_imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback, queue_size=1)
        self.proj_UTMK = Proj(init='epsg:5179')
        self.proj_WGS84 = Proj(init='epsg:4326')

        self.lane_detection = None
        self.dwa_PT = Path_Tracking_DWA(global_path_npy)
        self.gp_PT = Path_Tracking_PP_Stanley(global_path_npy)
        self.artificial_global_path = None
        self.calculate_flag = False
        self.finish_flag = False

        self.static_obs_time_flag = False
        self.static_time = 0.0
        self.record_time = None

        self.mission_end_flag = False
        self.static_end_flag = False
        self.shading_area_step = 0

        self.pos = [0, 0]
        self.speed = 0.0
        self.steer = 0.0
        self.DBSCAN = DBSCAN(eps=0.3, min_samples=5)
        # self.lane_steer = 0.0
        self.left_steer = 0.0
        self.yaw = 0.0
        self.record_yaw = None

        self.mission_traffic = mission_traffic()
        self.last_traffic_flag = 0
        self.test_time = 999999

    def lane_callback(self, msg):
        self.lane_steer = int(msg.data * -800)
        # print('crlnet_steer: ', self.lane_steer)

    def left_turn_steer(self, msg):
        self.left_steer = msg.data

    def imu_callback(self, imu):
        self.sub_imu = imu.orientation
        orientation_list = [self.sub_imu.x, self.sub_imu.y, self.sub_imu.z, self.sub_imu.w]
        _, _, self.yaw = euler_from_quaternion(orientation_list)

    def tm(self, fix):
        lon = fix.longitude
        lat = fix.latitude
        x, y = transform(self.proj_WGS84, self.proj_UTMK, lon, lat)
        self.pos = [x, y]
        # print(self.pos)

    def prepare_calculating(self):
        if self.calculate_flag is False:
            self.calculate_flag = True
            self.lane_detection = LidarLineDetection()

        if self.lane_detection.lidar is not None:
            return True
        else:
            return False

    # noinspection PyMethodMayBeStatic
    def distance(self, point1, point2):
        return sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def run(self, pose, heading, obs):
        # print('step: ', self.shading_area_step)
        # ###############################
        # self.shading_area_step = 1
        # if self.record_time is None:
        #     self.record_time = time.time()
        # ###############################
        self.shading_area_step = 2
        
        if self.shading_area_step == 0:
            obs_xy = []
            if len(obs) > 0:
                obs_xy = np.array(obs, dtype=float)
                labels = np.array(self.DBSCAN.fit_predict(obs))

                mask = labels != -1
                filtered_point = np.hstack((obs_xy[mask], np.zeros((mask.sum(), 1))))[:, :2]
                filtered_label = labels[mask]

                label_points = defaultdict(list)
                for l, p in zip(filtered_label, filtered_point):
                    label_points[l].append(p)
                obs_xy = np.array(list(map(lambda k: np.mean(label_points.get(k), axis=0), label_points)))
                obs_xy.reshape(1, -1)

            if self.static_end_flag is False:
                if len(obs_xy) > 0:
                    for i in range(len(obs_xy)):
                        if sqrt((obs_xy[i][0] - pose[0]) ** 2 + (obs_xy[i][1] - pose[1]) ** 2) < 3:
                            self.lane_detection.static_obs_flag = True
                            self.static_time = time.time()
                            self.static_end_flag = True
                            break

            # if self.lane_detection.static_obs_flag is True:
            #     self.static_time = time.time()

            if self.lane_detection.static_obs_flag is True and time.time() - self.static_time > 5 and len(
                self.lane_detection.obj_pointcloud.points) > 10:
                self.speed = 0
                self.steer = 0
                self.mission_end_flag = True
                return self.speed, self.steer

            # print('obs_len: ', len(self.lane_detection.obj_pointcloud.points))
            if self.mission_end_flag is True:
                if self.record_time is None:
                    self.record_time = time.time()
                self.lane_detection.static_obs_flag = False

            # print('gps distance : ', self.distance(self.pos, FINISH_POINT_XY))
            # if self.distance(self.pos, FINISH_POINT_XY) < 10:
            #     self.finish_flag = True
            #     speed = 60
            #     steer = 0

            self.speed = 60
            self.lane_detection.lidar_lane_detection(self.lane_detection.lidar)
            self.artificial_global_path = self.lane_detection.path
            self.steer = 1.3 * self.dwa_PT.gps_tracking_tunnel(pose, heading, obs, path=self.artificial_global_path)

            if self.record_time is not None:
                if time.time() - self.record_time > 7:
                    self.shading_area_step = 1
                    self.record_time = time.time()

        elif self.shading_area_step == 1:       # 차선 변경
            self.speed = 70
            self.lane_detection.lidar_lane_detection(self.lane_detection.lidar)
            self.artificial_global_path = self.lane_detection.path

            if 0 <= time.time() - self.record_time < 3:
                self.steer = self.dwa_PT.gps_tracking_tunnel(pose, heading, obs, path=self.artificial_global_path)  # self.lane_steer
            elif 3 <= time.time() - self.record_time < 5.5:
                self.steer = -800
            elif 5.5 <= time.time() - self.record_time < 8.5:
                self.steer = 800
            else:
                self.steer = self.dwa_PT.gps_tracking_tunnel(pose, heading, obs, path=self.artificial_global_path)  # self.lane_steer
                self.shading_area_step = 2
                self.record_time = -10
                self.test_time = time.time()

        elif self.shading_area_step == 2:       # 정지선 위를 지난 후 좌회전
            ###### lidar lane #####
            self.lane_detection.lidar_lane_detection(self.lane_detection.lidar)
            self.artificial_global_path = self.lane_detection.path
            #######################
            
            self.speed = self.mission_traffic.run()
            # self.speed = 70
            if self.last_traffic_flag == 2:
                self.steer = 0
            else:
                self.steer = 0.8 * self.dwa_PT.gps_tracking_tunnel(pose, heading, obs, path=self.artificial_global_path) + 200  # self.lane_steer
            
            # if self.mission_traffic.traffic_flag == 2:
            #     self.speed = self.mission_traffic.run()
            #     self.steer = self.dwa_PT.gps_tracking_tunnel(pose, heading, obs, path=self.artificial_global_path)  # self.lane_steer
            # elif self.mission_traffic.traffic_flag == 3:
            #     self.speed = self.mission_traffic.run()
            # if time.time() - self.test_time > 5:
            #     self.mission_traffic.traffic_flag = 3
            
            if self.last_traffic_flag == 2 and self.mission_traffic.traffic_flag == 3:
                self.record_time = time.time()
                self.record_yaw = self.yaw
                if self.record_yaw < 0:
                    self.record_yaw += 2 * pi
                
                # if 0 <= time.time() - self.record_time < 3:
                #     self.steer = 0
                # elif 3 <= time.time() - self.record_time < 8:
                #     self.steer = -300
                # elif 8 <= time.time() - self.record_time < 10:
                #     self.steer = 0
                # else:
                #     self.steer = self.dwa_PT.gps_tracking_tunnel(pose, heading, obs, path=self.artificial_global_path)  # self.lane_steer
            if self.record_yaw is not None:
                # if self.yaw < 0:
                #     self.yaw += 2 * pi
                
                # if self.yaw - self.record_yaw <= 0:
                #     self.yaw += 2 * pi

                # if self.yaw - self.record_yaw <= 0.5 * pi:
                if time.time() - self.record_time < 3:
                    self.steer = 0
                else:
                    self.steer = 1.2 * self.left_steer
                # else:
                    # self.steer = self.dwa_PT.gps_tracking_tunnel(pose, heading, obs, path=self.artificial_global_path)

            # print('lane_steer: ', self.lane_steer)
            self.last_traffic_flag = self.mission_traffic.traffic_flag
        else:
            pass

        return self.speed, self.steer
