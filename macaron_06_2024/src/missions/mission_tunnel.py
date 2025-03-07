#!/usr/bin/env python3
#-*-coding:utf-8-*-

import rospy
import os
import numpy as np
from math import sqrt

from std_msgs.msg import Bool, Header, ColorRGBA
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3, Point
from macaron_06.msg import lane_info

# from sensor.Dead_reckoning import Dead_reckoning_class
from sensor.lane_detection_research_from_hong.lidar_lane_detection import LidarLaneDetection

class MissionTunnel():
    def __init__(self):

        self.lane_info_pub = rospy.Publisher('/lane_info', lane_info, queue_size=1)
        self.lane_center_pub = rospy.Publisher('/lane_center_line', PointCloud, queue_size=1)
        self.dynamic_stop_pub=rospy.Publisher('/stop', Bool, queue_size=1)
        self.center_final_vizpath_pub = rospy.Publisher('/center_final_vizpath', Marker, queue_size=1)

        # self.Dead_reckoning = Dead_reckoning_class()
        self.lane_detection = LidarLaneDetection()

        self.lidar_lane_points = np.array([])
        self.lane_width_offset = 3.2
        self.cone_person_division_height = 0.5

        self.prev_stop_time = rospy.Time.now().to_sec()
        self.centroid_detect_time = rospy.Time.now().to_sec()
        self.stop_count = 0
        self.need2stop = False

        self.lidar_gps_offset = 1.03
        self.centroid_obs = [100, 0 ,0]
        self.dynamic_stop_count = 0
        self.dynamic_detected = False
        self.avoid_obs = False

        self.tunnel_left_dis = 0
        self.tunnel_right_dis = 0

    def center_visualize_path(self, cx, cy):
        rviz_msg_path=Marker(
            header=Header(frame_id='velodyne', stamp=rospy.get_rostime()),
            ns="final_center_path",
            id=777,
            type=Marker.LINE_STRIP,
            lifetime=rospy.Duration(0.5),
            action=Marker.ADD,
            scale=Vector3(0.1,0.0,0.0),
            color=ColorRGBA(r=1.0,g=1.0,b=1.0,a=0.8)
        )
        
        for x, y in zip(cx, cy):
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.1
            rviz_msg_path.points.append(p)
        self.center_final_vizpath_pub.publish(rviz_msg_path)

    def det_closest_point(self, points):
        min_distance = float('inf')
        closest_index = -1

        for index, (x, y, z, intensity) in enumerate(points):
            if x < 0.0 or x > 10 or z < -0.5: continue

            distance = sqrt(x**2 + y**2)
            if distance < min_distance:
                min_distance = distance
                closest_index = index

        if points[closest_index][0] < 0 or points[closest_index][0] > 10 or closest_index == -1:
            return [100, 0, -100]
        else:
            self.centroid_obs = points[closest_index]
            self.centroid_detect_time = rospy.Time.now().to_sec()
            return points[closest_index]
    
    def person_stop_check(self, closet_point):
        print(self.stop_count)
        if self.stop_count >= 300:
            self.need2stop = False
            return
        
        distance = sqrt(closet_point[0]**2 + closet_point[1]**2)
        print(distance)
        if distance < 5:
            self.need2stop = True
            self.prev_stop_time = rospy.Time.now().to_sec()

        if self.need2stop and rospy.Time.now().to_sec() - self.prev_stop_time <= 3:
            self.stop_count += 1
            self.dynamic_stop_pub.publish(True)
            self.dynamic_detected = True

        elif self.need2stop and rospy.Time.now().to_sec() - self.prev_stop_time > 3:
            self.need2stop = False
            self.dynamic_detected = True

            self.prev_stop_time = rospy.Time.now().to_sec()
        

    def activate_lidar_lane(self, lidar_data, centroids, left_tunnel_dis, right_tunnel_dis):
        # os.system('clear')
        center_waypoints, left_waypoints, right_waypoints = self.lane_detection.lidar_callback(lidar_data)

        avoid_obs = False
        lane = lane_info()

        if len(center_waypoints) >= 1:
            center_line_x = center_waypoints[:, 0]
            center_line_y = center_waypoints[:, 1]           
            center_line_yaw = center_waypoints[:, 2]
            
            lr_ratio = 100
            max_side_dis = max(abs(left_tunnel_dis), abs(right_tunnel_dis))
            both_side_dis = abs(left_tunnel_dis) + abs(right_tunnel_dis)
            if int(self.lane_detection.change_location) == 2:
                 lr_ratio = abs(left_tunnel_dis/(right_tunnel_dis+1e-5))
                #  print(f'lr_ratio: {lr_ratio}')
                #  print(f'max_side: {max_side_dis}')
                #  print(f'both_side: {both_side_dis}')
                #  print('==========================================')
                 
            if 0 < lr_ratio <= 1.25 and (max_side_dis < 15 or both_side_dis < 20) and not self.dynamic_detected and self.avoid_obs:
                center_line_y = [x-3.75 for x in center_line_y]
                lane.center_line_x = center_line_x         
                lane.center_line_y = center_line_y         
                lane.center_line_yaw = center_line_yaw     
                self.center_visualize_path(center_line_x, center_line_y)
                lane.speed_mode = 0
                lane.lane_mode = True
                if len(lane.center_line_x) != 0:    
                    self.lane_info_pub.publish(lane)
                return
            
            elif  50 > lr_ratio >= 2.1 and (max_side_dis < 15 or both_side_dis < 20) and not self.dynamic_detected and self.avoid_obs:
                center_line_y = [x+3 for x in center_line_y]
                lane.center_line_x = center_line_x         
                lane.center_line_y = center_line_y         
                lane.center_line_yaw = center_line_yaw     
                self.center_visualize_path(center_line_x, center_line_y)
                lane.speed_mode = 0
                lane.lane_mode = True
                if len(lane.center_line_x) != 0:    
                    self.lane_info_pub.publish(lane)
                return

            if len(centroids) != 0:
                
                closest_point = self.det_closest_point(centroids)
                left_line = []
                right_line = []
                center_line = []

                if len(left_waypoints) != 0:
                    left_line_m = (left_waypoints[-1][1]-left_waypoints[0][1]) / (left_waypoints[-1][0]-left_waypoints[0][0]+ 1e9)              
                    left_line_b = left_waypoints[0][1] - left_line_m * left_waypoints[0][0]
                    left_line = [left_line_m, left_line_b]


                if len(right_waypoints) != 0:
                    right_line_m = (right_waypoints[-1][1]-right_waypoints[0][1]) / (right_waypoints[-1][0]-right_waypoints[0][0]+ 1e9)              
                    right_line_b = right_waypoints[0][1] - right_line_m * right_waypoints[0][0]
                    right_line = [right_line_m, right_line_b]

                if len(center_waypoints) != 0:
                    center_line_m = (center_waypoints[-1][1]-center_waypoints[0][1]) / (center_waypoints[-1][0]-center_waypoints[0][0]+ 1e9)              
                    center_line_b = center_waypoints[0][1] - center_line_m * center_waypoints[0][0]
                    center_line = [center_line_m, center_line_b]

# ============================================================================================================================================
                # print(f'current_obs:  {closest_point}')
                if closest_point[0] == 100 and rospy.Time.now().to_sec() - self.centroid_detect_time < 1:
                    closest_point = self.centroid_obs
                # if tunnel_left_dis < tunnel_right_dis:
                #   center_line_y = [x - 4 for x in center_line_y]
                

                if len(left_line) != 0 and len(right_line) != 0 and closest_point[0] != 100:
                    
                    obs_left_lane_pos = left_line[0] * closest_point[0] + left_line[1]
                    obs_right_lane_pos = right_line[0] * closest_point[0] + right_line[1]
                    center_lane_pos = center_line[0] * closest_point[0] + center_line[1]
                    if closest_point[1] < obs_left_lane_pos and closest_point[1] > obs_right_lane_pos:
                        if closest_point[3] > self.cone_person_division_height:
                            self.person_stop_check(closest_point)
                            avoid_obs = True
                        # elif abs(obs_left_lane_pos-closest_point[1]) < abs(obs_right_lane_pos-closest_point[2]):
                        elif closest_point[1] <= center_lane_pos:
                            center_line_y = [x+0.8 for x in center_line_y]
                            avoid_obs = True
                            self.avoid_obs = True
                            
                            # 양쪽 모드에서 왼쪽으로 더 피하고 싶으면 아래 주석 풀기
                            # center_line_y = [x+1.0 for x in center_line_y]
                        # elif abs(obs_left_lane_pos-closest_point[1]) > abs(obs_right_lane_pos-closest_point[2]):
                        elif closest_point[1] > center_lane_pos:
                            center_line_y = [x-0.5 for x in center_line_y]
                            avoid_obs = True
                            self.avoid_obs = True
                    
                    # print('Mode: both')
                    # print(len(center_waypoints), len(left_waypoints), len(right_waypoints))
                    # print(f'left: {obs_left_lane_pos}, right: {obs_right_lane_pos}')
                    # print(f'obs: {closest_point}')
                    # print(f'lr: {lr}')

                elif len(left_line) != 0 and len(right_line) == 0 and closest_point[0] != 100:
                    obs_left_lane_pos = left_line[0] * closest_point[0] + left_line[1]
                    
                    if obs_left_lane_pos - self.lane_width_offset * 0.5 < closest_point[1] < obs_left_lane_pos:

                        if closest_point[3] > self.cone_person_division_height:
                            self.person_stop_check(closest_point)
                            avoid_obs = True
                            self.avoid_obs = True

                        elif closest_point[3] <= self.cone_person_division_height:
                            center_line_y = [x-1 for x in center_line_y]
                            avoid_obs = True
                            self.avoid_obs = True
                        
                    elif obs_left_lane_pos - self.lane_width_offset < closest_point[1] <= obs_left_lane_pos - self.lane_width_offset * 0.5:

                        if closest_point[3] > self.cone_person_division_height:
                            self.person_stop_check(closest_point)
                            avoid_obs = True
                            self.avoid_obs = True

                        elif closest_point[3] <= self.cone_person_division_height:
                            center_line_y = [x+0.5 for x in center_line_y]
                            avoid_obs = True
                            self.avoid_obs = True

                    # print('Mode: left')
                    # print(len(center_waypoints), len(left_waypoints), len(right_waypoints))
                    # print(obs_left_lane_pos, obs_left_lane_pos - self.lane_width_offset * 0.5)
                    # print(f'left: {obs_left_lane_pos}')
                    # print(f'obs: {closest_point}')


                elif len(right_line) != 0 and len(left_line) == 0 and closest_point[0] != 100:
                    obs_right_lane_pos = right_line[0] * closest_point[0] + right_line[1]
                    if obs_right_lane_pos + self.lane_width_offset * 0.5 > closest_point[1] > obs_right_lane_pos:
                        self.lane_detection.obs_stop = True
                        if closest_point[3] > self.cone_person_division_height:
                            self.person_stop_check(closest_point)
                            avoid_obs = True
                            self.avoid_obs = True

                        else:
                            center_line_y = [x+0.5 for x in center_line_y]
                            avoid_obs = True
                            self.avoid_obs = True

                    elif obs_right_lane_pos + self.lane_width_offset > closest_point[1] >= obs_right_lane_pos + self.lane_width_offset * 0.5:
                        self.lane_detection.obs_stop = True
                        if closest_point[3] > self.cone_person_division_height:
                            self.person_stop_check(closest_point)
                            avoid_obs = True
                            self.avoid_obs = True
                        else:
                            center_line_y = [x-1 for x in center_line_y]
                            avoid_obs = True
                            self.avoid_obs = True


                    # print('Mode: right')
                    # print(len(center_waypoints), len(left_waypoints), len(right_waypoints))
                    # print(obs_right_lane_pos + self.lane_width_offset * 0.5, obs_right_lane_pos)
                    # print(f'obs: {closest_point}')
                
            lane.center_line_x = center_line_x         
            lane.center_line_y = center_line_y         
            lane.center_line_yaw = center_line_yaw     
            self.center_visualize_path(center_line_x, center_line_y)

            # speed_mode -> 0: 장애물 모드(3km/h)  1: 저속 모드(5km/h)  2: 기본 모드(10km/h)  3: 고속 모드(15km/h)
            if self.lane_detection.change_location == 2: 
                if avoid_obs: lane.speed_mode = 0
                elif not avoid_obs and not self.dynamic_detected: lane.speed_mode = 1
                elif not avoid_obs and self.dynamic_detected: lane.speed_mode = 2

            elif self.lane_detection.change_location != 2: lane.speed_mode = 2
            else: lane.speed_mode = 1

            # print(f'Speed mode: {lane.speed_mode}')
            lane.lane_mode = True
            if len(lane.center_line_x) != 0:
                self.lane_info_pub.publish(lane)
            

        
        