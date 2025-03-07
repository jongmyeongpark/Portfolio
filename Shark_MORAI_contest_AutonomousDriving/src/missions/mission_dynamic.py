#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
import numpy as np
from math import *
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Point

PREDICT_TIME = 20 # 2초

class MissionDynamicObs():
    def __init__(self):

        self.dynamic_stop_pub=rospy.Publisher('/dynamic_stop', Bool, queue_size=1)
        self.stop_pub=rospy.Publisher('/stop', Bool, queue_size=1)
        self.follow_speed_pub=rospy.Publisher('/follow_speed', Float32, queue_size=1)

        self.prev_obj_list = []
        self.current_obj_list = []

        self.prev_time = 0.0
        self.current_time = 0.0
        self.no_detect_count = 0

        self.prev_stop_time = 0

        self.vehicle_count = 0

        self.need2stop = False
        self.car_passed = False



    def update_dynamic(self, obj_list):
        if obj_list == []:
            self.prev_obj_list = []
            return []
        
        if self.prev_obj_list == []:
            self.prev_obj_list = obj_list
            self.prev_time = rospy.Time.now().to_sec()
            return []



        self.current_obj_list = obj_list
        self.current_time = rospy.Time.now().to_sec()
        
        dt = self.current_time - self.prev_time

        dynamic_list = []

        for cur_obj in obj_list:
            obj_data = []
            for prev_obj in self.prev_obj_list:
                if cur_obj[2] == prev_obj[2]:
                    dx = cur_obj[0] - prev_obj[0]
                    dy = cur_obj[1] - prev_obj[1]
                    dis = sqrt(dx**2 + dy**2)
                    speed = dis / dt
                    # print(dis)
                    if 0.01 < speed < 6 and dis > 0.01:
                        obj_data = [[cur_obj[0], cur_obj[1]], [dx, dy], speed * 3.6]
            
            if obj_data != []:
                dynamic_list.append(obj_data)

        self.prev_time = self.current_time
        self.prev_obj_list = self.current_obj_list
        # print(dynamic_list)
        return dynamic_list
    
    def update_dynamic_vehicle(self, obj_list):
        if obj_list == []:
            self.no_detect_count += 1
            if self.no_detect_count > 2:
                self.no_detect_count = 0
                self.vehicle_count = 0
                self.prev_obj_list = []
            return []
        
        if self.prev_obj_list == []:
            self.prev_obj_list = obj_list
            self.prev_time = rospy.Time.now().to_sec()
        
        self.current_obj_list = obj_list
        self.current_time = rospy.Time.now().to_sec()
        dt = self.current_time - self.prev_time

        dynamic_list = []

        for cur_obj in obj_list:
            min_dis = 100
            obj_data = []
            for prev_obj in self.prev_obj_list:
                dx = cur_obj[0] - prev_obj[0]
                dy = cur_obj[1] - prev_obj[1]
                dis = sqrt(dx**2 + dy**2)
                speed = dis / dt
                # print(dis)
                if dis< min_dis and 0.01 < speed < 6 and dis > 0.01:
                    min_dis = dis
                    obj_data = [[cur_obj[0], cur_obj[1]], [dx, dy], speed * 3.6]
            
            if obj_data != []:
                dynamic_list.append(obj_data)

        self.prev_time = self.current_time
        self.prev_obj_list = self.current_obj_list
        # print(f'dynamic: {dynamic_list}')
        return dynamic_list
    
    def update_dynamic_person(self, obj_list):
        if obj_list == []:
            self.no_detect_count += 1
            if self.no_detect_count > 3:
                self.no_detect_count = 0
                self.prev_obj_list = []
            return []
        
        if self.prev_obj_list == []:
            self.prev_obj_list = obj_list
            self.prev_time = rospy.Time.now().to_sec()
        
        self.current_obj_list = obj_list
        self.current_time = rospy.Time.now().to_sec()
        dt = self.current_time - self.prev_time

        dynamic_list = []

        for cur_obj in obj_list:
            min_dis = 100
            obj_data = []
            for prev_obj in self.prev_obj_list:
                dx = cur_obj[0] - prev_obj[0]
                dy = cur_obj[1] - prev_obj[1]
                dis = sqrt(dx**2 + dy**2)
                speed = dis / dt
                # print(dis)
                if dis< min_dis and dis > 0.01:
                    min_dis = dis
                    obj_data = [[cur_obj[0], cur_obj[1]], [dx, dy], speed * 3.6]
            
            if obj_data != []:
                dynamic_list.append(obj_data)

        self.prev_time = self.current_time
        self.prev_obj_list = self.current_obj_list
        # print(f'dynamic: {dynamic_list}')
        return dynamic_list
        
    
    def vehicle_check(self, pose: Point, dynamic_list):
        # speed 단위는 km/h
        head_pose_x = 2.3 * np.cos(pose.z) + pose.x
        head_pose_y = 2.3 * np.sin(pose.z) + pose.y

        follow_speed = -1
        speed = 10
        
        for obs in dynamic_list:
            head_dis = sqrt((obs[0][0]-head_pose_x)**2+(obs[0][1]-head_pose_y)**2) # 차 범퍼
            pose_dis = sqrt((obs[0][0]-pose.x)**2+(obs[0][1]-pose.y)**2) # GPS 거라
            dis = min(head_dis, pose_dis)
            yaw = np.arctan2(obs[1][1], obs[1][0])
            if yaw < 0: yaw += np.pi * 2
            heading_diff = pose.z - yaw
            if heading_diff > 0:
                pass
            elif heading_diff < 0:
                heading_diff += 2 * np.pi

            # print(f'dis: {dis}')
            # print(f'diff: {heading_diff}')
            # print(self.car_passed)
            # print('=====================')
            if not self.car_passed and dis < 15 and heading_diff > 1.60:
                self.dynamic_stop_pub.publish(True)
                self.car_passed = True
                break
            elif not self.car_passed: 
                self.follow_speed_pub.publish(4)
                
            if dis < 5 and heading_diff < 1:
                follow_speed = 0
        
            elif obs[2] > speed:
                speed = obs[2]
                follow_speed = speed
                
        if follow_speed != -1:
            self.follow_speed_pub.publish(int(follow_speed))

    def person_check(self, pose, path, dynamic_list):
        head_pose_x = 2.3 * np.cos(pose.z) + pose.x
        head_pose_y = 2.3 * np.sin(pose.z) + pose.y
        stop_detect_time = 0.0

        for person in dynamic_list:

            if self.need2stop: break

            cur_dis = np.sqrt((person[0][0]-head_pose_x)**2 + (person[0][1]-head_pose_y)**2)

            for time_step in range(0, PREDICT_TIME):
                time_step *= 0.1
                predict_x = person[0][0] + person[1][0] * time_step
                predict_y = person[0][1] + person[1][1] * time_step
                distances = np.sqrt((path[0] - predict_x) ** 2 + (path[1] - predict_y) ** 2)
                min_index = np.argmin(distances)
                min_distance = distances[min_index]
                if (min_index <= 200 and min_distance <= 4) or cur_dis < 5:
                    self.need2stop =True
                    stop_detect_time = rospy.Time.now().to_sec()
                    break
        

        if self.need2stop:
            self.dynamic_stop_pub.publish(True)
            # print('activated')
            self.need2stop = False
        else:
            self.need2stop = False

    def person_check_with_static(self, pose, path, dynamic_list):
        head_pose_x = 2.3 * np.cos(pose.z) + pose.x
        head_pose_y = 2.3 * np.sin(pose.z) + pose.y
        stop_detect_time = 0.0
        for person in dynamic_list:

            if self.need2stop: break

            cur_dis = np.sqrt((person[0][0]-head_pose_x)**2 + (person[0][1]-head_pose_y)**2)
        

            for time_step in range(0, PREDICT_TIME):
                time_step *= 0.1
                predict_x = person[0][0] + person[1][0] * time_step
                predict_y = person[0][1] + person[1][1] * time_step
                distances = np.sqrt((path[0] - predict_x) ** 2 + (path[1] - predict_y) ** 2)
                min_index = np.argmin(distances)
                min_distance = distances[min_index]
                if (min_index <= 200 and min_distance <= 4) or cur_dis < 5:
                    self.need2stop =True
                    stop_detect_time = rospy.Time.now().to_sec()
                    break

        if self.need2stop and stop_detect_time - self.prev_stop_time > 5:
            self.dynamic_stop_pub.publish(True)
            # print('activated')
            self.prev_stop_time = rospy.Time.now().to_sec()
            self.need2stop = False
        else:
            self.need2stop = False

        if rospy.Time.now().to_sec() - self.prev_stop_time > 5:
            return True
        else: 
            return False 

    
    def dynamic_follow(self, current_pose, obj_list):
        dynamic_list = self.update_dynamic_vehicle(obj_list)
        self.vehicle_check(current_pose, dynamic_list)

    def dynamic_stop(self, current_pose, path_data, obj_list):
        dynamic_list = self.update_dynamic_person(obj_list)
        self.person_check(current_pose, path_data, dynamic_list)

    def dynamic_stop_with_static(self, current_pose, path_data, obj_list):
        dynamic_list = self.update_dynamic_person(obj_list)
        self.person_check_with_static(current_pose, path_data, dynamic_list)

    def image_dynamic_stop(self, result):
        stop_detect_time = rospy.Time.now().to_sec()
        if result and stop_detect_time - self.prev_stop_time > 5:
            self.dynamic_stop_pub.publish(True)
            self.prev_stop_time = rospy.Time.now().to_sec()
        else:
            pass
