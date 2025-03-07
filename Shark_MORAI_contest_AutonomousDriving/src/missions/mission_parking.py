#!/usr/bin/env python3
#-*-coding:utf-8-*-

import rospy
import numpy as np
from math import sqrt
from std_msgs.msg import Header, ColorRGBA, Bool, Int8
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from morai_msgs.msg import EventInfo


import path_planning.cubic_spline_planner as cubic_spline_planner

WB = 3
ANGLE = np.radians(27)
R_turn=WB/np.tan(ANGLE)
d=4.5

class MissionParking():
    def __init__(self):
        self.vizpath_pub = rospy.Publisher('/parking_vizpath', Marker, queue_size=1)
        self.path_pub = rospy.Publisher('/parking_path', PointCloud, queue_size=1)
        self.stop_pub = rospy.Publisher('/stop', Bool, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear', Int8, queue_size=1)


        self.parking_point = np.array([])
        self.backpoint = np.array([])

        self.start_current_pose = Point()
        self.distance = 11
        self.parking_detect_count = 0
        self.parking_status = 0
        self.parking_time = 0.0
        self.end_flag = False

    def visualize_path(self, current_pose, waypoints):
        rviz_msg_path=Marker(
            header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
            ns="parkings_path",
            id=190,
            type=Marker.LINE_STRIP,
            lifetime=rospy.Duration(0.5),
            action=Marker.ADD,
            scale=Vector3(0.2,0.0,0.0),
            color=ColorRGBA(r=1.0,g=0.0,b=1.0,a=0.8)
        )
        for point in waypoints:
            p = Point()
            p.x = point[0] - current_pose.x
            p.y = point[1] - current_pose.y
            p.z = 0.1
            rviz_msg_path.points.append(p)
        self.vizpath_pub.publish(rviz_msg_path)

    def pub_parking_path(self, waypoints, frontback):
        # frontback이 1이면 전진, -1이면 후진
        parking_path = PointCloud()
        for point in waypoints:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = frontback
            parking_path.points.append(p)
        self.path_pub.publish(parking_path)

    def pub_gear(self, num):
        self.gear_pub.publish(num)

    def current_pose_save(self, current_pose):
        self.start_current_pose = current_pose

    def detect(self, parking_point):        
        if self.parking_point.size == 0:
            self.parking_point = parking_point
            self.parking_detect_count += 1
        else:
            if (self.parking_point == parking_point).all():
                self.parking_detect_count +=1
            else:
                self.parking_point = np.array([])

        if self.parking_detect_count >= 5:
            back_x = self.parking_point[0] + self.distance * np.cos(1.60) - 0.1 * np.sin(1.60)
            back_y = self.parking_point[1] + self.distance * np.sin(1.60) + 0.1 * np.cos(1.60)
            backpoint = [back_x, back_y]
            second_x = self.parking_point[0] + (d + R_turn) * np.cos(1.60)
            second_y = self.parking_point[1] + (d + R_turn) * np.sin(1.60)
            second_pose = [second_x, second_y]
            first_x = self.parking_point[0] + d * np.cos(1.60) - (-R_turn) * np.sin(1.60)
            first_y = self.parking_point[1] + d * np.sin(1.60) + (-R_turn) * np.cos(1.60)
            first_pose = [first_x, first_y]
            zero_x = self.parking_point[0] + d * np.cos(1.60) - (-R_turn*1.1) * np.sin(1.60)
            zero_y = self.parking_point[1] + d * np.sin(1.60) + (-R_turn*1.1) * np.cos(1.60)
            zero_pose = [zero_x, zero_y]
            start_x = self.parking_point[0] + d * np.cos(1.60) - (-R_turn*1.25) * np.sin(1.60)
            start_y = self.parking_point[1] + d * np.sin(1.60) + (-R_turn*1.25) * np.cos(1.60)
            start_pose = [start_x, start_y]
            self.parking_status = 1
            self.backward_path = [start_pose, zero_pose, first_pose, second_pose, backpoint]

    def move2start(self, current_pose):
        path = [[self.start_current_pose.x, self.start_current_pose.y], self.backward_path[0]]
        waypoints = self.spline_interpolation(path)
        self.visualize_path(current_pose, waypoints)
        self.pub_parking_path(path, 1)

        distance = sqrt((current_pose.x-self.backward_path[0][0])**2 +(current_pose.y-self.backward_path[0][1])**2)
        # print(f'dis1: {distance}')
        if distance < 0.5:
            self.parking_status = 2

    def start2backpoint(self, current_pose):
        waypoints = self.spline_interpolation(self.backward_path)
        self.visualize_path(current_pose, waypoints)
        self.pub_parking_path(self.backward_path, -1)
        self.pub_gear(2)
        distance = sqrt((current_pose.x-self.backward_path[-1][0])**2 +(current_pose.y-self.backward_path[-1][1])**2)
        # print(f'dis2: {distance}')
        if distance < 1.5:
            self.parking_status = 3


    def backpoint2square(self, current_pose):
        path = [self.backward_path[-1], self.parking_point]
        waypoints = self.spline_interpolation(path)
        self.visualize_path(current_pose, waypoints)
        self.pub_parking_path(path, 1)
        self.pub_gear(4)
        distance = sqrt((current_pose.x-self.parking_point[0])**2 +(current_pose.y-self.parking_point[1])**2)
        # print(f'dis3: {distance}')
        if distance < 1:
            self.parking_status = 4

    def parking(self):
        if self.parking_time == 0.0:
            self.parking_time = rospy.Time.now().to_sec()
            self.pub_gear(1)

        now = rospy.Time.now().to_sec()
        if now - self.parking_time >= 5:
            self.parking_status = 5
        

    def square2backpoint(self, current_pose):
        path = [self.parking_point, self.backward_path[-1]]
        waypoints = self.spline_interpolation(path)
        self.visualize_path(current_pose, waypoints)
        self.pub_parking_path(path, -1)
        self.pub_gear(2)
        distance = sqrt((current_pose.x-self.backward_path[-1][0])**2 +(current_pose.y-self.backward_path[-1][1])**2)
        # print(f'dis4: {distance}')
        if distance < 0.5:
            self.parking_status = 6
       

    def back2globalpath(self):
        self.pub_parking_path([[0,0],[1,1]], 0)
        self.pub_gear(4)
        self.parking_status = 7
        self.end_flag = True
        
    
    def spline_interpolation(self, points):
        waypoint=np.empty((0,3))
        path_x = [point[0] for point in points]
        path_y = [point[1] for point in points]
        cx, cy, cyaw, _, _, _ = cubic_spline_planner.calc_spline_course(path_x, path_y, ds=0.1)

        for i in range(len(cx)):
            point=[cx[i],cy[i],cyaw[i]]
            waypoint = np.vstack((waypoint, np.array(point)))

        return waypoint
    
    def activate(self):
        
        pass