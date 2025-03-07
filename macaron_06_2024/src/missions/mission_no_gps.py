#!/usr/bin/env python3
#-*-coding:utf-8-*-

import rospy
import numpy as np

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

from sensor.Dead_reckoning_class_pub_smc import Dead_reckoning_class

class MissionNoGPS():
    def __init__(self):

        self.est_pose_pub = rospy.Publisher('/estimate_pose', Point, queue_size=1)

        self.Dead_reckoning = Dead_reckoning_class()

        self.pose_init = Point()
        self.prev_slam_pose = Point()
        self.prev_dead_pose = Point()
        self.prev_pose = Point()


        self.odom_init = Odometry()

        self.init_flag = False

        self.dynamic_detected = False
        self.lane_detect_mode = False

        self.fail_dead = False
        self.fail_flag = False


        self.Dead_reckoning_pose = [0,0]

    def init_slam_setting(self, odom_init, pose_init, heading_init):
        
        self.odom_init = odom_init
        self.pose_init = pose_init

    def init_dead_setting(self):
        self.Dead_reckoning.Dead_reckoning(1) # 헤딩값 보정 데이터 수집

    def init_setting(self, odom_init, pose_init, slam_heading_init):

        self.init_slam_setting(odom_init, pose_init, slam_heading_init)
        self.init_dead_setting() # 필요한 변수 추가하기
        print('init complete')

    def slam_activate(self, odom: Odometry, heading, init_heading):

        x = odom.pose.pose.position.x - self.odom_init.pose.pose.position.x
        y = odom.pose.pose.position.y - self.odom_init.pose.pose.position.y

        real_x = x * np.cos(init_heading) - y * np.sin(init_heading)
        real_y = x * np.sin(init_heading) + y * np.cos(init_heading)

        slam_pose = Point()
        slam_pose.x = real_x + self.pose_init.x
        slam_pose.y = real_y + self.pose_init.y
        slam_pose.z = heading

        self.prev_slam_pose = slam_pose
        return slam_pose
    
    def dead_reckoning(self):
        pose = self.Dead_reckoning.Dead_reckoning(2)# 데드렉코닝 시작
        self.prev_dead_pose.x = pose[0]
        self.prev_dead_pose.y = pose[1]
        self.prev_dead_pose.z = pose[2]

        return self.prev_dead_pose
    
    def fail_dead_check(self, dead_pose: Point, heading):
        dead_heading_diff = dead_pose.z - heading
        pi = np.pi
        if dead_heading_diff >= pi: dead_heading_diff -= pi
        if dead_heading_diff < -pi: dead_heading_diff += pi
        
        if abs(dead_heading_diff) <= 10/180 * pi:
            return True
        else: 
            return False

    
    def det_est_pose(self, slam_pose: Point, dead_pose: Point):

        pose = Point()
        if self.fail_dead:
            pose.x = dead_pose.x
            pose.y = dead_pose.y
            pose.z = dead_pose.z
        else:
            pose.x = slam_pose.x
            pose.y = slam_pose.y
            pose.z = slam_pose.z

        return pose

    def change2lane_detect(self, est_pose: Point):
        dis = np.sqrt((est_pose.x - self.pose_init.x)**2 + (est_pose.y - self.pose_init.y)**2)
        if dis > 75 or self.dynamic_detected:
            self.lane_detect_mode = True


    def activate(self, odom, heading, init_heading):

        if self.lane_detect_mode:
            pass

        slam_pose, dead_pose = [0,0], [0,0]

        slam_pose = self.slam_activate(odom, heading, init_heading)
        dead_pose = self.dead_reckoning() # <--------------- 여기에 데드 레코닝 추정 위치 리턴 받게 하기
        print(dead_pose)
        print('===========================')
        if not self.fail_dead: self.fail_dead = self.fail_dead_check(dead_pose, heading)

        est_pose = self.det_est_pose(slam_pose, dead_pose)
        est_pose = dead_pose
        self.est_pose_pub.publish(est_pose)

        self.prev_pose = est_pose
    