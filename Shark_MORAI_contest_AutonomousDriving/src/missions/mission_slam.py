#!/usr/bin/env python3
#-*-coding:utf-8-*-

import rospy
import numpy as np

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

class MissionSLAM():
    def __init__(self):
        self.slam_pose_pub = rospy.Publisher('/estimate_pose', Point, queue_size=1)

        self.pose_init = Point()
        self.odom_init = Odometry()

        self.current_odom = Odometry()
        self.prev_odom = Odometry()

    def init_setting(self, odom_init, pose_init):
        
        self.odom_init = odom_init
        self.pose_init = pose_init

    def activate(self, odom, heading, start_heading):
        print(odom.pose.pose.position.x, odom.pose.pose.position.y)
        
        x = odom.pose.pose.position.x - self.odom_init.pose.pose.position.x
        y = odom.pose.pose.position.y - self.odom_init.pose.pose.position.y

        real_x = x * np.cos(start_heading) - y * np.sin(start_heading)
        real_y = x * np.sin(start_heading) + y * np.cos(start_heading)

        slam_pose = Point()
        slam_pose.x = real_x + self.pose_init.x
        slam_pose.y = real_y + self.pose_init.y
        slam_pose.z = heading
        # print(slam_pose)
        # print('==========================================')
        self.slam_pose_pub.publish(slam_pose)
        # print(f'{odom.pose.pose.position.x, odom.pose.pose.position.y}')
        # print(f'{self.odom_init.pose.pose.position.x, self.odom_init.pose.pose.position.y}')
        # print(f'Init_pose: {self.pose_init.x , self.pose_init.y}')
        # print(f'SLAM_pose: {slam_pose.x , slam_pose.y}')
        # print('===========================================')

        # print(f'Init_odom: {self.odom_init.pose.pose.position.x , self.odom_init.pose.pose.position.y}')
        # print(f'current_odom: {odom.pose.pose.position.x , odom.pose.pose.position.y}')
        # print()
        # print('========')
        return slam_pose

    