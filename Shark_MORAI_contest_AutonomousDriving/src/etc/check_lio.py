#!/usr/bin/env python3
#-*-coding:utf-8-*-

# Python packages

import rospy
import os, sys
import numpy as np
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion

# message 파일
from std_msgs.msg import Float32, Bool, Int8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Point32
from sensor_msgs.msg import PointCloud2, PointCloud, Imu
from macaron_06.srv import MapLoad, MapLoadResponse

class PublishErp():
    def __init__(self):
        
        pass
        
class SubscribeErp: 
    def __init__(self):
        self.pose_sub = rospy.Subscriber('/current_pose', Point, self.pose_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber('/slam_pose', Point, self.slam_pose_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('/Odometry', Odometry, self.odom_callback, queue_size=2)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size = 2)

        self.pose = []
        self.pose_x = []
        self.pose_y = []

        self.slam_pose = []
        self.slam_pose_x = []
        self.slam_pose_y = []

        self.heading_list = []
        self.heading = 0.0

        self.first_odom_pose = []
        self.odom_pose = []
        self.odom_x = []
        self.odom_y = []
        self.odom_heading_list = []

        self.pose_yaw = 0.0
        self.odom_yaw = 0.0
        
    def pose_callback(self, data):
        if self.pose == [] and data.z != 0.0: 
            self.pose = [data.x, data.y]
            self.heading = data.z
        
        if data.z != 0.0:
            x = data.x
            y = data.y
            # self.pose_x.append(x * np.cos(-self.heading) - y * np.sin(-self.heading))
            # self.pose_y.append(x * np.sin(-self.heading) + y * np.cos(-self.heading))
            self.pose_x.append(x)
            self.pose_y.append(y)

    def slam_pose_callback(self, data):
        x = data.x
        y = data.y
        self.slam_pose_x.append(x)
        self.slam_pose_y.append(y)

    def imu_callback(self, imu):
        quaternion = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.pose_yaw = 2 * np.pi - yaw
        if self.pose_yaw < 0:
            self.pose_yaw += 2* np.pi
        if self.pose_yaw > 2 *np.pi:
            self.pose_yaw -= 2 * np.pi


    def odom_callback(self, odom):
        self.odom_pose = [odom.pose.pose.position.x, odom.pose.pose.position.y]
        if self.first_odom_pose == []: 
            self.first_odom_pose = self.odom_pose
        quaternion = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
        roll, pitch, self.odom_yaw = euler_from_quaternion(quaternion)
        x = self.odom_pose[0] - self.first_odom_pose[0]
        y = self.odom_pose[1] - self.first_odom_pose[1]
        # self.odom_x.append(x * np.cos(-self.odom_yaw) - y * np.sin(-self.odom_yaw))
        # self.odom_y.append(x * np.sin(-self.odom_yaw) + y * np.cos(-self.odom_yaw))
        self.odom_x.append(x)
        self.odom_y.append(y)
    
def main():
    rate = rospy.Rate(10)
    pub = PublishErp()
    erp = SubscribeErp()
    pose_yaw = []
    odom_yaw = []
    odom_x = []
    odom_y = []
    while not rospy.is_shutdown():
        if erp.pose_yaw != 0.0 and erp.odom_yaw != 0.0:
            pose_yaw.append(erp.pose_yaw)
            yaw = pose_yaw[0] + erp.odom_yaw
            if yaw < 0:
                yaw += 2*np.pi

            if yaw >= 2*np.pi:
                yaw -= 2*np.pi
            odom_yaw.append(yaw)
        try:
            print(f'slam: {erp.slam_pose_x[-1]}, {erp.slam_pose_y[-1]}')
            print(f'{erp.pose_x[-1]} ,{erp.pose_y[-1]}')
            print('===================================')
        except: pass
        rate.sleep()

    # plt.plot(range(0, len(pose_yaw)), pose_yaw, 'r')
    # plt.plot(range(0, len(odom_yaw)), odom_yaw, 'b')
    # plt.plot(odom_x, odom_y, 'r')
    plt.plot(erp.pose_x, erp.pose_y, 'b')
    plt.plot(erp.slam_pose_x, erp.slam_pose_y, 'r')
    plt.show()

        
if __name__ == '__main__':
    rospy.init_node("check_lio", anonymous=True)
    main()