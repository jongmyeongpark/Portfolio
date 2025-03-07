#!/usr/bin/env python
# -- coding: utf-8 --

# basic package
from datetime import date
import rospy
import time
from math import cos, sin, pi
import math
import numpy as np
import matplotlib.ticker as ticker
import time
from pyproj import Proj, transform
import matplotlib.pyplot as plt

# message file
from macaron_5.msg import erp_read
from sensor_msgs.msg import Imu, NavSatFix
from ublox_msgs.msg import NavPVT
from geometry_msgs.msg import Point, TwistWithCovarianceStamped

from tf.transformations import euler_from_quaternion

class Heading_test():

    def __init__(self):
        #Projection definition
        #UTM-K
        self.proj_UTMK = Proj(init='epsg:5179')
        #WGS1984
        self.proj_WGS84 = Proj(init='epsg:4326')

        self.enc = 0
        self.imu_heading = 0
        self.steer = 0
        self.heading = 0
        self.gps_heading = 0
        self.yaw_offset = 0
        self.pose = [0,0]
        self.speed = 0
        self.gps_pose = [0,0]

        #pub sub
        self.sub_imu = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)
        self.erp_sub = rospy.Subscriber('erp_read', erp_read, self.erp_callback, queue_size=1)        
        self.pose_sub = rospy.Subscriber('current_pose', Point, self.pose_callback, queue_size = 1) 
        self.sub_gps_heading = rospy.Subscriber('/ublox_gps/navpvt', NavPVT, self.gps_heading_callback, queue_size=1)
        self.sub_gps_pose = rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.gps_pose_callback, queue_size=1)
        #self.sub_gps_pose = rospy.Subscriber('/ublox_gps/fix_velocity', TwistWithCovarianceStamped, self.gps_velocity_callback, queue_size=1)

        #센서 주기 정리
        #gps : 0.1s
        #imu : 0.01s
        #erp_엔코더 : 0.1s
        #erp_speed : 0.03 - 0.5s -> 평균적으로 0.1s정도 생각하면 될듯
        
        #센서 주기 확인용
        self.time_stamp = 0
        self.prev = 0
        self.enc_speed = 0


    def pose_callback(self, data): # gps_pose + 보정된 헤딩

        self.pose = [data.x, data.y]
        self.heading = data.z

    def imu_callback(self, imu):

        global pose2, ekf_pose
        self.imu_heading = self.q_to_rpy(imu.orientation)

    def gps_pose_callback(self, Fix): 

        x,y = self.tf_to_tm(Fix.longitude, Fix.latitude)
        self.gps_pose = [x,y]

    def gps_heading_callback(self, head):

        self.gps_heading = self.tf_heading_to_rad(float(head.heading))
        
    def gps_velocity_callback(self, data):

        self.gps_velocity = np.hypot(data.twist.twist.linear.x, data.twist.twist.linear.y)


    def erp_callback(self, data):

        self.steer = data.read_steer/71
        self.speed = data.read_speed*(1000/3600/10)
        self.enc = data.read_ENC


##############좌표 변환 함수들#################

    def q_to_yaw(self, imu):

        yaw = (-1) * imu.x * pi / 180

        if yaw < 0:
            yaw = pi + (pi + yaw)
        
        if yaw > 2 * pi:
            yaw = yaw - (2 * pi)
        elif yaw <= 0:
            yaw = yaw + (2 *pi)

        return yaw

    
    def q_to_rpy(self, imu_q):
        
        orientation_list = [imu_q.x, imu_q.y, imu_q.z, imu_q.w]
        _,_,yaw = euler_from_quaternion(orientation_list)
        if yaw < 0:
            yaw = pi + (pi + yaw)

        # print(yaw)

        return yaw
    
    
    def tf_to_tm(self,lon,lat):

        x,y=transform(self.proj_WGS84,self.proj_UTMK,lon,lat)

        return x,y

    def tf_heading_to_rad(self, head):

        heading = 5*pi/2 - np.deg2rad(float(head / 100000))

        if heading > 2*pi:
            heading -= 2*pi

        return heading

######################################

def main():

    rospy.init_node("Heading_test_node")

    hd=Heading_test()
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        print("imu_heading : ", hd.imu_heading)
        print("gps_heading : ", hd.gps_heading)
        print("offset : ", hd.gps_heading-hd.imu_heading)
        print("calibrated_heading : ", hd.heading)
        print("-----------------------------------")
        rate.sleep()
    
if __name__ == '__main__':
    main()

