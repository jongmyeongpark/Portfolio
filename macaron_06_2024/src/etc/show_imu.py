#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from ublox_msgs.msg import NavPVT
from math import pi
import math
import numpy as np
from geometry_msgs.msg import Point
from macaron_5.msg import erp_read
from tf.transformations import euler_from_quaternion


pub_marker_imu = None
pub_marker_gps = None
erp_gear = None
imu_yaw = None
gps_yaw = None


def imu_callback(data):
    global imu_yaw, gps_yaw
    marker = Marker()

    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    marker.type = marker.ARROW
    marker.action = marker.ADD

    marker.scale.x = 1.0
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0

    marker.pose.orientation = data.orientation


    orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z,data.orientation.w]
    _,_,yaw = euler_from_quaternion(orientation_list)
    imu_yaw = yaw
    #self.yaw = self.q_to_yaw(self.yaw)
    #print("degree yaw!!!!!! : ",180*yaw/pi)
    pub_marker_imu.publish(marker)
    
    if gps_yaw != None and imu_yaw != None: 
        print("offsset : ",  gps_yaw - imu_yaw)

def erp_callback(erp) :
    global erp_gear
    erp_gear = erp.read_gear


def gps_heading_callback(head):
    global erp_gear, gps_yaw
    gps_heading = tf_heading_to_rad(float(head.heading))
    
    if(erp_gear == 2) :
        gps_heading += pi
        if gps_heading > 2 * pi:
            gps_heading -= (2 * pi)
        elif gps_heading <= 0:
            gps_heading += (2 * pi)
            
    gps_yaw = gps_heading

    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.id = 0

    # 시작점 설정
    marker.points.append(Point(0, 0, 0))
    # 끝점 설정 (예: 길이 1.0의 화살표가 주어진 각도로 나타나게 합니다.)
    marker.points.append(Point(math.cos(gps_heading), math.sin(gps_heading), 0))

    marker.scale.x = 0.05  # 화살표의 폭
    marker.scale.y = 0.1   # 화살표의 두께
    marker.scale.z = 0.1   # 화살표의 높이

    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    pub_marker_gps.publish(marker)

def erp_heading_callback(data):
    
    erp_heading = data.z
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.id = 1

    # 시작점 설정
    marker.points.append(Point(0, 0, 0))
    # 끝점 설정 (예: 길이 1.0의 화살표가 주어진 각도로 나타나게 합니다.)
    marker.points.append(Point(math.cos(erp_heading), math.sin(erp_heading), 0))

    marker.scale.x = 0.05  # 화살표의 폭
    marker.scale.y = 0.1   # 화살표의 두께
    marker.scale.z = 0.1   # 화살표의 높이

    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    pub_marker_erp.publish(marker)


def tf_heading_to_rad(head):
    heading = 5*pi/2 - np.deg2rad(float(head / 100000))
    if heading > 2*pi:
        heading -= 2*pi
    return heading

def main():
    global pub_marker_imu, pub_marker_gps, pub_marker_erp, gps_yaw, imu_yaw

    rospy.init_node('imu_visualization_node')
    if gps_yaw != None and imu_yaw != None: 
        print("offsset : ",  gps_yaw - imu_yaw)

    # rospy.Subscriber("/imu", Imu, imu_callback)
    rospy.Subscriber("/imu/data", Imu, imu_callback)
    rospy.Subscriber('/ublox_gps/navpvt', NavPVT, gps_heading_callback, queue_size=1)
    rospy.Subscriber('erp_read', erp_read, erp_callback, queue_size=1)
    rospy.Subscriber('/current_pose', Point, erp_heading_callback, queue_size = 1)


    pub_marker_imu = rospy.Publisher('/imu_marker', Marker, queue_size=10)
    pub_marker_gps = rospy.Publisher('/gps_marker', Marker, queue_size=10)
    pub_marker_erp = rospy.Publisher('/erp_marker', Marker, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main()
