#!/usr/bin/env python
import rospy
import time
import os, sys

from math import cos, sin, pi
import numpy as np
from pyproj import Proj, transform
import matplotlib.pyplot as plt

from macaron_5.msg import erp_read
from sensor_msgs.msg import LaserScan, PointCloud
from ublox_msgs.msg import NavPVT
from geometry_msgs.msg import Point

re_calc_points_NPY = "oobbss"

proj_UTMK = Proj(init='epsg:5179')
proj_WGS84 = Proj(init='epsg:4326')

class lidar_re_calc:
    def __init__(self):
        self.sub_gps_heading = rospy.Subscriber('/ublox_gps/navpvt', NavPVT, self.gps_heading_callback, queue_size=1)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size = 1)
        self.pose_sub = rospy.Subscriber('current_pose', Point, self.pose_callback, queue_size = 1) 
        
        self.gps_heading = 0
        self.obs_xy = np.empty((1, 3))
        self.sub_scan = []
        for i in range(810):
            self.sub_scan.append(0.0)
        self.pose = [0,0]
        self.obs = np.empty((1, 3))
        
    def gps_heading_callback(self, head):

        self.gps_heading = self.tf_heading_to_rad(float(head.heading))
        
    def scan_callback(self, scan):
        self.sub_scan = scan.ranges
        
    def pose_callback(self, data): # gps_pose + 보정된 헤딩
        self.pose = [data.x, data.y]
        
    def object_update(self):
        self.tf_tm(self.sub_scan, self.pose[0], self.pose[1], self.gps_heading)
        obs_clean = self.clean()
        for i in obs_clean:
            np.append(self.obs,np.array([i[0],i[1],0]),axis=0)

    def tf_tm(self, scan, x, y, heading) :
        self.obs_xy = np.empty((1, 3))
        resolution = 3
        T = [[cos(heading), -sin(heading), x], \
             [sin(heading),  cos(heading), y], \
             [      0     ,      0       , 1]]       # transform matrix, which is tranform lidar_xy coordinate to tm coordinate.
        
        last_scan_data = scan
        scan_data_for_search = []
        for i in range(0, 540):
            scan_data_for_search.append(last_scan_data[i+135])
            if np.isinf(scan_data_for_search[i]) or scan_data_for_search[i] < 0.1 or scan_data_for_search[i] > 15: # -90~90
                # scan_data_for_search[i] = 0``
                pass
            elif scan_data_for_search[i] <= 15:
                obs_x = scan_data_for_search[i]*sin(np.deg2rad(float(i)/resolution)) + 1.35
                obs_y = -scan_data_for_search[i]*cos(np.deg2rad(float(i)/resolution))
                self.obs_xy = np.append(self.obs_xy, [np.dot(T, np.transpose([obs_x, obs_y, 1]))], axis=0)
        self.obs_xy[:,2] = 0
    
    def clean(self):
        self.code = np.array(self.obs_xy)
        f_arr = []
        f_arr.append([self.code[0][0],self.code[0][1],0.0])

        for i in range(len(self.code)-1):
            m = abs(f_arr[-1][0] - self.code[i][0])
            n = abs(f_arr[-1][1] - self.code[i][1])
            if m >= 0.3 or n >= 0.3:
                f_arr.append([self.code[i][0],self.code[i][1],0.0]) 
            else:
                pass

        return f_arr  


def main():

    rospy.init_node("백파일 lidar 위치 보정")

    re_calc = lidar_re_calc()

    # points = []
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        re_calc.object_update()
        rate.sleep()
            
    PATH_ROOT=(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/"
    name = PATH_ROOT + re_calc_points_NPY
    np.save(name, re_calc.obs)
    plt.plot(re_calc.obs[:, 0], re_calc.obs[:, 1], 'ro')
    plt.show()
    
if __name__ == '__main__':
    main()
