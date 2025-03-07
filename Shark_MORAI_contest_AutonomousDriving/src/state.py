#!/usr/bin/env python3
#-*-coding:utf-8-*-

# 2024버전 state

# Python packages

import rospy
import os, sys
import numpy as np
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/src/missions")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/src/sensor")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/src/path_planning")
import pickle

# message 파일
from std_msgs.msg import Float32, Bool, Int8
from geometry_msgs.msg import Point, Point32
from sensor_msgs.msg import PointCloud2, PointCloud
from morai_2.srv import MapLoad, MapLoadResponse
from math import *
# 모듈 import
from path_planning.global_path import GlobalPath
from path_planning.dwa import DWA

GLOBAL_PATH_NAME = "morai_2_gpg.npy"  # "morai_0805.npy"    


class PublishErp():
    def __init__(self):
        
        self.current_s_pub = rospy.Publisher('/current_s', Float32, queue_size=1)
        self.current_q_pub = rospy.Publisher('/current_q', Float32, queue_size=1)
        self.target_speed_pub = rospy.Publisher('/target_speed', Float32, queue_size=1)

        self.map_service = rospy.Service('MapLoad', MapLoad, self.map_loader)

        self.path_name = ""

    def pub_sq(self, s, q):
        self.current_s_pub.publish(s)
        self.current_q_pub.publish(q)
    
    def pub_target_speed(self, target_speed):
        self.target_speed_pub.publish(target_speed)

    def map_loader(self, request):
        return MapLoadResponse(self.path_name)
        
class SubscribeErp: 
    def __init__(self):
        self.pose_sub = rospy.Subscriber('/current_pose', Point, self.pose_callback, queue_size=1)
        self.est_pose_sub = rospy.Subscriber('/estimate_pose', Point, self.estimate_pose_callback, queue_size=1)
        self.reckoning_sub=rospy.Subscriber('/reckoning_pose', Point, self.reckoning_pose_callback, queue_size=1) # x,y는 tm좌표, z에 들어가 있는 값이 heading
        self.local_sub = rospy.Subscriber('object3D', PointCloud, self.obs_callback, queue_size=1)

        
        self.pose = [0, 0]
        self.est_pose = [0, 0]
        self.rec_pose = [0, 0]
        self.heading = 0.0

        self.obs = []
        
    def pose_callback(self, data):
        self.pose = [data.x, data.y]
        self.heading = data.z

    def estimate_pose_callback(self, pose):
        self.est_pose = [pose.x, pose.y]

    def reckoning_pose_callback(self, pose):
        self.rec_pose = [pose.x, pose.y]

    def obs_callback(self, data):
        self.obs = []
        for point in data.points:
            p = Point32()
            p.x = point.x
            p.y = point.y
            p.z = 0
            self.obs.append([p.x, p.y])

def main():
    rate = rospy.Rate(10)
    pub = PublishErp()
    erp = SubscribeErp()
    
    # Global Path 선언
    PATH_ROOT=(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))+"/path/npy_file/"
    gp_name = PATH_ROOT + GLOBAL_PATH_NAME
    GB = GlobalPath(gp_name)
    pub.path_name = gp_name
    dwa = DWA(GB)
    
    print("morai")
    rospy.sleep(1)
    mode = ''
    erp_pose = []
    while not rospy.is_shutdown():
        if erp.est_pose and erp.pose[0] <= 0 or erp.pose[1] < 1800000:
            mode = 'slam'
            erp_pose = erp.est_pose 
        else:
            mode = 'gps'
            erp_pose = erp.pose

        s, q = GB.xy2sl(erp_pose[0], erp_pose[1])
        if abs(q) > 3:
            s, q = GB.xy2sl(erp_pose[0], erp_pose[1])
        
        current_index = GB.getClosestSIndexCurS(s)
        pub.pub_sq(s, q)

        if 174 < s <= 215:
            mode = 'dwa1'
            dwa.DWA(erp_pose[0], erp_pose[1], erp.heading, obs_xy=erp.obs, mode = 0, current_index=current_index, dwa_mode = 1)
        elif 220 < s <= 314:
            mode = 'dwa2'
            dwa.DWA(erp_pose[0], erp_pose[1], erp.heading, obs_xy=erp.obs, mode = 0, current_index=current_index, dwa_mode = 2)
        else:
            mode = 'dwa0'
            if len(dwa.obj_data_list_x) != 1:
                dwa.delete_global_obs()
            dwa.DWA(erp.pose[0], erp.pose[1], erp.heading, obs_xy=erp.obs, mode = 0, current_index=current_index, dwa_mode = 0)


        os.system('clear')
        print('s: ', s)
        print('q: ', q)
        print(f'Mode: {mode}')
        print(erp_pose[0], erp_pose[1])
        # if mode == 'slam':
        #     print(f'reckoning_pose: {erp.rec_pose[0]}, {erp.rec_pose[1]}')
        #     print(f'slam_pose: {erp.est_pose[0]}, {erp.est_pose[1]}')
        # print(f'{current_index}/{len(GB.rx)}')
        # deg = np.rad2deg(GB.ryaw[current_index])
        # rad = GB.ryaw[current_index]
        # if rad < 0: rad+=2*np.pi
        # print(rad)
        print("===============")

        rate.sleep()
        
if __name__ == '__main__':
    rospy.init_node("state_node", anonymous=True)
    main()