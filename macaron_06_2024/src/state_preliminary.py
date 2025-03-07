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
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Point, Point32
from sensor_msgs.msg import PointCloud2, PointCloud
from macaron_06.srv import MapLoad, MapLoadResponse
from macaron_06.msg import erp_write

from math import *
# 모듈 import
from global_path import GlobalPath
from dwa import DWA

WHERE = 1  # 맵 로딩

if WHERE == 1:  # 모라이 예선
    GLOBAL_PATH_NAME = "kcity_0810_tunnel.npy" # "sangam_0429.npy" "m_0322_practice.npy"    
else:
    GLOBAL_PATH_NAME = ""

############################## gps 음영구간 테스트 할 때 ########################################
# etc에 있는 Dead_reckpning_class_signal.py 코드 켜고 하기


class PublishErp():
    def __init__(self):
        
        self.current_s_pub = rospy.Publisher('/current_s', Float32, queue_size=1)
        self.current_q_pub = rospy.Publisher('/current_q', Float32, queue_size=1)
        self.target_speed_pub = rospy.Publisher('/target_speed', Float32, queue_size=1)
        self.ryaw_pub = rospy.Publisher('/path_yaw', Float32, queue_size=1)

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
        # self.slam_pose_sub = rospy.Subscriber('/slam_pose', Point, self.slam_pose_callback, queue_size=1)
        self.est_pose_sub = rospy.Subscriber('/estimate_pose', Point, self.estimate_pose_callback, queue_size=1)
        # self.dr_pose_sub = rospy.Subscriber('Dead_reckoning_pose', Point, self.dr_pose_callback, queue_size=1)
        self.pro_sub = rospy.Subscriber('processed', PointCloud2, self.pro_callback, queue_size=1)
        self.speed_sub = rospy.Subscriber('/speed', Float32, self.speed_callback, queue_size=1)
        # self.speed_sub = rospy.Subscriber('/Dead_reckoning_speed', Float32, self.dr_speed_callback, queue_size=1)
        
        self.pose = [935508.503, 1915801.339]
        self.dr_pose = [935508.503, 1915801.339] # dead_reckoning x,y
        # self.slam_pose = [935508.503, 1915801.339] # slam x,y
        self.heading = 0.0
        self.dr_heading = 0.0 # dead_reckoning heading
        # self.slam_heading = 0.0 # slam heading
        self.obs_xy_cpp = []
        self.processed = []
        self.cur_speed = 0.0
        self.dr_cur_speed = 0.0 # dead_reckoning speed
        self.slam_cur_speed = 0.0 # slam speed

        self.dr_flag = 0 # dead_reckoning flag
        self.slam_flag = 0 # slam flag
        
    def pose_callback(self, data):
        self.pose = [data.x, data.y]
        self.heading = data.z

    def estimate_pose_callback(self, pose):
        self.est_pose = [pose.x, pose.y]

    def dr_pose_callback(self, data): # dead_reckoning pose callback
        self.dr_flag = 1
        if self.dr_flag == 1:
            self.dr_pose = [data.x, data.y]
            self.dr_heading = data.z
            print("dead_reckoning ok")

    def slam_pose_callback(self, data): # slam pose callback
        self.slam_flag = 1
        if self.slam_flag == 1:
            self.slam_pose = [data.x, data.y]
            self.slam_heading = data.z
            print("slam ok")
    
    def pro_callback(self, data):
        self.processed = []
        for point in point_cloud2.read_points(data, field_names=("x", "y", "z", "intensity")):
            self.processed.append(point)

        self.processed = np.array(self.processed)
        self.processed = self.processed[:, :3]

    def speed_callback(self, data):
        self.cur_speed = data.data

    def dr_speed_callback(self, data): # dead_reckoning speed callback
        self.dr_cur_speed = data.data

    def tf2tm(self, no_z_points, x, y, heading):
        obs_tm=np.empty((1,3))
        T = [[cos(heading), -1*sin(heading), x], \
             [sin(heading),    cos(heading), y], \
             [      0     ,        0       , 1]] 
        for point in no_z_points:
            obs_tm = np.append(obs_tm,[np.dot(T,np.transpose([point[0]+1, point[1],1]))],axis=0) # point[0] -> 객체를 subscribe할 수 없음 오류
        obs_tm[:,2]=0
        obs_tm = np.delete(obs_tm, (0), axis = 0) 
        return obs_tm

def main():
    global WHERE
    rate = rospy.Rate(10)
    pub = PublishErp()
    erp = SubscribeErp()

    # Global Path 선언
    PATH_ROOT=(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))+"/path/npy_file/path/"
    gp_name = PATH_ROOT + GLOBAL_PATH_NAME
    try:   # ########## 피클 파일 불러오기
        GB = pickle.load(open(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))+"/path/pkl_file/" +
                              GLOBAL_PATH_NAME[0:len(GLOBAL_PATH_NAME)-4] + '.pkl', 'rb'))
    except FileNotFoundError:
        GB = GlobalPath(gp_name)

    GB = GlobalPath(gp_name)
    dwa = DWA(GB)
    pub.path_name = gp_name
    print("macaron6.0")
    rospy.sleep(1)
    mode = ''
    erp_pose = []
    while not rospy.is_shutdown():
        
        if erp.est_pose and erp.pose[0] <= 0 or erp.pose[1] < 1800000: # gps 교란이 어떻게 되는지 알아야 함
            mode = 'slam'
            erp_pose = erp.est_pose 
        else:
            mode = 'gps'
            erp_pose = erp.pose

        s, q = GB.xy2sl(erp_pose[0], erp_pose[1])
        # dr_s,_ = GB.xy2sl(erp.dr_pose[0], erp.dr_pose[1]) # dead_reckoning (s, q)
        # slam_s,_ = GB.xy2sl(erp.slam_pose[0], erp.slam_pose[1]) # slam (s, q)

        processed = erp.tf2tm(erp.processed, erp_pose[0], erp_pose[1], erp.heading)
        
        current_index = GB.getClosestSIndexCurS(s)
        # dr_current_index = GB.getClosestSIndexCurS(dr_s) # dead_reckoning index
        # slam_current_index = GB.getClosestSIndexCurS(slam_s) # slam index
        pub.pub_sq(s, q)
        pub.pub_target_speed(GB.rvel[current_index])

        if True: # erp.dr_flag == 0:
            dwa.DWA(erp.pose[0], erp.pose[1], erp.heading, obs_xy=processed, mode=0, current_s=current_index, cur_speed=erp.cur_speed, dwa_mode=0)
        # 장애물이 없고 global_path를 tracking 하는 경우
        elif False: # erp.dr_flag == 1 :
            dwa.DWA(erp.dr_pose[0], erp.dr_pose[1], erp.dr_heading, obs_xy=processed, mode=0, current_s=current_index, cur_speed=erp.dr_cur_speed, dwa_mode=0)
        elif False: # erp.dr_flag == 1 :
            dwa.DWA(erp.slam_pose[0], erp.slam_pose[1], erp.slam_heading, obs_xy=slam_processed, mode=0, current_s=slam_current_index, cur_speed=erp.cur_speed, dwa_mode=0)
        # 미니 정적 장애물인 경우 좌우 1.5m
        elif False: # erp.dr_flag == 0 or erp.slam_flag == 0: 
            dwa.DWA(erp.pose[0], erp.pose[1], erp.heading, obs_xy=processed, mode=0, current_s=current_index, cur_speed=erp.cur_speed, dwa_mode=2)
        # 대형 정적 장애물인 경우 좌 0.1m 우 2.3m
        elif False:
            dwa.DWA(erp.pose[0], erp.pose[1], erp.heading, obs_xy=processed, mode=1, current_s=current_index, cur_speed=erp.cur_speed, dwa_mode=1)
        # dead_reckoning 미니 정적 장애물 좌우 1.5m
        elif False: # erp.dr_flag == 1:
            dwa.DWA(erp.dr_pose[0], erp.dr_pose[1], erp.dr_heading, obs_xy=dr_processed, mode=0, current_s=dr_current_index, cur_speed=erp.dr_cur_speed, dwa_mode=2)
        # slam 미니 정적 장애물 좌우 1.5m
        elif False: #erp.slam_flag == 1:
            dwa.DWA(erp.slam_pose[0], erp.slam_pose[1], erp.slam_heading, obs_xy=slam_processed, mode=0, current_s=slam_current_index, cur_speed=erp.slam_cur_speed, dwa_mode=2)

        os.system('clear')
        print('s: ', s)
        print('q: ', q)
        print(erp.pose[0], erp.pose[1])
        print(f'{current_index}/{len(GB.rx)}')
        print("===============")

        ################ dead_reckoning s, q, cur_index print ################
        # os.system('clear')
        # print('s: ', dr_s)
        # print('q: ', dr_q)
        # print(erp.dr_pose[0], erp.dr_pose[1])
        # print(f'{ng_current_index}/{len(GB.rx)}')
        # print("===============")

        ################ slam s, q, cur_index print ################
        # os.system('clear')
        # print('s: ', slam_s)
        # print('q: ', slam_q)
        # print(erp.slam_pose[0], erp.slam_pose[1])
        # print(f'{slam_current_index}/{len(GB.rx)}')
        # print("===============")
        
        rate.sleep()
        
if __name__ == '__main__':
    rospy.init_node("state_node", anonymous=True)
    main()