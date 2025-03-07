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
from std_msgs.msg import Float32
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from macaron_06.srv import MapLoad, MapLoadResponse
from macaron_06.msg import erp_write, ctrl_mode

from math import *
# 모듈 import
from global_path import GlobalPath
from dwa_bon import DWA

WHERE = 1  # 맵 로딩

if WHERE == 1:  # 2024 본선
    GLOBAL_PATH_NAME = "2024_kcity_final_1013_fixed2.npy" #2024_kcity_final_1013_fixed2
else:
    GLOBAL_PATH_NAME = ""


class PublishErp():
    def __init__(self):
        
        self.current_s_pub = rospy.Publisher('/current_s', Float32, queue_size=1)
        self.current_q_pub = rospy.Publisher('/current_q', Float32, queue_size=1)
        self.map_service = rospy.Service('MapLoad', MapLoad, self.map_loader)
        self.ctrl_pub = rospy.Publisher('/ctrl_mode', ctrl_mode, queue_size=1)



        self.path_name = ""

    def pub_sq(self, s, q):
        self.current_s_pub.publish(s)
        self.current_q_pub.publish(q)
    
    def pub_target_speed(self, target_speed):
        self.target_speed_pub.publish(target_speed)

    def map_loader(self, request):
        return MapLoadResponse(self.path_name)
    
    def pub_ctrl(self, algo, speed):
        ctrl = ctrl_mode()
        ctrl.ctrl_algo = algo
        ctrl.ctrl_speed = speed

        self.ctrl_pub.publish(ctrl)
    
        
class SubscribeErp: 
    def __init__(self):
        self.pose_sub = rospy.Subscriber('/current_pose', Point, self.pose_callback, queue_size=1)
        self.pro_sub = rospy.Subscriber('/super_minjae_centroid', PointCloud2, self.lidar_callback, queue_size=1)
        self.speed_sub = rospy.Subscriber('/speed', Float32, self.speed_callback, queue_size=1)
        
        self.pose = [935508.503, 1915801.339]
        self.prev_pose = self.pose
        self.heading = 0.0
        self.processed = []
        self.cur_speed = 0.0
        
    def pose_callback(self, data):
        self.pose = [data.x, data.y]
        self.heading = data.z
    
    def lidar_callback(self, data):
        self.processed = []
        for point in point_cloud2.read_points(data, field_names=("x", "y", "z", "intensity")):
            self.processed.append(point)

        if not self.processed:
            return

        self.processed = np.array(self.processed)
        self.processed = self.processed[:, :3]

    def speed_callback(self, data):
        self.cur_speed = data.data

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
    dwa = DWA(gp_name)
    pub.path_name = gp_name
    print("macaron6.0")
    rospy.sleep(1)

    while not rospy.is_shutdown():
        if erp.pose != erp.prev_pose:
            s, q = GB.xy2sl(erp.pose[0], erp.pose[1])
        else:
            s, q = 0, 0

        processed = erp.tf2tm(erp.processed, erp.pose[0], erp.pose[1], erp.heading)
        
        current_index = GB.getClosestSIndexCurS(s)
        
        pub.pub_sq(s, q)

        ss = ''
        ########### Tracking speed and algorithm ###########
        if 0 <= s < 62 or 265 <= s < 335 or 637.4 <= s < 712.3 or 887.2 <= s < 1035: # 출발 ~ A, 1번 신호등 끝 ~ 대형, 고속주행, 6번 ~ 7번
            ctrl_algo = 1 # pid_pp_stanly
            ctrl_speed = 20
            ss= 'a'
        elif 1083.7 <= s: # 
            ctrl_algo = 1 # pid_pp_stanly
            ctrl_speed = 15
            ss= 'b'
        
        elif 803.4 <= s < 887.2: # 
            ctrl_algo = 1 # pid_pp_stanly
            ctrl_speed = 12
            ss= 'b'

        elif 213.4 <= s < 225.1 or 581.9 <= s < 600.9 or 754.1 <= s < 773.8 or 506 <= s < 522.1: # 신호등 가는 길목 멈추는 지점 까지
            ctrl_algo = 1
            ctrl_speed = 10
            ss= 'c'

        elif 62 <= s < 95.4 or 225.1 <= s < 265 or 522.1 <= s < 581.9: # 신호등 및 신호등 좌회전, B 끝나고 직선 전까지
            ctrl_algo = 2 # mpc
            ctrl_speed = 10
            ss= 'd'

        elif 95.4 <= s < 128 or 145.5 <= s < 213.4 or 390 <= s < 432 or 477.9 <= s < 506 or 600.9 <= 637.4 or 712.3 <= s < 754.1 or 773.8 <= s < 803.4: # mini앞 좌회전, 정지 우회전, 고속 끝 우회전
            ctrl_algo = 2 # mpc
            ctrl_speed = 7
            ss= 'e'

        elif 128 <= s < 145.5 or 335 <= s < 390 or 432 <= s < 477.9 or 1035 <= s < 1083: # All mission
            ctrl_algo = 2 # mpc
            ctrl_speed = 5
            ss= 'f'

        else:
            ctrl_algo = 2 # mpc
            ctrl_speed = 10
            ss = 'g'

            
        ########### Tracking speed and algorithm ###########

        ########### Tracking Path like DWA ###########
        if 128 <= s < 145.5: # mini_obs mode
            dwa.DWA(erp.pose[0], erp.pose[1], erp.heading, obs_xy=processed, mode=1, current_s=current_index, cur_speed=erp.cur_speed, dwa_mode=1)

        elif 340 <= s < 390: # big_obs mode
            dwa.DWA(erp.pose[0], erp.pose[1], erp.heading, obs_xy=processed, mode=2, current_s=current_index, cur_speed=erp.cur_speed, dwa_mode=2)

        else:
            dwa.DWA(erp.pose[0], erp.pose[1], erp.heading, obs_xy=processed, mode=0, current_s=current_index, cur_speed=erp.cur_speed, dwa_mode=0)


        ########### Tracking Path like DWA ###########
        
        pub.pub_ctrl(ctrl_algo, ctrl_speed)

        # os.system('clear')
        # print('s: ', s)
        # print('q: ', q)
        # print(f'state: {ss}')
        # print(erp.pose[0], erp.pose[1])
        # print(f'{current_index}/{len(GB.rx)}')
        # print("===============")
        
        rate.sleep()
        
if __name__ == '__main__':
    rospy.init_node("state_node", anonymous=True)
    main()