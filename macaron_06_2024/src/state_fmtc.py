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
    GLOBAL_PATH_NAME = "delivery_manhae_last.npy" #"2024_kcity_pre.npy"
else:
    GLOBAL_PATH_NAME = ""


class PublishErp():
    def __init__(self):
        
        self.current_s_pub = rospy.Publisher('/current_s', Float32, queue_size=1)
        self.current_q_pub = rospy.Publisher('/current_q', Float32, queue_size=1)
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
        # self.pro_sub = rospy.Subscriber('/super_minjae_processed', PointCloud2, self.lidar_callback, queue_size=1)
        self.pro_sub = rospy.Subscriber('/centroid', PointCloud2, self.lidar_callback, queue_size=1)
        self.speed_sub = rospy.Subscriber('/speed', Float32, self.speed_callback, queue_size=1)

        self.ctrl_pub = rospy.Publisher('/ctrl_mode', ctrl_mode, queue_size=1)
        
        self.pose = [935508.503, 1915801.339]
        self.prev_pose = self.pose
        self.heading = 0.0
        self.processed = []
        self.cur_speed = 0.0

        self.ctrl_algo = 0
        self.ctrl_speed = 0

    def pub_ctrl(self, algo, speed):
        ctrl = ctrl_mode()
        ctrl.ctrl_algo = algo
        ctrl.ctrl_speed = speed

        self.ctrl_pub.publish(ctrl)

        
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
    rate = rospy.Rate(25)
    pub = PublishErp()
    erp = SubscribeErp()
    ctrl = ctrl_mode()

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

        if 0 <= s: #< 26.9 or 68 <= s < 80 or 188 <= s < 275 or 613.5 <= s < 680: # A전, 미니~대형, 좌회전~주차
            ctrl_algo = 2 # pid_pp_stanly
            ctrl_speed = 5
        # elif 420 <= s < 520: # 가드레일 고속
        #     ctrl_algo = 1
        #     ctrl_speed = 10
        # elif 26.9 <= s < 68 or 138 <= s < 162 or 275 <= s < 360 or 697 < s: # A, mini, big, parking
        #     ctrl_algo = 2 # mpc
        #     ctrl_speed = 5
        # else: # 좌회전
        #     ctrl_algo = 2 # mpc
        #     ctrl_speed = 10

        # if 138 <= s < 162: # mini_obs mode
        #     dwa.DWA(erp.pose[0], erp.pose[1], erp.heading, obs_xy=processed, mode=1, current_s=current_index, cur_speed=erp.cur_speed, dwa_mode=1)

        # elif 278 <= s < 306: # big_obs mode
        #     dwa.DWA(erp.pose[0], erp.pose[1], erp.heading, obs_xy=processed, mode=2, current_s=current_index, cur_speed=erp.cur_speed, dwa_mode=2)

        if True:
            dwa.DWA(erp.pose[0], erp.pose[1], erp.heading, obs_xy=processed, mode=0, current_s=current_index, cur_speed=erp.cur_speed, dwa_mode=0)
        
        erp.pub_ctrl(ctrl_algo, ctrl_speed)

        os.system('clear')
        print('s: ', s)
        print('q: ', q)
        print(erp.pose[0], erp.pose[1])
        print(f'{current_index}/{len(GB.rx)}')
        print("===============")
        
        rate.sleep()
        
if __name__ == '__main__':
    rospy.init_node("state_node", anonymous=True)
    main()