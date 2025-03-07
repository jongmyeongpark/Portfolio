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
from macaron_06.msg import erp_write, ctrl_mode
from visualization_msgs.msg import Marker, MarkerArray

from math import *
# 모듈 import
from global_path import GlobalPath
from dwa_pre import DWA

WHERE = 1  # 맵 로딩

if WHERE == 1:  # 모라이 예선
    GLOBAL_PATH_NAME = "2024_kcity_pre_1019_fixed.npy" # "sangam_0429.npy" "m_0322_practice.npy"    2024_kcity_pre_1019_fixed
else:
    GLOBAL_PATH_NAME = ""

############################## gps 음영구간 테스트 할 때 ########################################
# etc에 있는 Dead_reckpning_class_signal.py 코드 켜고 하기


class PublishErp():
    def __init__(self):
        
        self.current_s_pub = rospy.Publisher('/current_s', Float32, queue_size=1)
        self.current_q_pub = rospy.Publisher('/current_q', Float32, queue_size=1)
        self.ryaw_pub = rospy.Publisher('/path_yaw', Float32, queue_size=1)
        self.ctrl_pub = rospy.Publisher('/ctrl_mode', ctrl_mode, queue_size=1)

        self.map_service = rospy.Service('MapLoad', MapLoad, self.map_loader)

        self.path_name = ""

    def pub_sq(self, s, q):
        self.current_s_pub.publish(s)
        self.current_q_pub.publish(q)

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
        self.pro_sub = rospy.Subscriber('/centroid', PointCloud2, self.lidar_callback, queue_size=1)
        self.speed_sub = rospy.Subscriber('/speed', Float32, self.speed_callback, queue_size=1)
        self.u_turn_sub = rospy.Subscriber('/track_line', PointCloud, self. u_turn_callback, queue_size=2)
        
        self.pose = [935508.503, 1915801.339]
        self.heading = 0.0
        self.lidar = []
        self.cur_speed = 0.0

        self.u_turn_line = []
        self.u_turn_line_global = []
        
    def pose_callback(self, data):
        self.pose = [data.x, data.y]
        self.heading = data.z
    
    def lidar_callback(self, data):
        self.lidar = []
        for point in point_cloud2.read_points(data, field_names=("x", "y", "z", "intensity")):
            self.lidar.append(point)

        if not self.lidar:
            return

        self.lidar = np.array(self.lidar)
        self.lidar = self.lidar[:, :3]

    def speed_callback(self, data):
        self.cur_speed = data.data

    def u_turn_callback(self, track):
        self.u_turn_line = []
        for point in track.points:
            self.u_turn_line.append([point.x, point.y])

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
        ctrl_algo = 1 # pid_pp_stanly
        ctrl_speed = 10
        s, q = GB.xy2sl(erp.pose[0], erp.pose[1], mode=0)
        if abs(q) > 4:
            s, q = GB.xy2sl(erp.pose[0], erp.pose[1], base_iter=300)


        processed = erp.tf2tm(erp.lidar, erp.pose[0], erp.pose[1], erp.heading)
        uturn_line = erp.tf2tm(erp.u_turn_line, erp.pose[0], erp.pose[1], erp.heading)
        # print("uturn line == ", uturn_line)
        # if len(uturn_line) != 0: erp.u_turn_line_global = uturn_line

        current_index = GB.getClosestSIndexCurS(s)
        pub.pub_sq(s, q)

        # print("uturn_lane == ", len(erp.u_turn_line_global))
        # # print(erp.heading)
        # # print(GB.ryaw[current_index])
        # print("============================")

        '''
        예선에서 처음 곡선 뒤에 유턴 전까지 PID로 갈 수 있는지 테스트 해봐야함
        만약 PID가 된다면 어디서 어디까지 되는지 s값 필요하고
        만약 PID가 안된다면 처음 우회전 하는 부분부터 MPC 10km/h로 달리면 됨
        유턴 끝나면 톨게이트 지날 때 까지는 PID쓰는게 좋아보임 10km/h로
        톨게이트 지나면 오른쪽으로 붙어야 함으로 DWA를 쓰는데 속도는 7km/h정도가 적합해 보임
        유턴 또한 내가 만든 DWA유턴 모드로 달리는데 속도는 7km/h가 좋아보임

        위의 속도는 임의로 내가 정한거라 테스트 하면서 괜찮은 속도를 찾아야 할 듯 
        '''
        ########### Tracking speed and algorithm ###########
        if 0 <= s < 60: # 처음
            ctrl_algo = 1 # pid_pp_stanly
            ctrl_speed = 18
        elif 60 <= s < 88.7: # 곡선 나오기 전까지
            ctrl_algo = 1 # pid_pp_stanly
            ctrl_speed = 15
        elif 88.7 <= s < 120: # 우회전
            ctrl_algo = 2 # mpc
            ctrl_speed = 8
        elif 120 <= s < 170: # 코너링 전 직진
            ctrl_algo = 1
            ctrl_speed = 12
        elif 170 <= s < 320: # 코너링
            ctrl_algo = 2
            ctrl_speed = 10
        elif 320 <= s < 380: # uturn 구간
            ctrl_algo = 2
            ctrl_speed = 5
        elif 380 <= s < 445: # uturn 구간 후 톨게이트 통과
            ctrl_algo = 1
            ctrl_speed = 10
        else: # 톨게이트 통과 후 레인 정렬
            ctrl_algo = 2
            ctrl_speed = 7

        ########### Tracking speed and algorithm ###########
        try:
            if 320 <= s < 380: # u-turn mode
                dwa.DWA(erp.pose[0], erp.pose[1], erp.heading, obs_xy=uturn_line, mode=2, current_s=current_index, cur_speed=erp.cur_speed, dwa_mode=2)[0]

            else:
                dwa.DWA(erp.pose[0], erp.pose[1], erp.heading, obs_xy=processed, mode=0, current_s=current_index, cur_speed=erp.cur_speed, dwa_mode=0)[0]
        except:
            pass

        pub.pub_ctrl(ctrl_algo, ctrl_speed)

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