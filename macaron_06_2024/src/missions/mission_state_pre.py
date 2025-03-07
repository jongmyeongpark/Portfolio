#!/usr/bin/env python3
#-*-coding:utf-8-*-

# 라이브러리 임포트
import time
import rospy
import os, sys
import numpy as np
from math import sqrt
# 파일 경로 추가
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/sensor")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/path_planning")

# 메세지 임포트
from macaron_06.srv import MapLoad
from std_msgs.msg import Float32, Int8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Point32
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, CompressedImage
from macaron_06.msg import erp_read, lidar_info
from path_planning.global_path import GlobalPath

# 미션 파일 임포트
from mission_tunnel import MissionTunnel
from mission_fail import MissionFail


# bagfile 테스트 시 사용
# GB_PATH에서 사용자 이름(takrop)를 자신의 경로로 설정해서 실행하기
BAGFILE = False
GB_PATH = "/home/takrop/catkin_ws/src/macaron_06/path/npy_file/path/"
GB = "2024_kcity_pre.npy"

NO_GPS_INIT_DIS = 10

class MissionState():
    def __init__(self):
        
        self.gp_name = ""
        self.gp_path = None
        self.pose_sub = rospy.Subscriber('/current_pose', Point, self.pose_callback, queue_size=1)# 출발(기어 바꾸고 자동 모드 변경, 규정집 보니까 삭제해도 될 듯)
        self.current_s_sub = rospy.Subscriber('/current_s', Float32, self.s_callback, queue_size=1)
        self.current_q_sub = rospy.Subscriber('/current_q', Float32, self.q_callback, queue_size=1)
        self.cluster_sub = rospy.Subscriber('/cluster', lidar_info, self.cluster_callback, queue_size=2)
        self.centroid_sub = rospy.Subscriber('/centroid', PointCloud2, self.centroid_callback, queue_size=1)
        self.lane_detection_sub = rospy.Subscriber('/minmax_ransac', PointCloud2, self.lane_data_callback, queue_size=1)
        self.cluster_sub = rospy.Subscriber('/cluster', lidar_info, self.lidar_info_callback ,queue_size=1)
        
        self.map_client = rospy.ServiceProxy('MapLoad', MapLoad)
        
        # 미션 모음   
        self.mission_tunnel = MissionTunnel()

        # 멤버 변수
        self.current_pose = Point()

        self.current_s = 0.0
        self.current_q = 0.0
        self.max_index = 0

        self.cluster_data = lidar_info()
        self.lane_data = PointCloud2()
        self.centroid = []
        self.tunnel_left_dis = 0
        self.tunnel_right_dis = 0

    def map_loader(self):
        response = self.map_client("")
        if response != "":
            self.gp_name = response.response
            print(self.gp_name)
    
    def pose_callback(self, pose):
        if pose.x > 0 and pose.y > 0:
            self.current_pose = pose

    def s_callback(self, s):
        self.current_s = s.data 

    def q_callback(self, q):
        self.current_q = q.data

    def cluster_callback(self, cluster: lidar_info):
        self.cluster_data = cluster

    def centroid_callback(self, centroids: PointCloud2):
        obstacles = []
        for centroid in point_cloud2.read_points(centroids, field_names=("x", "y", "z", "intensity")):
            obstacles.append(centroid)

        if len(obstacles) == 0:
            return
        else:
            obstacles = np.array(obstacles)
            self.centroid = obstacles[:, :]

    def lidar_info_callback(self, cluster: lidar_info):
        self.tunnel_left_dis = cluster.maxY
        self.tunnel_right_dis = cluster.minY

    def lane_data_callback(self, lane_data):
        self.lane_data = lane_data

    def generate_map(self):
        self.gp_path = GlobalPath(self.gp_name)

    def mission_activate(self):
        s = self.current_s

        if s > 495 or abs(self.current_q) > 10:
            try:
                self.mission_tunnel.activate_lidar_lane(self.lane_data, self.centroid, self.tunnel_left_dis, self.tunnel_right_dis)
            except:
                pass


        

def main():
    rospy.init_node('mission_state', anonymous=True)
    MS = MissionState()
    rate = rospy.Rate(10)

    # while (MS.gp_name == ""):
    #     try:
    #         os.system('clear')
    #         print("Loading")
    #         if BAGFILE:
    #             MS.gp_name = GB_PATH + GB
    #         else: 
    #             MS.map_loader()
    #         MS.generate_map()
    #         print("Map loading completed")
    #     except: time.sleep(1)

    print('activated')
    while not rospy.is_shutdown():
        MS.mission_activate()

        rate.sleep()
    
    # MS.mission_tunnel.Dead_reckoning.Dead_reckoning(4)
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass