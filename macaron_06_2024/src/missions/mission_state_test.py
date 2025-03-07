#!/usr/bin/env python3
#-*-coding:utf-8-*-

# 라이브러리 임포트
import time
import rospy
import os, sys
import pickle
import numpy as np
import cv2
from cv_bridge import CvBridgeError
from math import cos, sin , sqrt
from tf.transformations import euler_from_quaternion
from sensor_msgs import point_cloud2
import matplotlib.pyplot as plt
from macaron_06.msg import lidar_info

# 파일 경로 추가
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/sensor")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/path_planning")

# 메세지 임포트
from macaron_06.srv import MapLoad, MapLoadRequest
from std_msgs.msg import Float32 , Header , Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Point32
from sensor_msgs.msg import PointCloud, CompressedImage, PointCloud2 ,PointField
from macaron_06.msg import erp_read
from path_planning.global_path import GlobalPath
from sensor_msgs import point_cloud2
import threading

# 미션 파일 임포트
# from mission_no_gps import MissionNoGPS
from mission_fail import MissionFail
from path_planning.mission_parking_2024 import mission_parking , parking_detection
from sensor.parking import ParkingDetector

# bagfile 테스트 시 사용
# GB_PATH에서 사용자 이름(takrop)를 자신의 경로로 설정해서 실행하기
BAGFILE = True
GB_PATH = "/home/macaron/catkin_ws/src/macaron_06/path/npy_file/path/"
GB = "manhae_06.28_c.npy"

#parameters for 주차 공간 detect
global_path_npy="" ##주차공간 판단을 위해 생성된 경로

MAX_X= 3.4

MIN_X= -2.8

MIN_Y= -4.5#-3.3

goal_point= [[955792.223178623,1951218.2729486357],[955796.4560314512,1951220.325041688],[955800.8293687594,1951222.2433917387]]
                
STOP_TIME = 3
            


class MissionState():
    def __init__(self):
        self.gp_name = ""
        self.gp_path = None
        self.pose_sub = rospy.Subscriber('/current_pose', Point, self.pose_callback, queue_size=1)# 출발(기어 바꾸고 자동 모드 변경, 규정집 보니까 삭제해도 될 듯)
        self.current_s_sub = rospy.Subscriber('/current_s', Float32, self.s_callback, queue_size=1)
        self.current_q_sub = rospy.Subscriber('/current_q', Float32, self.q_callback, queue_size=1)
        self.erp_sub = rospy.Subscriber('/erp_read', erp_read, self.erp_callback, queue_size=1)
        self.lidar_odom_sub = rospy.Subscriber('/lidar_odom', Odometry, self.lidar_odom_callback, queue_size=1)
        
        self.imu_yaw_sub = rospy.Subscriber('/imu_yaw', Float32, self.imu_yaw_callback, queue_size=1)
        self.map_client = rospy.ServiceProxy('MapLoad', MapLoad)

        self.centroid_pub = rospy.Publisher("/centroid_filtered", PointCloud2, queue_size=10)
        self.stop_pub = rospy.Publisher('/stop', Bool, queue_size=1)
        
        # 미션 모음   
        # self.mission_no_gps = MissionNoGPS()
        self.mission_parking = mission_parking()
        self.mission_fail = MissionFail()
        self.parking_detection = parking_detection(global_path_npy)
        self.detector = ParkingDetector()

        # 멤버 변수
        self.current_pose = Point()
        self.imu_heading = 0.0
        self.imu_init = False

        self.current_s = 0.0
        self.current_q = 0.0
        self.max_index = 0
        
        self.lidar_odom = Odometry()
        self.slam_init_heading = 0.0
        self.slam_init = False

        self.mission_state = 0

        self.E_stop = False
        self.fail = False
        
        self.detection_flag = False
        self.search_step = 0
        self.margin = 0.5
        self.check_time = 0.0

        self.counted = 0
        self.check_flag = False
        self.find_flag = False

    def map_loader(self):
        response = self.map_client("")
        if response != "":
            self.gp_name = response.response
            print(self.gp_name)

    def erp_callback(self, data):
        self.E_stop = data.read_E_stop
    
    def pose_callback(self, pose):
        if pose.x > 0 and pose.y > 0:
            self.current_pose = pose

    def s_callback(self, s):
        self.current_s = s.data 

    def q_callback(self, q):
        self.current_q = q.data
    
    def lidar_odom_callback(self, odom):
        self.lidar_odom = odom
        if self.imu_heading != 0 and self.slam_init_heading == 0.0:
            self.slam_init_heading = self.imu_heading
            self.slam_init = True

    def imu_yaw_callback(self, yaw):
        self.imu_heading = yaw.data
        self.imu_init = True


    def generate_map(self):
        self.gp_path = GlobalPath(self.gp_name)

    def fail_check(self):
        self.fail = self.mission_fail.check(E_stop=self.E_stop, fail_pose=self.current_pose)
        
    def step_check(self):
        min_dis=10000
        check_ind=5
        for ind, point in enumerate(goal_point):
            dis = sqrt((point[0] - self.current_pose.x)**2 +(point[1] - self.current_pose.y)**2 )
            if dis < min_dis:
                min_dis = dis
                check_ind=ind
                
        return check_ind
        
        

        

    def mission_activate(self):
        
        self.fail_check()
        

        # SLAM 위치 추정
        # if 0 <= self.current_s < 100:
        #     pass

        # elif 100 <= self.current_s < 200:
        #     pass
        # if self.slam_init and self.imu_init or True:
        #     self.current_pose.z = self.imu_heading
            
        #     if not self.mission_no_gps.init_flag:        
        #         self.mission_no_gps.init_setting(odom_init=self.lidar_odom, pose_init=self.current_pose, slam_heading_init = self.slam_init_heading)
            
        #     print(self.current_pose.x, self.current_pose.y, self.current_pose.z)
        #     self.mission_no_gps.activate(odom=self.lidar_odom, heading=self.current_pose.z)


        if not self.detection_flag:

                    
            self.search_step=self.step_check()
            
            print(f"Now Searching Parking{self.search_step + 1}....")

            
            
            if not self.check_flag:
                
                
                is_finded = self.detector.is_finded

                
                if is_finded: 
                    print("detected")
                    
                    self.check_flag = True
                   
                    self.counted += 1

                else:
                    pass
                    
                    
            else:
                is_finded = self.detector.is_finded

                if is_finded and self.counted < 3:
                    print("detected")
                    self.counted += 1

                elif is_finded and self.counted >= 3:
                    print("detected")
                    self.detection_flag = True

                    print(f"parking {self.search_step +1} is possible")
                    

                else:
                    self.counted = 0
                    self.check_flag = False


            print("============================")
                    

                
        else:            
        
            
            if self.mission_parking.parking_status == 0:
                self.mission_parking.find_point(self.current_pose, self.search_step)
                
            elif self.mission_parking.parking_status == 1:
                self.mission_parking.move2start(self.current_pose,self.search_step)
                
            elif self.mission_parking.parking_status == 2:
                self.mission_parking.start2backpoint(self.current_pose, self.search_step)
                    
            elif self.mission_parking.parking_status == 3:
                print(f"STOP!!!! for {STOP_TIME}seconds")
                self.mission_parking.change_parking_status()

                self.check_time = rospy.Time.now().to_sec()
            
            elif self.mission_parking.parking_status == 4:
                time_passed = rospy.Time.now().to_sec() - self.check_time

                print(f"time_passed : {time_passed}")
                self.stop_pub.publish(True)

                if time_passed >= STOP_TIME:
                    self.mission_parking.change_parking_status()
                    print("Back to global path")
            
            elif self.mission_parking.parking_status == 5 :
                self.mission_parking.back2glob(self.current_pose)


            else:
                print("Mission Parking comeplete!!!")
                
            

def main():
    rospy.init_node('mission_state', anonymous=True)
    MS = MissionState()
    rate = rospy.Rate(10)

    while (MS.gp_name == ""):
        try:
            os.system('clear')
            print("Loading")
            if BAGFILE:
                MS.gp_name = GB_PATH + GB
            else: 
                MS.map_loader()
            MS.generate_map()
            print("Map loading completed")
        except: time.sleep(1)

    print('activated')
    while not rospy.is_shutdown():
        MS.mission_activate()



        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass