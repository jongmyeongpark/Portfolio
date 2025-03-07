#!/usr/bin/env python3
#-*-coding:utf-8-*-

# 라이브러리 임포트
import time
import rospy
import os, sys
import numpy as np
import cv2
from cv_bridge import CvBridgeError


# 파일 경로 추가
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/sensor")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/path_planning")

# 메세지 임포트
from macaron_06.srv import MapLoad, MapLoadRequest
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud, CompressedImage
from macaron_06.msg import erp_read
from path_planning.global_path import GlobalPath

# 미션 파일 임포트
from mission_fail import MissionFail
from sensor.final.delivery_judge import DeliveryJudge
from sensor.final.mission_traffic_kcity import mission_traffic_kcity
from sensor.final.calibration_4final_no_img import LidarCameraCalibration
from mission_parking import mission_parking
from mission_right_stop import MissionRightStop


# bagfile 테스트 시 사용
# GB_PATH에서 사용자 이름(takrop)를 자신의 경로로 설정해서 실행하기
BAGFILE = False
GB_PATH = "/home/macaron/catkin_ws/src/macaron_06/path/npy_file/path/"
GB = "manhae_06.28_c.npy"

parking_heading =[0.379,0.379,0.379]
goal_point=[[955796.9187568901,1951219.324092402],[955801.4069710096,1951221.2306428265],[955802.5400200367,1951221.7128280522]]

PARKING_LEN = 5.35
PARKING_WIDTH = 3.25
class MissionState():
    def __init__(self):
        self.gp_name = ""
        self.gp_path = None
        self.pose_sub = rospy.Subscriber('/current_pose', Point, self.pose_callback, queue_size=1)# 출발(기어 바꾸고 자동 모드 변경, 규정집 보니까 삭제해도 될 듯)
        self.current_s_sub = rospy.Subscriber('/current_s', Float32, self.s_callback, queue_size=1)
        self.current_q_sub = rospy.Subscriber('/current_q', Float32, self.q_callback, queue_size=1)
        self.erp_sub = rospy.Subscriber('/erp_read', erp_read, self.erp_callback, queue_size=1)

        self.target_speed_pub = rospy.Publisher('/mission_target_speed', Float32, queue_size=1)
        self.map_client = rospy.ServiceProxy('MapLoad', MapLoad)
        
        # 미션 모음   
        self.mission_fail = MissionFail()
        self.mission_delivery = DeliveryJudge()
        self.mission_traffic = mission_traffic_kcity()
        self.mission_parking = mission_parking(PARKING_LEN,PARKING_WIDTH,goal_point,parking_heading)
        self.mission_right_stop = MissionRightStop()
        self.calib_point = LidarCameraCalibration()

        # 멤버 변수
        self.current_pose = Point()

        self.current_s = 0.0
        self.current_q = 0.0
        self.max_index = 0

        self.mission_state = 0
        self.delivery_fail = False
        self.parking_fail = False

        self.E_stop = False
        self.fail = False

    def map_loader(self):
        response = self.map_client("")
        if response != "":
            self.gp_name = response.response
            print(self.gp_name)

    def erp_callback(self, data: erp_read):
        self.E_stop = data.read_E_stop
        self.auto = data.read_AorM
    
    def pose_callback(self, pose):
        if pose.x > 0 and pose.y > 0:
            self.current_pose = pose

    def s_callback(self, s):
        self.current_s = s.data 

    def q_callback(self, q):
        self.current_q = q.data

    def generate_map(self):
        self.gp_path = GlobalPath(self.gp_name)

    def fail_check(self):
        self.fail = self.mission_fail.check(E_stop=self.E_stop, fail_pose=self.current_pose)


    # 아래의 함수가 while문 내에서 계속 실행됨
    def mission_activate(self):
        s = self.current_s
        # if 26.9 < s < 53 or 330 < s < 360: # 배달 구역
        if s < 90:
            if not self.delivery_fail:
                self.mission_fail.simple_fail_check(self.E_stop, self.auto)
                self.delivery_fail = self.mission_fail.check_fail()
                self.mission_delivery.activate(self.current_pose, self.current_s)
            # if 26.9 < s < 53:
            #     self.target_speed_pub.publish(10)
            # else:
            #     self.target_speed_pub.publish(5)

        # elif 144 < s < 162: # 정적 장애물 구역
        #     self.target_speed_pub.publish(5)
            
        # # elif 221 < s < 254 or 518 < s < 570 or 598 < s < 633 or 720 < s < 750: # 좌회전
        # #     self.target_speed_pub.publish(10)

        # elif 188 < s < 275 or 604.5 < s < 680: # 고속 주행
        #     self.target_speed_pub.publish(10)

        # elif 520 < s < 525: # 우회전
        #     self.mission_right_stop.stop1()
        #     self.target_speed_pub.publish(5)
     

        # elif 278 < s < 306: # 대형 정적 장애물
        #     self.target_speed_pub.publish(5)
            
        # elif 76.6 < s < 115 or 552 < s < 574:
        #     self.mission_traffic.run(self.current_s)
        #     self.target_speed_pub.publish(10)

        elif 96 < s:
            if not self.parking_fail:
                self.mission_fail.simple_fail_check(self.E_stop, self.auto)
                self.parking_fail = self.mission_fail.check_fail()
                self.mission_parking.run(self.current_pose)
            # self.target_speed_pub.publish(5)


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