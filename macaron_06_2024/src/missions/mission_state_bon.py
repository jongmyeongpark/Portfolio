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
from geometry_msgs.msg import Point, Point32
from sensor_msgs.msg import PointCloud, CompressedImage
from macaron_06.msg import erp_read
from path_planning.global_path import GlobalPath

# 미션 파일 임포트
from mission_fail import MissionFail
from sensor.final.delivery_judge import DeliveryJudge
from sensor.final.mission_traffic_kcity import mission_traffic_kcity
from mission_parking import mission_parking
from mission_right_stop import MissionRightStop

# bagfile 테스트 시 사용
# GB_PATH에서 사용자 이름(takrop)를 자신의 경로로 설정해서 실행하기
BAGFILE = False
GB_PATH = "/home/macaron/catkin_ws/src/macaron_06/path/npy_file/path/"
GB = "manhae_06.28_c.npy"

parking_heading= [4.316041497795199,4.322160729993996 ,4.336455656438726]
goal_point= [[935536.1489751234,1915871.687415901],[935533.7110172126,1915867.2349804495],[935531.281245241,1915862.7159135127]]

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
        self.parking_delivery_pub = rospy.Publisher('/parking_delivery', PointCloud, queue_size=1)
        self.target_speed_pub = rospy.Publisher('/mission_target_speed', Float32, queue_size=1)

        self.map_client = rospy.ServiceProxy('MapLoad', MapLoad)
        
        # 미션 모음   
        self.mission_fail = MissionFail()
        self.mission_delivery = DeliveryJudge()
        self.mission_traffic = mission_traffic_kcity()
        self.mission_parking = mission_parking(PARKING_LEN,PARKING_WIDTH,goal_point,parking_heading)
        self.mission_right_stop = MissionRightStop()

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

    def fail_parking_delivery(self):
        fail_path = PointCloud()
        for point in [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]:
            p = Point32()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            fail_path.points.append(p)
        self.parking_delivery_pub.publish(fail_path)


    # 아래의 함수가 while문 내에서 계속 실행됨
    def mission_activate(self):
        s = self.current_s
        
        # 속도 세팅 
        if 0 < s < 68.9 or 663 <= s < 731.3 or 932.3 <= s < 1047 or s >= 1100:
            self.target_speed_pub.publish(20)
        elif 68.9 <= s < 118.4 or 137.5 <= s < 170 or 341 <= s < 520 or 1047 < s < 1100:
            self.target_speed_pub.publish(5)
        elif 118.4 <= s < 137.5 or 170 <= s < 283.8 or 731.3 <= s < 834.7:
            self.target_speed_pub.publish(8)
        elif 283.8 <= s < 341 or 520 <= s < 533 or 834.7 <= s < 932.3:
            self.target_speed_pub.publish(15)
        elif 533 <= s < 663:
            self.target_speed_pub.publish(10)

        # =======================================================================================================

        if 73.7 < s < 95.4 or 432 < s < 477.9: # 배달 구역
            self.mission_fail.check(self.E_stop, self.auto)
            self.delivery_fail = self.mission_fail.check_fail()
            try:
                if not self.delivery_fail: self.mission_delivery.activate(self.current_pose, self.current_s)
                elif self.delivery_fail: self.fail_parking_delivery()
            except:
                pass

        elif 184 < s < 195 or 199 < s < 205 or 774 < s < 785: # 우회전
            try:
                if 186.9 <= s <= 195 : self.mission_right_stop.stop1()
                elif 202.6 <= s <= 205: self.mission_right_stop.stop2()
                elif 777.2 <= s <= 785: self.mission_right_stop.stop3()
            except:
                pass
        
        elif 210.1 < s < 235.1 or 263.1 < s < 288.1 or 400.1 < s < 425.1 or 507.1 < s < 532.1 or 585.9 < s < 610.9 or 883.2 < s < 908.2 or 929.3 < s < 954.3:
            try:
                self.mission_traffic.run(self.current_s)
            except:
                pass

        elif 1035 < s < 1150:
            self.mission_fail.check(self.E_stop, self.auto)
            self.parking_fail = self.mission_fail.check_fail()
            try:
                if not self.parking_fail: self.mission_parking.run(self.current_pose)
                elif self.delivery_fail: self.fail_parking_delivery()
            except:
                pass

        # if 115 < s < 133: # 정적 장애물 구역
        #     pass
        # if 221 < s < 254 or 518 < s < 570 or 598 < s < 633 or 720 < s < 750: # 좌회전
        #     pass
        # if 357 < s < 413: # 대형 정적 장애물
        #     pass


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