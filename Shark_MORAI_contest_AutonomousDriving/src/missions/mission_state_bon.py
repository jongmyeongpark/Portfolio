#!/usr/bin/env python3
#-*-coding:utf-8-*-

# 라이브러리 임포트
import rospy
import time
import os, sys
import pickle
import numpy as np
import cv2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
from math import cos, sin

# 파일 경로 추가
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/sensor")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/path_planning")

# 메세지 임포트
from morai_2.srv import MapLoad, MapLoadRequest
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Point, Point32
from sensor_msgs.msg import PointCloud ,PointCloud2, Imu, CompressedImage
from nav_msgs.msg import Odometry
from morai_msgs.msg import CtrlCmd
from path_planning.global_path import GlobalPath
from path_planning.cubic_spline_planner import calc_spline_course


# 라이다 파일 임포트
# from sensor.only_lidar_detect_dynamic import ProcessedPointCloud
from sensor.lidar_dynamic_2 import ProcessedPointCloud
from sensor.lidar_dynamic_2_sub import ProcessedPointCloudSub
from sensor.lidar_preprocess import Detection
from missions.parking_area_detect import ProcessedPointCloud_


# 미션 파일 임포트
from mission_slam import MissionSLAM 
from mission_dynamic import MissionDynamicObs
from mission_static import MissionStaticObs
from traffic_light import shark_traffic_light_detect
from human_detection import YoloPersonDetect
from mission_reckoning import Position
from mission_parking import MissionParking

# bagfile 테스트 시 사용
# GB_PATH에서 사용자 이름(takrop)를 자신의 경로로 설정해서 실행하기
BAGFILE = False
GB_PATH = "/home/takrop/catkin_ws/src/morai_2/path/npy_file/"
GB = "manhae_06.28_c.npy"

class MissionState():
    def __init__(self):
        
        self.gp_name = ""
        self.gp_path = None
        self.final_stop_pub = rospy.Publisher('/final_stop', Bool, queue_size=1)
        self.target_speed_pub = rospy.Publisher('/target_speed', Float32, queue_size=1)

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback, queue_size=2)
        self.velodyne_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.velodyne_callback, queue_size=1)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)

        self.selected_path_sub = rospy.Subscriber('/SLpath', PointCloud, self.path_callback, queue_size=2)
        self.pose_sub = rospy.Subscriber('/current_pose', Point, self.pose_callback, queue_size=1)
        self.est_pose_sub = rospy.Subscriber('/estimate_pose', Point, self.estimate_pose_callback, queue_size=1)
        self.current_s_sub = rospy.Subscriber('/current_s', Float32, self.s_callback, queue_size=1)
        self.current_q_sub = rospy.Subscriber('/current_q', Float32, self.q_callback, queue_size=1)
        self.speed_sub = rospy.Subscriber('/speed', Float32, self.speed_callback, queue_size=1)
        self.ctrl_sub = rospy.Subscriber("/ctrl_cmd", CtrlCmd, self.ctrl_callback ,queue_size=1)

        self.lidar_odom_sub = rospy.Subscriber('/lidar_odom', Odometry, self.lidar_odom_callback, queue_size=1)
        self.map_client = rospy.ServiceProxy('MapLoad', MapLoad)

        # 정적, 동적 판단 라이다
        self.dynamic_lidar = ProcessedPointCloud()
        self.dynamic_lidar_sub = ProcessedPointCloudSub()
        self.static_lidar = Detection()
        self.parking_lidar = ProcessedPointCloud_()

        # 미션 모음   
        self.mission_slam_1 = MissionSLAM()
        self.mission_slam_2 = MissionSLAM()
        self.mission_dynamic_1 = MissionDynamicObs()
        self.mission_dynamic_2 = MissionDynamicObs()
        self.mission_static = MissionStaticObs()
        self.mission_parking = MissionParking()
        self.mission_light = shark_traffic_light_detect()
        self.yolo_person_detect = YoloPersonDetect()

        # 멤버 변수 ------------------------------------

        # 이미지
        self.bridge = CvBridge()
        self.img_flag = False  

        # 음영구역 및 주행
        self.current_pose = Point()
        self.start_pose = Point()
        self.est_pose = Point()
        self.pose_init = False

        self.current_s = 0.0
        self.current_q = 0.0
        self.max_index = 0

        self.lidar_odom = Odometry()
        self.ctrl_steer, self.ctrl_speed = 0, 0

        self.slpath = [[0,1], [0,1], [0,1]]

        # 라이다
        self.velodyne_pointcloud = np.array([0,0])
        self.velodyne_pointcloud2 = list()

        # IMU
        self.imu_data = None
        self.imu_flag = False

        # 미션 상태
        self.mission_state = 0

        # 플래그
    
    def pose_callback(self, pose):
        if pose.x < 0:
            self.current_pose = self.est_pose
            return

        if not self.pose_init:
            self.start_pose = pose
            self.pose_init = True

        self.current_pose = pose

    def estimate_pose_callback(self, pose):
        self.est_pose = pose

    def s_callback(self, s):
        self.current_s = s.data 
        try:
            self.current_index = self.gp_path.getClosestSIndexCurS(self.current_s)
        except: pass
 
    def q_callback(self, q):
        self.current_q = q.data

    def speed_callback(self, speed):
        self.current_speed = speed.data

    def path_callback(self, path: PointCloud):
        path_x = []
        path_y = []
        for i in range(0, len(path.points)):
            path_x.append(path.points[i].x)
            path_y.append(path.points[i].y)

        if len(path_x) >= 2:
            path_x, path_y, path_yaw, _, _, _ = calc_spline_course(path_x, path_y, ds=0.1)

        else:
            path_x = [self.current_pose.x, 1+self.current_pose.x]
            path_y = [self.current_pose.y, 1+self.current_pose.y]
            path_x, path_y, path_yaw, _, _, _ = calc_spline_course(path_x, path_y, ds=0.1)

        self.slpath = [path_x, path_y, path_yaw]

    def image_callback(self, msg):
        try:
            self.img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            self.img_flag = True
        except: 
            self.img_flag = False  
        
    def velodyne_callback(self, velodyne_data):
        self.velodyne_pointcloud2 = list(map(lambda x: list(x), pc2.read_points(velodyne_data, field_names=("x", "y", "z", "intensity"), skip_nans=True)))
        self.velodyne_pointcloud  = np.array(list(pc2.read_points(velodyne_data, field_names=("x", "y", "z"), skip_nans=True)))
        
    def imu_callback(self, imu_data):
        self.imu_data = imu_data

    def lidar_odom_callback(self, odom):
        self.lidar_odom = odom

    def ctrl_callback(self, ctrl: CtrlCmd):
        self.ctrl_steer, self.ctrl_speed = ctrl.velocity, ctrl.steering 
        
    def map_loader(self):
        response = self.map_client("")
        if response != "":
            self.gp_name = response.response
            print(self.gp_name)

    def generate_map(self):
        self.gp_path = GlobalPath(self.gp_name)
        self.max_index = len(self.gp_path.rx)

    def mission_activate(self):
        s = self.current_s
        if self.current_pose.x < 0: self.current_pose = self.est_pose

        if 400 <= s < 425:
            self.target_speed_pub.publish(4)
        else:
            self.target_speed_pub.publish(0)

        collision_mode_off = False
        if 27 <= s < 152.1:
            dynamic_list = self.dynamic_lidar.activate(self.velodyne_pointcloud2, self.current_pose)
            self.mission_dynamic_1.dynamic_follow(self.current_pose, dynamic_list)
        elif 415 <= s < 498:
            dynamic_list = self.dynamic_lidar.activate(self.velodyne_pointcloud2, self.current_pose)
            self.mission_dynamic_2.dynamic_follow(self.current_pose, dynamic_list)
        elif 204 <= s < 214:
            dynamic_list = self.dynamic_lidar.activate(self.velodyne_pointcloud2, self.current_pose)
            collision_mode_off = self.mission_dynamic_1.dynamic_stop_with_static(self.current_pose, self.slpath, dynamic_list)
        elif 521 <= s < 529:
            dynamic_list = self.dynamic_lidar.activate(self.velodyne_pointcloud2, self.current_pose)
            self.mission_dynamic_1.dynamic_stop(self.current_pose, self.slpath, dynamic_list)
        elif 625 <= s < 770:
            dynamic_list = self.dynamic_lidar_sub.activate(self.velodyne_pointcloud2, self.current_pose)
            self.mission_dynamic_1.dynamic_stop(self.current_pose, self.slpath, dynamic_list)
        elif 150 <= s < 167.7 or 321 <= s < 332 or 400 <= s < 415 or 605 <= s < 625:
            result = self.yolo_person_detect.detect_human(self.img)
            self.mission_dynamic_1.image_dynamic_stop(result)

        if 168 <= s < 204:
            # print('static')
            self.static_lidar.show_clusters(self.velodyne_pointcloud)
            if not collision_mode_off:
                self.mission_static.obs_collision_check(self.current_pose, self.current_speed)
        elif 220 < s <= 314:
            self.static_lidar.show_clusters(self.velodyne_pointcloud)
        elif 524 <= s < 618:
            self.static_lidar.show_clusters(self.velodyne_pointcloud)

        if 205 <= s < 321:
            if s < 212:        
                self.mission_slam_1.init_setting(odom_init=self.lidar_odom, pose_init=self.current_pose)
            else:
                self.est_pose = self.mission_slam_1.activate(odom=self.lidar_odom, heading=self.current_pose.z, start_heading=self.start_pose.z)
        elif 509 <= s < 600:
            if s < 514:
                self.mission_slam_2.init_setting(odom_init=self.lidar_odom, pose_init=self.current_pose)
            else:
                self.est_pose = self.mission_slam_2.activate(odom=self.lidar_odom, heading=self.current_pose.z, start_heading=self.start_pose.z)


        if 326 <= s < 400 and not self.mission_parking.end_flag:
            if self.mission_parking.parking_status == 0:
                parking_point = np.array(self.parking_lidar.activate(self.velodyne_pointcloud2, self.current_pose))
                if parking_point.size != 0 and self.mission_parking.parking_status == 0:
                    self.mission_parking.detect(parking_point)
                    self.mission_parking.current_pose_save(self.current_pose)

            if self.mission_parking.parking_status == 1:
                self.mission_parking.move2start(self.current_pose)
            elif self.mission_parking.parking_status == 2:
                self.mission_parking.start2backpoint(self.current_pose)
            elif self.mission_parking.parking_status == 3:
                self.mission_parking.backpoint2square(self.current_pose)
            elif self.mission_parking.parking_status == 4:
                self.mission_parking.parking()
            elif self.mission_parking.parking_status == 5:
                self.mission_parking.square2backpoint(self.current_pose)
            elif self.mission_parking.parking_status == 6:
                self.mission_parking.back2globalpath()
            else:
                pass

        if 659 <= s < 690 and self.img_flag:
           self.mission_light.detect_traffic_light(self.img, self.current_s)

        if s > 771:
            self.final_stop_pub.publish(True)

def main():
    MS = MissionState()
    rospy.init_node('mission_state', anonymous=True)
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

    while not rospy.is_shutdown():
        MS.mission_activate()

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass