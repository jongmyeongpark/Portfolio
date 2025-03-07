#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
import time
import numpy as np

from math import *
from std_msgs.msg import Float64, Bool, String
from sensor_msgs.msg import LaserScan, NavSatFix, PointCloud, Imu, PointCloud2, ChannelFloat32
from geometry_msgs.msg import Point, Point32, Quaternion
import sensor_msgs.point_cloud2 as pc2
from ublox_msgs.msg import NavPVT
from tf.transformations import euler_from_quaternion

from macaron_06.msg import erp_read

from location import gps_imu_fusion
from lidar_module import lidar_module


class SensorDataHub:
    def __init__(self):
        # 구독자 선언
        self.sub_gps = rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.pose_callback, queue_size=1)
        self.sub_heading = rospy.Subscriber('/ublox_gps/navpvt', NavPVT, self.heading_callback, queue_size=1)
        self.sub_imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback, queue_size=1)
        self.lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback, queue_size=1)
        self.parameter_sub = rospy.Subscriber('/lidar_mode', String, self.mode_callback, queue_size=1)
        self.erp_sub = rospy.Subscriber('erp_read', erp_read, self.erp_callback, queue_size=1)
        self.local_check = rospy.Subscriber('localization_mode', Bool, self.local_check_callback, queue_size=1)

        # 발행자 선언
        self.localization_pub = rospy.Publisher('current_pose', Point, queue_size=1)  # x,y는 tm좌표, z에 들어가 있는 값이 heading
        self.tm_lidar_pub = rospy.Publisher('object', PointCloud, queue_size=100)  # tm좌표계로 변환 & 2D좌표
        self.obs_pub = rospy.Publisher('object3D', PointCloud, queue_size=100)  # lidar기준 좌표계 & 3D좌표

        # 사용하는 객체 선언
        self.loc_fusion = gps_imu_fusion()

        # flag 선언
        self.lidar_flag = False
        self.gps_flag = False
        self.imu_flag = False

        self.localization_flag = False

        self.previous_time = time.time()

        # Sub 받은 데이터 저장 공간
        self.sub_coord = [0.0, 0.0]
        self.sub_gps_heading = 0.0
        self.obs_xyz = [0, 0, 0]
        self.sub_imu = Quaternion()
        self.linear_velocity = 0.0
        self.steer_angle = 0.0
        self.count = 0
        self.lidar_timestamp = None

        # lidar 파라미터
        self.bottom = -0.3  # 음수, 라이다 위치부터 아래 몇 미터 까지 인지
        self.above = 13  # 위 몇 미터 까지 인지
        self.front = 20  # 몇미터 앞까지 볼건지
        self.width = 10  # 라이다로 볼 데이터의 폭 (2x라면 왼쪽으로 x만큼 오른쪽으로 x만큼)
        self.behind = 0  # 양수, 라이다 기준 몇미터 뒤까지 볼건지
        self.min_intensity = 0  # 세기
        self.roi = [self.bottom, self.above, self.front, self.width, self.min_intensity, self.behind]

        # dbscan 설정
        self.epsilon = 0.2  # 입실론 값 0.4
        self.min_points = 4  # 한 군집을 만들 최소 포인트 개수 4
        self.z_com_flag = True  # z값 압축을 한다면 True를 사용해서 풀어줘야 함

        # voxel 설정
        self.delta = 0.01  # 데이터가 delta의 배수로 나타나짐

        # ransac 설정 https://gnaseel.tistory.com/33
        self.p_dist = 0.1  # 추출된 모델로부터 거리가 n이하이면 inlier로 취급함
        self.reps = 100  # ransac 반복횟수
        self.p_angle = 3.14 / 8  # 라디안, 추출된 모델과 xy평면의 최대 각도 차이

        # obs_pub에 사용되는 msg 객체 선언
        self.pos = Point()
        self.obs = PointCloud()
        self.len_obs = PointCloud()
        self.tm_lidar = PointCloud()

        self.local_pos = Point()
        self.wheelbase = 1.04  # [m]
        self.yaw = 0

        self.ss_flag = True
        self.pre_imu = 0

        self.lidar_module = lidar_module()

    # #########callback 함수 모음##########
    # 각 센서에서 데이터가 들어오면 객체 내부의 데이터 저장공간에 저장
    def pose_callback(self, fix):
        self.sub_coord = [fix.longitude, fix.latitude]
        self.gps_flag = True

    def heading_callback(self, head):
        self.sub_gps_heading = float(head.heading)

    def lidar_callback(self, lidar):
        self.obs_xyz = list(map(lambda x: list(x), pc2.read_points(lidar, field_names=("x", "y", "z"), skip_nans=True)))
        self.lidar_timestamp = lidar.header.stamp
        self.lidar_flag = True

    def mode_callback(self, string):
        mode = string.data
        if mode == 'cruising':
            self.bottom = -0.3
            self.above = 13
            self.front = 20
            self.width = 0
            self.behind = 0
            self.min_intensity = 0
            self.epsilon = 0.2
            self.min_points = 4
            self.z_com_flag = True
            self.delta = 0.01
            self.p_dist = 0.1
            self.reps = 100
            self.p_angle = 3.14 / 8

        elif mode == 'tunnel':
            self.bottom = -0.5
            self.above = 13
            self.front = 20
            self.width = 5
            self.behind = 0
            self.min_intensity = 0
            self.epsilon = 0.2
            self.min_points = 4
            self.z_com_flag = True
            self.delta = 0.01
            self.p_dist = 0.1
            self.reps = 100
            self.p_angle = 3.14 / 8

        elif mode == 'delivery':
            self.bottom = -0.3
            self.above = 13
            self.front = 20
            self.width = 10
            self.behind = 0
            self.min_intensity = 0
            self.epsilon = 0.2
            self.min_points = 4
            self.z_com_flag = True
            self.delta = 0.01
            self.p_dist = 0.1
            self.reps = 100
            self.p_angle = 3.14 / 8

        elif mode == 'UT':
            self.bottom = -0.6
            self.above = 13
            self.front = 20
            self.width = 6
            self.behind = 0
            self.min_intensity = 0
            self.epsilon = 0.8
            self.min_points = 3
            self.z_com_flag = True
            self.delta = 0.01
            self.p_dist = 0.1
            self.reps = 100
            self.p_angle = 3.14 / 8

        elif mode == 'parking_parallel':
            self.bottom = -0.6
            self.above = 13
            self.front = 20
            self.width = 9
            self.behind = 0
            self.min_intensity = 0
            self.epsilon = 0.2
            self.min_points = 4
            self.z_com_flag = True
            self.delta = 0.01
            self.p_dist = 0.1
            self.reps = 100
            self.p_angle = 3.14 / 8

        elif mode == 'static_obstacle_mini':
            self.bottom = -0.3
            self.above = 13
            self.front = 20
            self.width = 4
            self.behind = 0
            self.min_intensity = 0
            self.epsilon = 0.2
            self.min_points = 4
            self.z_com_flag = True
            self.delta = 0.01
            self.p_dist = 0.1
            self.reps = 100
            self.p_angle = 3.14 / 8

        elif mode == 'static_obstacle':
            self.bottom = -0.5
            self.above = 13
            self.front = 20
            self.width = 6
            self.behind = 0
            self.min_intensity = 0
            self.epsilon = 0.2
            self.min_points = 4
            self.z_com_flag = True
            self.delta = 0.01
            self.p_dist = 0.1
            self.reps = 100
            self.p_angle = 3.14 / 8

        elif mode == 'track':
            self.bottom = -0.3
            self.above = 13
            self.front = 20
            self.width = 6
            self.behind = 0
            self.min_intensity = 0
            self.epsilon = 0.2
            self.min_points = 4
            self.z_com_flag = True
            self.delta = 0.01
            self.p_dist = 0.1
            self.reps = 100
            self.p_angle = 3.14 / 8

        elif mode == 'end':
            self.bottom = -5
            self.above = 13
            self.front = 20
            self.width = 10
            self.behind = 0
            self.min_intensity = 0
            self.epsilon = 0.2
            self.min_points = 4
            self.z_com_flag = True
            self.delta = 0.01
            self.p_dist = 0.1
            self.reps = 100
            self.p_angle = 3.14 / 8

        self.roi = [self.bottom, self.above, self.front, self.width, self.min_intensity, self.behind]

    def imu_callback(self, imu):
        self.sub_imu = imu.orientation
        orientation_list = [self.sub_imu.x, self.sub_imu.y, self.sub_imu.z, self.sub_imu.w]
        _, _, self.yaw = euler_from_quaternion(orientation_list)
        # self.yaw = self.q_to_yaw(self.yaw)
        # print("yaw!!!!!!!!!!!! : ", 180*self.yaw/pi)
        self.imu_flag = True

    # noinspection PyMethodMayBeStatic
    def q_to_yaw(self, imu):
        # q means Quaternion
        # orientation_list = [imu.x, imu.y, imu.z, imu.w]
        # (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        # print(yaw)
        yaw = (-1) * imu.x * pi / 180

        if yaw < 0:
            yaw = pi + (pi + yaw)

        return yaw

    ######################################
    # gps, imu, lidar 가 모두 작동해서 데이터가 들어왔는지 확인
    def sensor_check(self):
        return self.gps_flag and self.imu_flag and self.lidar_flag

    ######################################
    # erp42의 velocity, steer값을 가져옴

    def erp_callback(self, erp):
        self.linear_velocity = erp.read_speed * (1000 / 3600 / 10)
        self.steer_angle = - erp.read_steer / 71

    ########################################

    def local_check_callback(self, msg):
        self.localization_flag = msg.data

    def localization_update(self, select_heading):
        current_time = time.time()
        if self.localization_flag is False:
            x, y = self.loc_fusion.tf_to_tm(self.sub_coord[0], self.sub_coord[1])
            heading = self.loc_fusion.get_heading(x, y, self.sub_imu, self.sub_gps_heading,
                                                  select_heading)  # 지금은 그냥 gps 헤딩을 그대로 쓰지만, imu가 추가된 해딩을 처리하는 클래스가 필요.
            self.pos.x = x
            self.pos.y = y
            self.pos.z = heading
            self.local_pos.x = self.pos.x
            self.local_pos.y = self.pos.y
            self.local_pos.z = self.pos.z
        else:
            self.pos.x = -1.0
            self.pos.y = 0
            self.pos.z = 0

    def object_update(self):
        # 객체 선언
        self.obs = PointCloud()
        channel = ChannelFloat32()

        # callback 받은 데이터 불러오기
        obs_xyz = self.obs_xyz
        lidar_timestamp = self.lidar_timestamp

        # msg 기본 설정
        self.obs.header.stamp = lidar_timestamp
        channel.name = 'cluster'

        # voxel 화 & roi 설정
        obs_xyz = self.lidar_module.new_voxel_roi(obs_xyz, roi=self.roi)
        obs_xyz = list(map(lambda x: list(x), set(obs_xyz)))

        # ransac 돌리기
        # obs_xyz = self.lidar_module.ransac(obs_xyz, reps=self.reps, p_dist=self.p_dist, p_angle=self.p_angle)

        # z값 압축
        obs_xyz = self.lidar_module.z_compressor(obs_xyz)

        # DBSCAN 돌리기
        obs_xyz, labels = self.lidar_module.dbscan(obs_xyz, epsilon=self.epsilon, min_points=self.min_points)

        # 가공한 데이터 msg 에 담기
        channel.values = labels
        for i in obs_xyz:
            point = Point32()
            point.x = i[0]
            point.y = i[1]
            point.z = i[2]
            self.obs.points.append(point)
        self.obs.channels.append(channel)
        self.obs.header.frame_id = 'map'

        # tm좌표계로 변환 후 다른 msg 에 담기
        tm_lidar = self.lidar_module.tf2tm(obs_xyz, self.pos.x, self.pos.y, self.pos.z)
        self.tm_lidar = PointCloud()
        self.tm_lidar.header.frame_id = 'macaron'
        for i in tm_lidar:
            point = Point32()
            point.x = i[0] - self.pos.x
            point.y = i[1] - self.pos.y
            point.z = 0
            '''
            point.x = i[0] # - self.pos.x  사실은 빼기 하는게 틀린거임 지금 글로벌 좌표계로 라이다 관측값(좌표)를 보내려는건데 tm_lidar는 이미 글로벌 좌표계임 
            point.y = i[1] # - self.pos.y    
            '''

            self.tm_lidar.points.append(point)
            # print("self_tm_lidar:",self.tm_lidar)
        # self.tm_lidar.channels.append(channel)

    ######################################
    # def rubber_len_check(self):
    #     self.object_update()
    #     self.rub_len = 

    #     return to_len
    ########## publish 함수 모음############

    def pub_pose(self):
        self.localization_pub.publish(self.pos)

    def pub_obs(self):
        self.obs_pub.publish(self.obs)

    def pub_tf_lidar(self):
        # print("tmttmtmtm", self.tm_lidar)

        self.tm_lidar_pub.publish(self.tm_lidar)
    ######################################


def main():
    # 기본 설정
    rospy.init_node('data_hub_new', anonymous=True)
    # rate=rospy.Rate(1) #지금 코드에서는 안써도 됨

    Data = SensorDataHub()  # 객체 생성
    start_rate = time.time()  # 시간 스템프

    # 센서가 모두 동작하기 전까지는 대기
    # while not Data.senser_check():
    #     print("senser not ready")
    #     rate.sleep()
    #     continue

    while not rospy.is_shutdown():
        if time.time() - start_rate > 0.01:  # 0.2초 간격으로 실행. 데이터 처리가 0.3초보다 빨라도 여유롭게 0.3초간격으로 실행하고, 더 늦으면 업데이트 되는대로 실행.
            # print("rate : "),
            # print(int((time.time() - start_rate)*1000)),
            # print("(ms), "),

            start_rate = time.time()  # 시작 시간 스템프 찍고
            # 각 정보 업데이트 후 발행
            Data.localization_update(2)

            # 라이다 센서에 값이 들어오면 발행
            if Data.lidar_flag is True:
                Data.object_update()
                Data.pub_obs()
                Data.pub_tf_lidar()
                Data.tm_lidar = PointCloud()

            # Data.object_update_back()
            Data.pub_pose()

            # Data.pub_obs_back()
            # print(Data.pos)

            print("Processing time : ")
            print(int(1 / (time.time() - start_rate)))
            print("(fps)")


if __name__ == '__main__':
    main()
