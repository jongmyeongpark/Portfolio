#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
import time
import numpy as np

from math import *
from std_msgs.msg import Float64, Bool, String, Header
from sensor_msgs.msg import LaserScan, NavSatFix, PointCloud, Imu, PointCloud2, PointField
from geometry_msgs.msg import Point, Point32, Quaternion
import sensor_msgs.point_cloud2 as pc2
from ublox_msgs.msg import NavPVT
from tf.transformations import euler_from_quaternion

from macaron_6.msg import erp_read

from location import gps_imu_fusion
from lidar_module import lidar_module


class SensorDataHub:
    def __init__(self):
        # 구독자 선언
        self.sub_gps = rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.pose_callback, queue_size=1)
        self.sub_heading = rospy.Subscriber('/ublox_gps/navpvt', NavPVT, self.heading_callback, queue_size=1)
        self.sub_imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback, queue_size=1)
        self.lidar_sub = rospy.Subscriber('/lidar_data', PointCloud2, self.lidar_callback, queue_size=1)
        self.erp_sub = rospy.Subscriber('erp_read', erp_read, self.erp_callback, queue_size=1)
        self.local_check = rospy.Subscriber('localization_mode', Bool, self.local_check_callback, queue_size=1)

        # 발행자 선언
        self.localization_pub = rospy.Publisher('current_pose', Point, queue_size=1)  # x,y는 tm좌표, z에 들어가 있는 값이 heading
        self.tm_lidar_pub = rospy.Publisher('object', PointCloud2, queue_size=100)  # tm좌표계로 변환 & 2D좌표
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
        self.obs_xyz = list(map(lambda x: list(x), pc2.read_points(lidar, field_names=("x", "y", "z","rgb"), skip_nans=True)))
        self.lidar_timestamp = lidar.header.stamp
        self.lidar_flag = True


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

        # callback 받은 데이터 불러오기
        obs_xyz = self.obs_xyz
        
        for i in self.obs_xyz:
            point = Point32()
            point.x = i[0]
            point.y = i[1]
            point.z = i[2] 
            
            self.obs.points.append(point)

        # tm좌표계로 변환 후 다른 msg 에 담기
        tm_lidar = self.lidar_module.tf2tm(obs_xyz, self.pos.x, self.pos.y, self.pos.z)

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'macaron'

        if tm_lidar.size > 0:    # 존재해야 뺄셈 가능
            tm_lidar[:, 0] = tm_lidar[:, 0] - self.pos.x
            tm_lidar[:, 1] = tm_lidar[:, 1] - self.pos.y


        # for i in tm_lidar:
        #     point = Point32()
        #     point.x = i[0] - self.pos.x
        #     point.y = i[1] - self.pos.y
        #     point.z = 0

        #     self.tm_lidar.points.append(point)
        # print("self_tm_lidar:",self.tm_lidar)

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('rgb', 12, PointField.FLOAT32, 1)]
        
        self.tm_lidar = pc2.create_cloud(header, fields, tm_lidar)



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
    rospy.init_node('data_hub_cpp', anonymous=True)
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
