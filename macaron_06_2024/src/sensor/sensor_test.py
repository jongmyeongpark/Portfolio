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

from macaron_5.msg import erp_read

from location import gps_imu_fusion
from lidar_module import lidar_module


class gps_test:

    def __init__(self):
        self.heading = 0
        self.sub_erp_gear = 1
        self.imu_heading = 0


        self.gps_imu_fusion = gps_imu_fusion()

        self.lidar_module = lidar_module()

        self.vector_heading = 0

        self.pos = Point()
        self.tm_lidar = PointCloud()
        self.obs = PointCloud()

        self.x = 0
        self.y = 0

        self.obs_xyz = [0, 0, 0]
        self.lidar_timestamp = None

        self.flag = False
        self.first_flag = False

        self.gps_flag = False
        self.imu_flag = False

        self.imu_orientation = None
        self.heading_pvt = None

        self.gps_fusion_heading = 0

        #lidar 파라미터
        self.bottom=-5 #음수, 라이다 위치부터 아래 몇미터까지인지
        self.above=13 #위 몇미터까지인지
        self.front=20 #몇미터 앞까지 볼건지
        self.width=10 #라이다로 볼 데이터의 폭 (2x라면 왼쪽으로x만큼 오른쪽으로 x만큼)
        self.behind=0 #양수, 라이다기준 몇미터 뒤까지 볼건지
        self.min_intensity=0 #세기
        self.roi=[self.bottom,self.above,self.front,self.width,self.min_intensity,self.behind]


        #dbscan 설정
        self.epsilon=0.2 #입실론 값 0.4
        self.min_points=4 #한 군집을 만들 최소 포인트 개수 4
        self.z_com_flag=True #z값 압축을 한다면 True를 사용해서 풀어줘야 함
        self.lidar_flag = False
        #voxel 설정
        self.delta=0.01 #데이터가 delta의 배수로 나타나짐

        #ransac 설정 https://gnaseel.tistory.com/33
        self.p_dist=0.1 #추출된 모델로부터 거리가 n이하이면 inlier로 취급함
        self.reps=100 #ransac 반복횟수
        self.p_angle=3.14/8 #라디안, 추출된 모델과 xy평면의 최대 각도 차이


        rospy.Subscriber('erp_read', erp_read, self.erp_callback, queue_size=1)
        rospy.Subscriber('/ublox_gps/navpvt',NavPVT, self.heading_callback, queue_size=1)
        rospy.Subscriber('/imu',Imu, self.imu_callback, queue_size=1)
        rospy.Subscriber('/current_pose', Point, self.pose_callback, queue_size=1)

        self.tm_lidar_pub=rospy.Publisher('/object1', PointCloud, queue_size=100)
        self.lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback, queue_size = 1)


    def pub_tf_lidar(self):
        #print("tmttmtmtm", self.tm_lidar)

        self.tm_lidar_pub.publish(self.tm_lidar)

    def update_heading(self):
        if self.flag is False:
            return
        if self.first_flag is False:
            self.x = self.pos.x
            self.y = self.pos.y
            self.first_flag = True

        dis = np.hypot(self.x-self.pos.x, self.y- self.pos.y)
        #print("dis: ",dis)
        if(dis > 0.05):
            self.vector_heading = atan2(self.y-self.pos.y, self.x-self.pos.x)+pi
            self.x = self.pos.x
            self.y = self.pos.y

            if(self.vector_heading > 2*pi):
                self.vector_heading -= 2*pi
            elif(self.vector_heading <= 0):
                self.vector_heading += 2*pi


            if(self.sub_erp_gear == 2) :
                self.vector_heading += pi
                if self.vector_heading > 2 * pi:
                    self.vector_heading -= (2 * pi)
                elif self.vector_heading <= 0:
                    self.vector_heading += (2 * pi)

    def lidar_callback(self, lidar):
        self.obs_xyz = list(map(lambda x: list(x), pc2.read_points(lidar , field_names=("x", "y", "z"), skip_nans=True)))
        self.lidar_timestamp = lidar.header.stamp
        self.lidar_flag = True

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

        # voxel 화 & roi 설정 #
        obs_xyz = self.lidar_module.new_voxel_roi(obs_xyz,roi=self.roi)
        obs_xyz = list(set(obs_xyz))

        # ransac 돌리기
        obs_xyz = self.lidar_module.ransac(obs_xyz,reps=self.reps,p_dist=self.p_dist,p_angle=self.p_angle)

        # z값 압축
        obs_xyz = self.lidar_module.z_compressor(obs_xyz)

        # DBSCAN 돌리기
        obs_xyz, labels = self.lidar_module.dbscan(obs_xyz,epsilon=self.epsilon,min_points=self.min_points)

        # 가공한 데이터 msg 에 담기
        channel.values = labels
        for i in obs_xyz:
            point = Point32()
            point.x = i[0]
            point.y = i[1]
            point.z = i[2]
            self.obs.points.append(point)
        self.obs.channels.append(channel)
        self.obs.header.frame_id='map'

        # tm좌표계로 변환 후 다른 msg 에 담기
        tm_lidar = self.lidar_module.tf2tm(obs_xyz, self.pos.x, self.pos.y, self.gps_fusion_heading)
        self.tm_lidar = PointCloud()
        self.tm_lidar.header.frame_id = 'macaron'
        for i in tm_lidar:
            point = Point32()
            point.x = i[0]-self.pos.x
            point.y = i[1]-self.pos.y
            point.z = 0

            self.tm_lidar.points.append(point)



    def imu_callback(self, msg:Imu):
        self.imu_heading = self.q_to_rpy(msg.orientation)

        self.imu_orientation = msg.orientation

        self.imu_flag = True

    def q_to_rpy(self, imu_q):

        orientation_list = [imu_q.x, imu_q.y, imu_q.z, imu_q.w]
        _,_,yaw = euler_from_quaternion(orientation_list)
        if yaw < 0:
            yaw = pi + (pi + yaw)

        # print(yaw)

        return yaw



    def pose_callback(self, msg:Point):
        self.pos = msg
        self.flag = True

    def heading_callback(self, msg:NavPVT):

        self.heading_pvt = float(msg.heading)

        self.heading = self.tf_heading_to_rad(msg.heading)

        self.gps_flag = True

    def erp_callback(self, msg:erp_read):
        self.sub_erp_gear = msg.read_gear

    def tf_heading_to_rad(self, head):
        heading_ = pi/2 - np.deg2rad(float(head / 100000))
        if heading_ > 2*pi:
            heading_ -= 2*pi
        if heading_ < 0:
            heading_ += 2*pi
        #####################0401 수정######################################
        #후진상태일때 gps heading값 180도 돌림#
        if(self.sub_erp_gear == 2) :
            heading_ += pi
            if heading_ > 2 * pi:
                heading_ -= (2 * pi)
            elif heading_ <= 0:
                heading_ += (2 * pi)
        ####################################################################
        return heading_



rospy.init_node('test_node')

rate = rospy.Rate(10)

gps=gps_test()

while(not rospy.is_shutdown()):
    #print("######################")
    #print("gps: ",gps.heading)
    #print("gps2:",gps.vector_heading)
    #print("imu: ",gps.imu_heading)
    #print("diff", np.rad2deg(gps.heading-gps.vector_heading))


    if gps.lidar_flag is True and gps.imu_flag is True and gps.gps_flag is True:
        gps.object_update()
        gps.gps_fusion_heading = gps.gps_imu_fusion.get_heading(gps.pos.x, gps.pos.y, gps.imu_orientation, gps.heading_pvt, 1)
        #print("heading: ", gps.gps_fusion_heading)

    gps.update_heading()
    gps.pub_tf_lidar()


    rate.sleep()
