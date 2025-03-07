#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
import time
import math
import numpy as np
##sdf
from macaron_6.msg import erp_read
from std_msgs.msg import Float64,Float32
from sensor_msgs.msg import LaserScan, NavSatFix, PointCloud, Imu, MagneticField
from geometry_msgs.msg import Point, Point32, Quaternion

from ublox_msgs.msg import NavPVT
import matplotlib.pyplot as plt



from location import gps_imu_fusion
from lidar import lidar

from tf.transformations import euler_from_quaternion

from pyproj import Proj, Transformer, CRS

proj_UTMK = CRS(init='epsg:5179')
proj_WGS84 = CRS(init='epsg:4326')


class SensorDataHub:
    def __init__(self):
        #구독자 선언
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size = 1)
        self.yd_laser_sub = rospy.Subscriber('/scan2', LaserScan, self.scan_yd_callback, queue_size = 1)
        self.sub_gps = rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.pose_callback, queue_size=1)
        self.sub_heading = rospy.Subscriber('/ublox_gps/navpvt', NavPVT, self.heading_callback, queue_size=1)
        self.erp_sub = rospy.Subscriber('erp_read', erp_read, self.erp_callback, queue_size=1)
        self.sub_imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback, queue_size=1)
        self.sub_mag = rospy.Subscriber('/imu/mag', MagneticField, self.mag_callback, queue_size=1)

        #발행자 선언
        self.localization_pub=rospy.Publisher('current_pose', Point, queue_size=1) # x,y는 tm좌표, z에 들어가 있는 값이 heading
        self.obs_pub=rospy.Publisher('object', PointCloud, queue_size=1) # PointCloud.points[i].x 이런식으로 사용 #visual
        self.obs_back_pub=rospy.Publisher('object_back', PointCloud, queue_size=1)
        self.speed_pub=rospy.Publisher('/speed', Float32, queue_size=1) # speed [m/s]
        self.GPS_heading=rospy.Publisher('/GPS_heading', Float32, queue_size=1) # speed [m/s]
        #사용하는 객체 선언
        self.loc_fusion = gps_imu_fusion()
        self.Lidar = lidar()

        #flag 선언
        self.lidar_flag = False
        self.gps_flag = False
        self.imu_flag = False
        
        #Sub 받은 데이터 저장 공간
        self.sub_cood = [0.0, 0.0]
        self.sub_gps_heading = 0.0
        self.sub_scan = []
        self.sub_scan_yd = []
        for i in range(810):
            self.sub_scan.append(0.0)
        self.sub_imu = Quaternion()
        
        #obs_pub에 사용되는 msg 객체 선언
        self.pos = Point()
        self.obs = PointCloud()
        self.len_obs = PointCloud()

        self.GPS_INPUT_TIME = time.time()
        self.GPS_LAST_TIME = time.time()
        self.pos2 = [0,0]
        self.last_pos = [0,0]

        self.erp_init_Count= 0
        self.dt = 0
        self.last_enc = 0
        self.enc = 0 
        self.erp_velocity2  = 0
        self.erp_INPUT_TIME = time.time()
        self.erp_LAST_TIME = time.time()

        self.GPS_DATA_dT = 0
        self.GPS_DATA_LAST_TIME = time.time()
        self.GPS_DATA_accel = 0
        self.GPS_DATA_velocity_kmh = 0 #속도
        self.GPS_DATA_LAST_velocity_kmh = 0 #이전 속도
        self.GPS_DATA_heading_degrees = 0
        self.GPS_DATA_pos_heading_radians = 0
        
        self.GPS_VECOCITY_Heading = 0
        self.last_pos = [0,0]

        self.IMU_heading_array = []
        self.GPS_DATA_heading_array = []
        self.GPS_heading_array = []

        self.yaw = 0
        self.count_array = []
        self.count = 0
        self.yaw_edit = 0

        self.x = []
        self.y = []
        self.imu_yaw = 0
    ##########callback 함수 모음##########
    # 각 센서에서 데이터가 들어오면 객체 내부의 데이터 저장공간에 저장

    def imu_callback(self, imu):
        # orientation = (imu.orientation.x-0.00264, imu.orientation.y-0.00264, imu.orientation.z, imu.orientation.w)
        orientation = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
        # self.x.append(imu.orientation.x)
        # self.y.append(imu.orientation.y)
        _,_,yaw = euler_from_quaternion(orientation)
        # yaw = math.atan2(imu.orientation.y, imu.orientation.x)
        # yaw = -yaw
        # yaw += np.deg2rad(-8.0 + (50.0/60.0)) 

        # yaw = (2*np.pi - yaw) - 3.976
        # yaw += np.pi *0.5
        if yaw >= 2*np.pi: yaw -= 2*np.pi
        if yaw < 0: yaw += 2*np.pi
        self.imu_yaw = yaw
        # print(f'deg: {heading}')
        # print(f'rad: {yaw}')
        # print()
        
        self.imu_flag = True

        if self.GPS_DATA_pos_heading_radians != 0:
            if self.GPS_DATA_heading_degrees >= self.imu_yaw:
                self.yaw_edit = self.GPS_DATA_pos_heading_radians - self.imu_yaw
            else:
                self.yaw_edit = self.imu_yaw - self.GPS_DATA_pos_heading_radians 

        self.imu_yaw = self.imu_yaw + self.yaw_edit
        
        print(self.imu_yaw,self.GPS_DATA_pos_heading_radians,self.GPS_VECOCITY_Heading,"_______",self.yaw_edit,np.rad2deg(self.yaw_edit))

        self.count_array.append(self.count)
        self.count += 1
        self.IMU_heading_array.append(self.yaw)
        self.GPS_DATA_heading_array.append(self.GPS_DATA_pos_heading_radians)
        self.GPS_heading_array.append(self.GPS_VECOCITY_Heading)

    def mag_callback(self, mag):
        pi = np.pi
        yaw = np.arctan2(mag.magnetic_field.y, mag.magnetic_field.x) + pi * 0.5

        if yaw >= 2 * pi:
            yaw -= 2*pi

        if yaw < 0:
            yaw += 2*pi
        self.mag_yaw = yaw
    ###################################################
    def pose_callback(self, Fix):
        self.sub_cood = [Fix.longitude, Fix.latitude]
        self.gps_flag = True

        lon = Fix.longitude
        lat = Fix.latitude
        transformer = Transformer.from_crs(proj_WGS84, proj_UTMK)
        x,y = transformer.transform(lon,lat)
        # self.sub_cood = [x, y]

        self.pos2 = [x, y]
        self.GPS_VECOCITY_Heading = np.arctan2(self.pos2[1] - self.last_pos[1],self.pos2[0] - self.last_pos[0])

        if self.GPS_VECOCITY_Heading  < 0:
            self.GPS_VECOCITY_Heading  += 2 * np.pi

        self.last_pos = self.pos2 
        self.GPS_INPUT_TIME = time.time()    

    def erp_callback(self, data):
        self.steer = (data.read_steer)/71
        self.enc = data.read_ENC
        self.erp_INPUT_TIME = time.time()

    def heading_callback(self, head):
        self.sub_gps_heading = float(head.heading)
        #######################################
        self.GPS_DATA_dT = abs(self.GPS_INPUT_TIME - self.GPS_DATA_LAST_TIME)
        velocity_ms = head.gSpeed / 1000
        self.GPS_DATA_velocity_kmh = velocity_ms * 3.6
        ####################################################
        if self.GPS_DATA_velocity_kmh < 0.3 :
            self.GPS_move_flag = 0
        else:  
            self.GPS_move_flag = 1  
        ####################################################
        if self.GPS_DATA_dT != 0:
            self.GPS_DATA_accel = (self.GPS_DATA_velocity_kmh-self.GPS_DATA_LAST_velocity_kmh) / self.GPS_DATA_dT / 3.6

        if self.GPS_move_flag == 1:
            pos_heading_degrees = head.heading / 1e5
            self.GPS_DATA_pos_heading_radians = np.radians(pos_heading_degrees)
            self.GPS_DATA_pos_heading_radians = -self.GPS_DATA_pos_heading_radians

        self.GPS_DATA_LAST_TIME = self.GPS_INPUT_TIME 
        self.GPS_DATA_LAST_velocity_kmh = self.GPS_DATA_velocity_kmh
        
    def scan_callback(self, scan):
        self.sub_scan = scan.ranges
        self.lidar_flag = True

    def scan_yd_callback(self, scan):
        self.sub_scan_yd = scan.ranges
        self.lidar_flag = True

    # gps, imu, lidar 가 모두 작동해서 데이터가 들어왔는지 확인
    def senser_check(self):
        return self.gps_flag and self.imu_flag and self.lidar_flag

    ##########update 함수 모음############
    def localization_update(self, select_heading):
        x, y = self.loc_fusion.tf_to_tm(self.sub_cood[0], self.sub_cood[1])
        # heading = self.loc_fusion.get_heading(x, y, self.sub_imu, self.sub_gps_heading, select_heading) # 지금은 그냥 gps 헤딩을 그대로 쓰지만, imu가 추가된 해딩을 처리하는 클래스가 필요.
        self.pos.x = x #955920.9088
        self.pos.y = y #1950958.212
        self.pos.z = self.imu_yaw

    def object_update(self):
        self.obs = PointCloud()
        self.Lidar.tf_tm(self.sub_scan, self.pos.x , self.pos.y, self.pos.z) # 들어온 점을 tm좌표로 변환
        obs_clean = self.Lidar.clean() #격자화
        # obs_clean = [[955920.0, 1950958.0],[955921.0, 1950958.0],[955919.0, 1950958.0],[955921.0, 1950959.0],[955922.0, 1950960.0],[955918.0, 1950958.0],[955920.0, 1950960.0]]
        
        for i in obs_clean:
            p = Point32()
            p.x = i[0]
            p.y = i[1]
            p.z = 0
            self.obs.points.append(p)
    ########yd lidar tf좌표 저장#########
    def object_update_back(self):
        self.obs_back = PointCloud()
        obs_back = self.Lidar.tf_tm_yd(self.sub_scan_yd) 

        for i in obs_back:
            p = Point32()
            p.x = i[0]
            p.y = i[1]
            p.z = 0
            self.obs_back.points.append(p)
    ######################################
    # def rubber_len_check(self):
    #     self.object_update()
    #     self.rub_len = 

    #     return to_len
    ##########publish 함수 모음############
    def pub_pose(self):
        self.localization_pub.publish(self.pos)

    def pub_obs(self):
        self.obs_pub.publish(self.obs)

    def pub_speed(self):
        self.speed_pub.publish(self.GPS_DATA_velocity_kmh)

    def pub_GPS_heading(self):
        self.GPS_heading.publish(self.GPS_DATA_pos_heading_radians)     
    #########
    def pub_obs_back(self):
        self.obs_back_pub.publish(self.obs_back)
    ######################################

def main():
    #기본 설정
    rospy.init_node('data_hub', anonymous=True)
    # rate=rospy.Rate(1) #지금 코드에서는 안써도 됨

    Data = SensorDataHub() # 객체 생성
    start_rate = time.time() # 시간 스템프

    # 센서가 모두 동작하기 전까지는 대기
    # while not Data.senser_check():
    #     print("senser not ready")
    #     rate.sleep()
    #     continue

    while not rospy.is_shutdown():
        if time.time() - start_rate > 0.1: # 0.2초 간격으로 실행. 데이터 처리가 0.3초보다 빨라도 여유롭게 0.3초간격으로 실행하고, 더 늦으면 업데이트 되는대로 실행.
            # print("rate : "),
            # print(int((time.time() - start_rate)*1000)),
            # print("(ms), "),

            start_rate = time.time() # 시작 시간 스템프 찍고
            # 각 정보 업데이트 후 발행
            Data.localization_update(2)
            # Data.object_update()
            # Data.object_update_back()
            Data.pub_pose()
            Data.pub_obs()
            Data.pub_speed()
            Data.pub_GPS_heading()
            #Data.pub_obs_back()
            # print(Data.pos)

            # print("Processing time : "),
            # print(int((time.time() - start_rate)*1000)),
            # print("(ms)")

    start = 5   
    plt.figure(1)
    # plt.plot(Data.count_array[start:], Data.GPS_heading_array[start:], label='GPS', color='red')
    plt.plot(Data.count_array[start:], Data.IMU_heading_array[start:], label='IMU', color='blue')
    plt.plot(Data.count_array[start:], Data.GPS_DATA_heading_array[start:], label='GPS_Data', color='orange')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('HEADING')
    plt.show()
   

   

if __name__ == '__main__':
    main()