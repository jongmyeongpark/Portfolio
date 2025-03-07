#!/usr/bin/env python3
#-*-coding:utf-8-*-
import rospy
import time
import math
import numpy as np
##sdf
from macaron_06.msg import erp_read
from std_msgs.msg import Float64,Float32
from sensor_msgs.msg import LaserScan, NavSatFix, PointCloud, Imu, MagneticField
from geometry_msgs.msg import Point, Point32, Quaternion

from ublox_msgs.msg import NavPVT
import matplotlib.pyplot as plt



from location import gps_imu_fusion
# from lidar import lidar

from tf.transformations import euler_from_quaternion

from pyproj import Proj, Transformer, CRS

proj_UTMK = CRS(init='epsg:5179')
proj_WGS84 = CRS(init='epsg:4326')

TIME = 0.05 # HZ 설정 => HZ = 1/시간

class SensorDataHub:
    def __init__(self):
        #구독자 선언
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size = 1)
        self.yd_laser_sub = rospy.Subscriber('/scan2', LaserScan, self.scan_yd_callback, queue_size = 1)
        self.sub_gps = rospy.Subscriber('/smc_2000/fix', NavSatFix, self.pose_callback, queue_size=1)
        self.sub_heading = rospy.Subscriber('/smc_2000/navpvt', NavPVT, self.heading_callback, queue_size=1)
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
        # self.Lidar = lidar()

        #flag 선언
        self.lidar_flag = False
        self.gps_flag = False
        self.imu_flag = False
        
        #Sub 받은 데이터 저장 공간
        self.sub_cood = [0.0, 0.0]
        self.sub_gps_heading = 0.0
        
        #obs_pub에 사용되는 msg 객체 선언
        self.pos = Point()
        self.pos_xy = [0,0]

        self.pos_flag = False
        self.pos_move = False

        self.GPS_init = 0
        self.GPS_now_time = time.time()
        self.GPS_last_time = time.time()
        self.GPS_dt = 0

        self.GPS_velocity = 0 #속도
        self.velocity_ms = 0
        self.GPS_heading = 0

        self.GPS_VECOCITY_Heading = 0
        self.last_pos = [0,0]

        self.imu_yaw = 0.0
        self.yaw = 0

        self.heading = 0

        self.GPS_plot_x,self.GPS_plot_y =  [],[]
        self.GPS_plot_x_raw,self.GPS_plot_y_raw =  [],[]

    ##########callback 함수 모음##########
    # 각 센서에서 데이터가 들어오면 객체 내부의 데이터 저장공간에 저장

    def imu_callback(self, imu):
        orientation = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
        _,_,yaw = euler_from_quaternion(orientation)
        if yaw >= 2*np.pi: yaw -= 2*np.pi
        if yaw < 0: yaw += 2*np.pi
        self.imu_yaw = yaw

        self.sub_imu = imu.orientation
        self.imu_flag = True

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

        if self.GPS_init == 0:
            self.GPS_last_time = time.time()
            self.GPS_init = 1 

        self.GPS_now_time = time.time()
        self.GPS_dt = abs(self.GPS_last_time - self.GPS_now_time)
        self.GPS_last_time = time.time()
        
        # self.GPS_plot_x_raw.append(x)
        # self.GPS_plot_y_raw.append(y)
 
        self.pos_flag = True
        # self.sub_cood = [x, y]

        self.pos_xy  = [x, y]

    def erp_callback(self, data):
        self.steer = (data.read_steer)/71
        self.enc = data.read_ENC
        self.erp_INPUT_TIME = time.time()

    def heading_callback(self,nav_msg):
        self.velocity_ms = nav_msg.gSpeed / 1000
        if self.velocity_ms >= 5.6: #erp최대 속도 20km => 5.56m/s
            self.velocity_ms = 5.6
        self.GPS_velocity = self.velocity_ms * 3.6
        self.GPS_heading = np.radians(nav_msg.heading / 1e5)

        if self.GPS_velocity > 0.5:
            self.pos_move = True
        else:
            self.pos_move = False

        if self.pos_move == True:    
            self.sub_gps_heading = float(nav_msg.heading)

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
        # x, y = self.loc_fusion.tf_to_tm(self.sub_cood[0], self.sub_cood[1])
        x,y = self.pos_xy[0],self.pos_xy[1]
        if self.imu_flag != False:
            self.heading = self.loc_fusion.get_heading(x, y, self.sub_imu, self.sub_gps_heading, select_heading) # 지금은 그냥 gps 헤딩을 그대로 쓰지만, imu가 추가된 해딩을 처리하는 클래스가 필요.
        self.pos.x = x #955920.9088
        self.pos.y = y #1950958.212
        self.pos.z = self.heading
        # print("heading",heading,self.GPS_heading,self.imu_yaw)

        print(x,y,self.pos_flag)
        self.pos_flag = False

        # self.GPS_plot_x.append(x)
        # self.GPS_plot_y.append(y)

        
       
    def Dead_reckoning(self):
        dt = 0
        if self.GPS_dt != 0:
            div = int(abs(((1/TIME)) - ((1/self.GPS_dt)))) + 1
        else:
            div = 2    
        if self.pos_flag != True and self.pos_move == True: #GPS신호가 업데이트 되기 전이고, 속도가 0이 아닐때
            dt = self.GPS_dt / div
            print(self.GPS_dt,dt,div)
            self.predict_pos_x = np.cos(self.heading) * self.velocity_ms * dt
            self.predict_pos_y = np.sin(self.heading) * self.velocity_ms * dt

            max_predict_distance = 20/3.6*1/div #해당 HZ에서 최고속도일때 갈 수 있는 최대 거리
            print("예측값",self.predict_pos_x,self.predict_pos_y,max_predict_distance)
            if abs(self.predict_pos_x) < max_predict_distance and abs(self.predict_pos_y) < max_predict_distance:
                self.pos_xy[0] += self.predict_pos_x
                self.pos_xy[1] += self.predict_pos_y
        

           

    # def object_update(self):
    #     self.obs = PointCloud()
    #     self.Lidar.tf_tm(self.sub_scan, self.pos.x , self.pos.y, self.pos.z) # 들어온 점을 tm좌표로 변환
    #     obs_clean = self.Lidar.clean() #격자화
    #     # obs_clean = [[955920.0, 1950958.0],[955921.0, 1950958.0],[955919.0, 1950958.0],[955921.0, 1950959.0],[955922.0, 1950960.0],[955918.0, 1950958.0],[955920.0, 1950960.0]]
        
    #     for i in obs_clean:
    #         p = Point32()
    #         p.x = i[0]
    #         p.y = i[1]
    #         p.z = 0
    #         self.obs.points.append(p)
    # ########yd lidar tf좌표 저장#########
    # def object_update_back(self):
    #     self.obs_back = PointCloud()
    #     obs_back = self.Lidar.tf_tm_yd(self.sub_scan_yd) 

    #     for i in obs_back:
    #         p = Point32()
    #         p.x = i[0]
    #         p.y = i[1]
    #         p.z = 0
    #         self.obs_back.points.append(p)
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
        self.speed_pub.publish(self.GPS_velocity)

    def pub_GPS_heading(self):
        # self.GPS_heading.publish(self.GPS_heading_radians)     
        print("data X")
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
        if time.time() - start_rate > TIME: # 0.2초 간격으로 실행. 데이터 처리가 0.3초보다 빨라도 여유롭게 0.3초간격으로 실행하고, 더 늦으면 업데이트 되는대로 실행.
            # print("rate : "),
            # print(int((time.time() - start_rate)*1000)),
            # print("(ms), "),

            start_rate = time.time() # 시작 시간 스템프 찍고
            # 각 정보 업데이트 후 발행
            Data.localization_update(2)
            Data.Dead_reckoning()
            # Data.object_update()
            # Data.object_update_back()
            Data.pub_pose()
            Data.pub_speed()
            # rospy.loginfo(f'yaw: {Data.imu_yaw}')
            #Data.pub_obs_back()
            # print(Data.pos)

            # print("Processing time : "),
            # print(int((time.time() - start_rate)*1000)),
            # print("(ms)")
    
    
    # plt.figure(1)
    # plt.plot(Data.GPS_plot_x[100:], Data.GPS_plot_y[100:], label='GPS_POS', color='red')
    # plt.plot(Data.GPS_plot_x_raw[100:], Data.GPS_plot_y_raw[100:], label='GPS_POS_raw', color='blue')
    # plt.xlabel('X')
    # plt.ylabel('Y')
    # plt.title('GPS_data')
    # plt.show()

   

   

if __name__ == '__main__':
    main()